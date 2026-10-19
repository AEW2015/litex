"""Software-only FK33 cold initialization; no XCI initializer is executed.

Transport contract: read(stack, raw_apb_address), write(stack, address, value),
block(), commit_ready(), health(). Calls must themselves have bounded timeouts.
health() must return trip=False and temperature_c < 60, from fresh telemetry.
An exception always blocks application access, including after partial success.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

PROFILE = Path(__file__).with_name("fk33_hbm_profile.json")


def validate_address(stack, address):
    if stack not in (0, 1) or not isinstance(address, int) or not 0 <= address < (1 << 22) or address % 4:
        raise ValueError("Expected stack 0/1 and aligned raw 22-bit APB byte address")


def legacy_address_to_apb(address):
    """Explicit conversion for imported legacy addresses, never silent truncation."""
    if not isinstance(address, int) or address < 0:
        raise ValueError("Invalid legacy address")
    stack = (address >> 28) - 1
    raw = address - ((stack + 1) << 28)
    validate_address(stack, raw)
    return stack, raw


class InitError(RuntimeError):
    pass


class ColdInitializer:
    def __init__(self, transport, trace, profile=PROFILE, *, sleep=time.sleep,
                 monotonic=time.monotonic, poll_deadline_s=30.0):
        self.transport = transport
        self.profile = json.loads(Path(profile).read_text())
        self.trace = trace
        self.sleep = sleep
        self.clock = monotonic
        self.poll_deadline_s = poll_deadline_s
        if poll_deadline_s <= 0:
            raise ValueError("Poll deadline must be positive")
        self.step = "not_started"
        p = self.profile
        if (p['reference_mhz'], p['hbm_mhz'], p['stacks'], p['regions_per_stack']) != (100, 900, 2, 8):
            raise InitError("Only reviewed FK33 100/900 MHz two-stack profile is supported")
        for key, count in [('ms_pre', 11), ('pll', 16), ('ms_post', 3), ('phy', 9), ('mc', 43)]:
            if len(p[key]) != count:
                raise InitError(f"Incomplete {key} register table")
        if any(x['address'] == 0x18 for x in p['phy']):
            raise InitError("PHY06 drive-strength write is forbidden")
        for key in ('ms_pre', 'pll', 'ms_post', 'phy', 'mc'):
            for x in p[key]:
                validate_address(0, x['address'])
                if not 0 <= x['value'] <= 0xffffffff:
                    raise InitError("Register value outside 32 bits")

    def log(self, operation, **fields):
        self.trace.write(json.dumps({'monotonic_s': self.clock(), 'step': self.step,
                                     'operation': operation, **fields}) + '\n')
        self.trace.flush()

    def health(self):
        h = self.transport.health()
        self.log('health', **h)
        if h.get('trip') is not False or not 0 <= h.get('temperature_c', 1000) < 60:
            raise InitError("Trip asserted, telemetry invalid, or temperature limit reached")

    def read(self, stack, address):
        validate_address(stack, address)
        value = self.transport.read(stack, address)
        if not isinstance(value, int) or not 0 <= value <= 0xffffffff:
            raise InitError("Invalid APB read result")
        self.log('read', stack=stack, address=address, value=value)
        return value

    def write(self, stack, address, value):
        validate_address(stack, address)
        self.log('write_start', stack=stack, address=address, value=value)
        self.transport.write(stack, address, value)
        self.log('write_complete', stack=stack, address=address, value=value)

    def table(self, name, stack, offset=0):
        self.step = f'stack{stack}.{name}.offset{offset:x}'
        self.health()
        for x in self.profile[name]:
            self.write(stack, offset + x['address'], x['value'])

    def poll(self, stack, address, mask, expected):
        start = self.clock()
        for attempt in range(128):
            self.health()
            if self.clock() - start >= self.poll_deadline_s:
                break
            value = self.read(stack, address)
            if self.clock() - start >= self.poll_deadline_s:
                break
            if value & mask == expected:
                self.log('poll_complete', stack=stack, address=address, attempts=attempt + 1,
                         elapsed_s=self.clock() - start, value=value, mask=mask, expected=expected)
                return value
            if attempt != 127:
                self.sleep(.001)
        raise InitError(f"{self.step}: poll timeout at stack {stack} APB {address:#x}")

    def temperature(self, stack):
        self.step = f'stack{stack}.temperature'
        self.write(stack, 0x240004, 0x00a0800f)
        self.poll(stack, 0x240004, 0x00ffffff, 0x0020800f)
        raw = self.read(stack, 0x24000c)
        temperature = (raw >> 24) & 127
        self.log('temperature', stack=stack, raw=raw, valid=not bool(raw & 0x80000000), temperature_c=temperature)
        if raw & 0x80000000 or temperature >= 60:
            raise InitError("HBM full-temperature read invalid or above limit")
        return temperature

    def run(self):
        p = self.profile
        results = {'status': 'FAIL', 'controllers': [], 'temperature_c': []}
        self.transport.block()
        self.log('blocked')
        try:
            self.health()
            # Preserve actual drive strength; Wolf explicitly omits PHY06 writes.
            phy06 = {}
            for stack in range(2):
                for region in range(8):
                    self.step = f'stack{stack}.region{region}.preserve_phy06'
                    raw = self.read(stack, region * 0x20000 + 0x18)
                    phy06[stack, region] = raw
            for stack in range(2):
                self.step = f'stack{stack}.reset_controllers'
                for region in range(8):
                    self.write(stack, region * 0x20000 + 0x104000, 0)
                self.sleep(1)  # Wolf's explicit reset settling interval.
                self.table('ms_pre', stack)
                self.table('pll', stack)
                self.table('ms_post', stack)
                for region in range(8):
                    self.table('phy', stack, region * 0x20000)
                    self.table('mc', stack, region * 0x20000)
                for region in range(8):
                    offset = region * 0x20000
                    self.step = f'stack{stack}.region{region}.dfi_init'
                    self.write(stack, offset + 0x110050, 1)
                    self.poll(stack, offset + 0x110034, 1, 1)
                    self.write(stack, offset + 0x104000, 1)
                    self.write(stack, offset + 0x11000c, 1)
                for region in range(8):
                    offset = region * 0x20000
                    self.step = f'stack{stack}.region{region}.calibration'
                    status = self.poll(stack, offset + 0x10, p['calibration_status_mask'], p['calibration_status_expected'])
                    words = []
                    for selector in p['selectors']:
                        self.write(stack, offset + 0x54, selector)
                        raw = self.read(stack, offset + 0x60)
                        words.append(raw)
                        if raw & p['calibration_result_mask'] != p['calibration_result_expected']:
                            raise InitError(f"Invalid calibration window for stack {stack} region {region} selector {selector:#x}: {raw:#x}")
                    results['controllers'].append({'stack': stack, 'apb_region': region,
                        'physical_label': p['physical_label_by_region'][region], 'status_raw': status,
                        'valid_windows_raw': words, 'success': True})
                temperature = self.temperature(stack)
                self.transport.set_temperature(stack, temperature)
                results['temperature_c'].append(temperature)
                for region in range(8):
                    self.step = f'stack{stack}.region{region}.verify_phy06'
                    raw = self.read(stack, region * 0x20000 + 0x18)
                    if raw != phy06[stack, region]:
                        raise InitError("PHY06 changed during initialization; drive strength was not preserved")
            self.step = 'commit_ready'
            # Stack 0 may have waited while stack 1 trained; refresh both now.
            for stack in range(2):
                temperature = self.temperature(stack)
                self.transport.set_temperature(stack, temperature)
                results['temperature_c'][stack] = temperature
            self.step = 'commit_ready'
            self.health()
            self.transport.commit_ready()
            self.log('ready')
            results['status'] = 'PASS'
            return results
        except Exception as exc:
            results.update(error=str(exc), failing_step=self.step)
            self.log('failure', **results)
            try:
                self.transport.block()
                self.log('blocked_after_failure')
            except Exception as block_error:
                self.log('block_error', error=str(block_error))
            raise
