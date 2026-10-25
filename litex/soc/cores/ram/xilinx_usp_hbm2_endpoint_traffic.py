"""Application-side traffic and data checking for the ordered HBM endpoint.

The default region is PC31 local 0x200000..0x3fffff. Every command has 16 native
beats and eight 512-bit application words. SEED writes the whole region once;
WRITE, READ and MIXED offer commands for a captured number of SYS clocks. READ
and MIXED require a completed SEED with the same seed. A mixed write stores the
same address-dependent value as SEED, so either read/write ordering is valid.

STOP, time expiry and READY loss close new offers. An already offered AW/AR
remains stable until accepted; its data and all responses finish normally.
The board must use endpoint_quiesce and retain native admission until drained.
Elapsed cycles include draining, and latency measures application acceptance
through B or final packed R consumption. Counters are 32-bit, saturate on
overflow and set an error; byte totals belong in 64-bit firmware/host arithmetic.
IDs are captured seed bits 5:0 XOR 0x15 for writes and XOR 0x2a for reads;
they remain constant within a run, preserving the endpoint's one-ID lock.
"""

from migen import Cat, If, Mux, Signal

from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStatus, CSRStorage


class HBMEndpointTrafficCSR(LiteXModule):
    def __init__(self, endpoint, *, region_bursts=4096):
        if endpoint.max_outstanding != 4 or len(endpoint.aw.addr) != 33 or len(endpoint.aw.id) != 6:
            raise ValueError("Traffic requires the four-command address33/ID6 endpoint")
        if region_bursts < 1 or region_bursts > 4096 or region_bursts & (region_bursts - 1):
            raise ValueError("The region must contain a power of two from 1 to 4096 bursts")
        self.admission_enable = Signal()
        self.stop = Signal()
        self.endpoint_quiesce = Signal()
        self.pending = Signal()
        self.drained = Signal()
        self.idle = self.drained
        self.quiesced = Signal()
        self.busy = busy = Signal()
        self._csrs = []

        def storage(name, width):
            value = CSRStorage(width, name=name)
            setattr(self, name, value)
            self._csrs.append(value)
            return value

        def status(name, width):
            value = CSRStatus(width, name=name)
            setattr(self, name, value)
            self._csrs.append(value)
            return value

        for name, width in (("config_mode", 2), ("config_cycles", 32), ("config_seed", 32),
                            ("start", 1), ("clear_errors", 7)):
            storage(name, width)
        status("status", 32)
        status("errors", 7)
        counter_names = ("cycles", "write_beats", "read_beats", "write_bursts", "read_bursts",
            "aw_count", "ar_count", "packed_w", "packed_r", "aw_stalls", "w_stalls",
            "ar_stalls", "read_wait")
        counters = {name:Signal(32) for name in counter_names}
        for name in counter_names:
            self.comb += status(name, 32).status.eq(counters[name])
        for name in ("write_latency_min", "write_latency_max", "read_latency_min", "read_latency_max"):
            status(name, 32)

        # # #

        mode = Signal(2)
        duration = Signal(32)
        seed = Signal(32)
        seeded = Signal()
        seeded_value = Signal(32)
        seed_allowed = Signal()
        done = Signal()
        aw_offer, ar_offer = Signal(), Signal()
        aw_index, ar_index = Signal(12), Signal(12)
        aw_address, ar_address = Signal(33), Signal(33)
        write_index, read_index = Signal(3), Signal(3)
        write_outstanding, read_outstanding = Signal(3), Signal(3)
        issue = Signal()
        closing = Signal()
        stop_pending = Signal()
        work_pending = Signal()
        start_request = self.start.re & self.start.storage
        start_accept = Signal()
        finish = Signal()
        overflow = Signal()
        fatal = self.errors.status & 0x6f
        elapsed = counters["cycles"]
        config_valid = ((self.config_mode.storage == 0) |
            ((self.config_cycles.storage != 0) & (self.config_cycles.storage <= (1 << 28))))
        seed_valid = ((self.config_mode.storage < 2) |
            (seeded & (self.config_seed.storage == seeded_value)))
        self.comb += [
            start_accept.eq(start_request & ~busy & self.drained & self.admission_enable &
                ~self.stop & endpoint.link_ready & ~endpoint.protocol_error &
                (self.errors.status == 0) & config_valid & seed_valid),
            closing.eq(stop_pending | self.stop | ~self.admission_enable | (fatal != 0) |
                ((mode != 0) & (elapsed >= duration)) |
                ((mode == 0) & (counters["aw_count"] == region_bursts))),
            issue.eq(busy & ~closing),
        ]
        self.write_queue = writes = stream.SyncFIFO([("addr", 33)], 4)
        self.write_times = write_times = stream.SyncFIFO([("time", 32)], 4)
        self.read_queue = reads = stream.SyncFIFO([("addr", 33), ("time", 32)], 4)
        aw_fire = endpoint.aw.valid & endpoint.aw.ready
        ar_fire = endpoint.ar.valid & endpoint.ar.ready
        w_fire = endpoint.w.valid & endpoint.w.ready
        b_fire = endpoint.b.valid & endpoint.b.ready
        r_fire = endpoint.r.valid & endpoint.r.ready
        r_last = r_fire & endpoint.r.last
        self.comb += [
            endpoint.aw.valid.eq(aw_offer), endpoint.aw.addr.eq(aw_address),
            endpoint.aw.len.eq(15), endpoint.aw.id.eq(seed[:6] ^ 0x15),
            endpoint.ar.valid.eq(ar_offer), endpoint.ar.addr.eq(ar_address),
            endpoint.ar.len.eq(15), endpoint.ar.id.eq(seed[:6] ^ 0x2a),
            writes.sink.valid.eq(aw_fire), writes.sink.addr.eq(aw_address),
            write_times.sink.valid.eq(aw_fire), write_times.sink.time.eq(elapsed),
            reads.sink.valid.eq(ar_fire), reads.sink.addr.eq(ar_address), reads.sink.time.eq(elapsed),
            endpoint.w.valid.eq(writes.source.valid),
            endpoint.w.strb.eq((1 << 64) - 1),
            endpoint.w.first.eq(write_index == 0), endpoint.w.last.eq(write_index == 7),
            writes.source.ready.eq(w_fire & (write_index == 7)),
            endpoint.b.ready.eq(1), endpoint.r.ready.eq(1),
            write_times.source.ready.eq(b_fire & write_times.source.valid),
            reads.source.ready.eq(r_last & reads.source.valid),
            self.endpoint_quiesce.eq(self.stop & ~aw_offer & ~ar_offer),
        ]
        write_word = Signal(32)
        read_word = Signal(32)
        self.comb += [write_word.eq((writes.source.addr >> 2) | (write_index << 4)),
                      read_word.eq((reads.source.addr >> 2) | (read_index << 4))]
        mismatch = Signal()
        bad_data = 0
        for lane in range(16):
            self.comb += endpoint.w.data[32*lane:32*(lane + 1)].eq(seed ^ write_word ^ lane)
            bad_data = bad_data | (endpoint.r.data[32*lane:32*(lane + 1)] != (seed ^ read_word ^ lane))
        self.comb += mismatch.eq(bad_data)
        read_id = seed[:6] ^ 0x2a
        metadata = ((endpoint.r.keep != (1 << 64) - 1) | (endpoint.r.first != (read_index == 0)) |
            (endpoint.r.last != (read_index == 7)) | (endpoint.r.id != Cat(read_id, read_id)))
        unexpected = ((b_fire & ~write_times.source.valid) | (r_fire & ~reads.source.valid))
        failures = Cat(r_fire & reads.source.valid & mismatch,
            r_fire & reads.source.valid & metadata,
            (b_fire & ((endpoint.b.id != (seed[:6] ^ 0x15)) | (endpoint.b.resp != 0))) |
                (r_fire & (endpoint.r.resp != 0)),
            unexpected, start_request & ~start_accept, endpoint.protocol_error, overflow)
        clear = Signal(7)
        self.comb += clear.eq(Mux(self.clear_errors.re & ~busy, self.clear_errors.storage, 0))
        self.sync += self.errors.status.eq((self.errors.status & ~clear) | failures)
        self.comb += [
            work_pending.eq(aw_offer | ar_offer | writes.source.valid | write_times.source.valid |
                reads.source.valid | (write_outstanding != 0) | (read_outstanding != 0) | ~endpoint.drained),
            finish.eq(busy & closing & ~work_pending),
            self.pending.eq(busy | work_pending),
            self.drained.eq(~self.pending & ~endpoint.protocol_error),
            self.quiesced.eq(self.drained & endpoint.quiesced),
            self.status.status.eq(Cat(busy, done, seeded, self.admission_enable, self.stop,
                self.drained, self.quiesced, endpoint.link_ready, endpoint.protocol_error)),
        ]
        self.sync += [
            If(start_accept,
                mode.eq(self.config_mode.storage), duration.eq(self.config_cycles.storage),
                seed.eq(self.config_seed.storage), busy.eq(1), done.eq(0), seed_allowed.eq(1),
                stop_pending.eq(0),
                aw_index.eq(0), ar_index.eq(0), write_index.eq(0), read_index.eq(0),
                If((self.config_mode.storage == 0) |
                    ((self.config_mode.storage == 1) & (self.config_seed.storage != seeded_value)), seeded.eq(0))),
            If(~self.admission_enable | self.stop | (fatal != 0), seeded.eq(0), seed_allowed.eq(0)),
            If(busy & closing, stop_pending.eq(1)),
            If(issue & (mode != 2) & ~aw_offer & writes.sink.ready & write_times.sink.ready,
                aw_offer.eq(1), aw_address.eq((31 << 28) + 0x200000 + (aw_index << 9))),
            If(issue & ((mode == 2) | (mode == 3)) & ~ar_offer & reads.sink.ready,
                ar_offer.eq(1), ar_address.eq((31 << 28) + 0x200000 + (ar_index << 9))),
            If(aw_fire, aw_offer.eq(0),
                If(aw_index == region_bursts - 1, aw_index.eq(0)).Else(aw_index.eq(aw_index + 1))),
            If(ar_fire, ar_offer.eq(0),
                If(ar_index == region_bursts - 1, ar_index.eq(0)).Else(ar_index.eq(ar_index + 1))),
            If(w_fire, write_index.eq(write_index + 1)),
            If(r_fire, If(endpoint.r.last, read_index.eq(0)).Else(read_index.eq(read_index + 1))),
            If(finish,
                busy.eq(0), done.eq(1),
                If((mode == 0) & (counters["write_bursts"] == region_bursts) &
                    (fatal == 0) & seed_allowed & self.admission_enable & ~self.stop,
                    seeded.eq(1), seeded_value.eq(seed))),
        ]
        for count, take, retire in ((write_outstanding, aw_fire, b_fire),
                                     (read_outstanding, ar_fire, r_last)):
            self.sync += If(take & ~retire, count.eq(count + 1)).Elif(retire & ~take,
                If(count != 0, count.eq(count - 1)))

        events = {"cycles": (busy, 1), "aw_count": (aw_fire, 1), "ar_count": (ar_fire, 1),
            "write_beats": (w_fire, 2), "read_beats": (r_fire, 2),
            "packed_w": (w_fire, 1), "packed_r": (r_fire, 1),
            "write_bursts": (b_fire, 1), "read_bursts": (r_last, 1),
            "aw_stalls": (endpoint.aw.valid & ~endpoint.aw.ready, 1),
            "w_stalls": (endpoint.w.valid & ~endpoint.w.ready, 1),
            "ar_stalls": (endpoint.ar.valid & ~endpoint.ar.ready, 1),
            "read_wait": ((read_outstanding != 0) & ~r_fire, 1)}
        # Overflow is an event; the CSR holds its sticky error. A retained
        # event level would prevent an idle W1C from clearing that error.
        self.sync += overflow.eq(0)
        for name, (event, step) in events.items():
            count = counters[name]
            self.sync += If(start_accept, count.eq(0)).Elif(busy & event,
                If(count <= 0xffffffff - step, count.eq(count + step)).Else(count.eq(0xffffffff), overflow.eq(1)))
        for direction, event, valid, timestamp in (
                ("write", b_fire, write_times.source.valid, write_times.source.time),
                ("read", r_last, reads.source.valid, reads.source.time)):
            minimum, maximum = Signal(32, reset=0xffffffff), Signal(32)
            latency = Signal(32)
            self.comb += [latency.eq(elapsed - timestamp),
                getattr(self, direction + "_latency_min").status.eq(
                    Mux(counters[direction + "_bursts"] == 0, 0, minimum)),
                getattr(self, direction + "_latency_max").status.eq(maximum)]
            self.sync += If(start_accept, minimum.eq(0xffffffff), maximum.eq(0)).Elif(busy & event & valid,
                If(latency < minimum, minimum.eq(latency)), If(latency > maximum, maximum.eq(latency)))

    def get_csrs(self):
        return self._csrs
