"""Exclusive-owner native HBM burst exercisers; no APB, clocks or READY changes.

Start only while processor firmware is READY with no pending memory command.
Hardware idle is a transaction boundary, not a processor pause acknowledgement.
A rejected CPU Wishbone cycle remains blocked by the target admission gate until
CYC drops. STOP/quiesce finishes the current write/masked-write/read unit.
Counters count accepted AXI beats (32 bytes of bus payload, not enabled bytes).
"""
from pathlib import Path
from migen import Cat, ClockSignal, If, Instance, Mux, ResetSignal, Signal
from migen.genlib.record import DIR_M_TO_S
from litex.gen import LiteXModule
from litex.soc.interconnect import axi, stream
from litex.soc.interconnect.csr import CSRStatus, CSRStorage


class HBMBurstBIST(LiteXModule):
    CHANNELS = tuple(range(32))

    def __init__(self, platform, ready, quiesce_request, all_drained, host_idle):
        path = Path(__file__).resolve().parent / "xilinx_usp_hbm2" / "fk33_hbm_bist.sv"
        platform.add_source(str(path), language="systemverilog")
        self.ready, self.quiesce_request = ready, quiesce_request
        self.all_drained, self.host_idle = all_drained, host_idle
        self.owner = Signal()
        self.release_owner = Signal()
        self.local_owners = {}
        self.csr_read_pipeline = True
        self.dispatch_pending = Signal()
        self.busy_mask, self.done_mask = Signal(32), Signal(32)
        self.stopped_mask, self.error_mask = Signal(32), Signal(32)
        self.accepted_start, self.start_rejected, self.config_valid = Signal(), Signal(), Signal()
        self._ports = {}
        self._port_csrs = {}
        self.engines, self.engine_status = {}, {}
        self.request_buffers, self.response_buffers = {}, {}
        self._csrs = []
        def storage(name, width, reset=0):
            value = CSRStorage(width, reset=reset, name=name)
            setattr(self, name, value); self._csrs.append(value)
            return value
        def status(name, width, attribute=None):
            value = CSRStatus(width, name=name)
            setattr(self, attribute or name, value); self._csrs.append(value)
            return value
        storage("start", 1)
        storage("stop", 1)
        storage("mask", 32, 3)
        storage("base", 28, 0x100000)
        storage("burst_beats", 5, 16)
        storage("burst_count", 32, 1)
        storage("seed", 32, 0x94660000)
        storage("strobe", 32, 0xffffffff)
        storage("stall", 3)  # bits: W gaps, R backpressure, B backpressure.
        storage("masked", 1)
        status("status", 6)
        status("busy_mask", 32, "_busy_csr")
        status("done_mask", 32, "_done_csr")
        status("stopped_mask", 32, "_stopped_csr")
        status("error_mask", 32, "_error_csr")
        status("reject", 2)
        status("elapsed", 64)
        status("overlap_cycles", 64)
        status("transfer_mask", 32)
        status("active_mask", 32)
        self._active = Signal(32)
        self._stop_pending = Signal()
        self._elapsed, self._overlap = Signal(64), Signal(64)
        self._transfer = Signal(32)
        self._transfer_r = Signal(32)
        # Snapshot validated inputs before the registered engine dispatch. Host
        # CSR writes while owned cannot change an already accepted operation.
        self._config = {name: Signal(len(getattr(self, name).storage)) for name in
            ("mask", "base", "burst_beats", "burst_count", "seed", "strobe", "stall", "masked")}
        self.sync += If(self.accepted_start, *[
            value.eq(getattr(self, name).storage) for name, value in self._config.items()])
        request = self.start.re & self.start.storage
        available = (ready & all_drained & host_idle & ~quiesce_request &
                     ~self.owner & (self.busy_mask == 0))
        # Aligned 512-byte units and 1..16 beats make the final-byte bound
        # equivalent to base_units + count <= 0x80000. Bound all 32 count bits
        # separately before using the narrow sum; invalid high bits cannot wrap.
        end_unit = Signal(21)
        self.comb += [
            end_unit.eq(self.base.storage[9:] + self.burst_count.storage[:20]),
            self.config_valid.eq((self.mask.storage != 0) &
                (self.base.storage[:9] == 0) &
                (self.burst_beats.storage >= 1) & (self.burst_beats.storage <= 16) &
                (self.burst_count.storage >= 1) & (self.burst_count.storage <= 0x80000) &
                (end_unit <= 0x80000)),
            self.accepted_start.eq(request & available & self.config_valid),
            self.release_owner.eq(self.owner & ~self.dispatch_pending &
                (self.busy_mask == 0) & all_drained & host_idle),
            self.start_rejected.eq(request & ~self.accepted_start),
            self._busy_csr.status.eq(self.busy_mask),
            self._done_csr.status.eq(self.done_mask),
            self._stopped_csr.status.eq(self.stopped_mask),
            self._error_csr.status.eq(self.error_mask),
            self.elapsed.status.eq(self._elapsed),
            self.overlap_cycles.status.eq(self._overlap),
            self.transfer_mask.status.eq(self._transfer_r),
            self.active_mask.status.eq(self._active),
            self.status.status.eq(Cat(self.owner, self.busy_mask != 0,
                self.error_mask != 0,
                (self._active != 0) & ((self.done_mask & self._active) == self._active),
                self._stop_pending, self.config_valid)),
        ]
        self.sync += [
            self._transfer_r.eq(self._transfer),
            self.dispatch_pending.eq(0),
            If(self.accepted_start,
                self.owner.eq(1), self.dispatch_pending.eq(1), self._active.eq(self.mask.storage),
                self._stop_pending.eq(0), self._elapsed.eq(0), self._overlap.eq(0),
                self.reject.status.eq(0), self._transfer_r.eq(0),
            ).Else(
                # Global elapsed includes ownership drain overhead, from the
                # accepted start until all engines and native trackers retire.
                If(self.owner, self._elapsed.eq(self._elapsed + 1)),
                If((self._transfer_r & (self._transfer_r - 1)) != 0,
                    self._overlap.eq(self._overlap + 1)),
                If(self.release_owner,
                    self.owner.eq(0)),
                If((self.stop.re & self.stop.storage) | quiesce_request,
                    self._stop_pending.eq(1)),
                If(self.start_rejected,
                    If(self.owner | (self.busy_mask != 0), self.reject.status.eq(1))
                    .Elif(~self.config_valid, self.reject.status.eq(2))
                    .Else(self.reject.status.eq(3))),
            ),
        ]

    def add_port(self, channel, cpu_axi, mux_axi):
        if channel not in self.CHANNELS or channel in self._ports:
            raise ValueError("BIST ports must be unique members of PC0-31")
        for interface in (cpu_axi, mux_axi):
            if (interface.data_width, interface.address_width, interface.id_width) != (256, 33, 6):
                raise ValueError("BIST ownership requires AXI256/address33/ID6")
            if interface.version != "axi4":
                raise ValueError("BIST uses the ordered-write AXI4 shape of the HBM AXI3 subset")
        # Same-edge copies of global ownership, protected from equivalence
        # merging so each native mux has a local select driver.
        local_owner = Signal(name=f"hbm_bist_pc{channel:02d}_owner",
            attr={("DONT_TOUCH", "TRUE")})
        self.local_owners[channel] = local_owner
        self.sync += If(self.accepted_start, local_owner.eq(1)).Elif(
            self.release_owner, local_owner.eq(0))
        engine = axi.AXIInterface(data_width=256, address_width=33, id_width=6)
        self.engines[channel] = engine
        mux_engine = axi.AXIInterface(data_width=256, address_width=33, id_width=6)
        # Slice only the BIST branch; CPU paths retain their existing timing.
        # Independent channels preserve AW/W independence and full payloads.
        self.request_buffers[channel] = {}
        for group in ("aw", "w", "ar"):
            request = getattr(engine, group)
            buffer = stream.Buffer(request.description, pipe_valid=True, pipe_ready=True)
            self.submodules += buffer
            self.request_buffers[channel][group] = buffer
            self.comb += request.connect(buffer.sink)
            self.comb += buffer.source.connect(getattr(mux_engine, group))
        self.response_buffers[channel] = {}
        for group in ("b", "r"):
            response = getattr(mux_engine, group)
            buffer = stream.Buffer(response.description, pipe_valid=True, pipe_ready=True)
            self.submodules += buffer
            self.response_buffers[channel][group] = buffer
            self.comb += response.connect(buffer.sink)
            self.comb += buffer.source.connect(getattr(engine, group))
        metrics = {name: Signal(width, name=f"hbm_bist_pc{channel:02d}_{name}")
            for name, width in (("write_beats", 64), ("read_beats", 64),
                ("cycles", 64), ("errors", 64), ("first_address", 33),
                ("expected", 32), ("actual", 32), ("code", 8))}
        self.engine_status[channel] = metrics
        busy, done, stopped, faulted, transfer = Signal(), Signal(), Signal(), Signal(), Signal()
        self._ports[channel] = busy
        csrs = []
        for name, value in metrics.items():
            csr = CSRStatus(len(value), name=f"pc{channel:02d}_{name}")
            csrs.append(csr); self.comb += csr.status.eq(value)
        self._port_csrs[channel] = csrs
        self.comb += [
            self.busy_mask[channel].eq(busy), self.done_mask[channel].eq(done),
            self.stopped_mask[channel].eq(stopped),
            self.error_mask[channel].eq(faulted),
            # Count concurrent native-side acceptance, before BIST response
            # buffers; engine read consumption can occur a later cycle.
            self._transfer[channel].eq(local_owner &
                ((mux_axi.w.valid & mux_axi.w.ready) |
                 (mux_axi.r.valid & mux_axi.r.ready))),
        ]
        parameters = dict(p_CHANNEL=channel, i_clk=ClockSignal("sys"), i_rst=ResetSignal("sys"),
            i_start=self.dispatch_pending, i_enable=self._config["mask"][channel],
            i_stop=self._stop_pending | self.quiesce_request | ~self.ready,
            i_cfg_base=self._config["base"], i_cfg_beats=self._config["burst_beats"],
            i_cfg_count=self._config["burst_count"], i_cfg_seed=self._config["seed"],
            i_cfg_strobe=self._config["strobe"], i_cfg_stall=self._config["stall"],
            i_cfg_masked=self._config["masked"],
            o_busy=busy, o_done=done, o_stopped=stopped, o_faulted=faulted, o_transfer=transfer,
            o_write_beats=metrics["write_beats"], o_read_beats=metrics["read_beats"],
            o_cycles=metrics["cycles"], o_errors=metrics["errors"],
            o_first_bad_addr=metrics["first_address"],
            o_first_bad_expected=metrics["expected"], o_first_bad_actual=metrics["actual"],
            o_first_bad_code=metrics["code"])
        outputs = {"aw": ("addr", "id", "len", "size", "burst", "valid"),
                   "w": ("data", "strb", "last", "valid"), "b": ("ready",),
                   "ar": ("addr", "id", "len", "size", "burst", "valid"),
                   "r": ("ready",)}
        inputs = {"aw": ("ready",), "w": ("ready",),
                  "b": ("id", "resp", "valid"), "ar": ("ready",),
                  "r": ("id", "data", "resp", "last", "valid")}
        for group, names in outputs.items():
            for name in names: parameters[f"o_m_{group}{name}"] = getattr(getattr(engine, group), name)
        for group, names in inputs.items():
            for name in names: parameters[f"i_m_{group}{name}"] = getattr(getattr(engine, group), name)
        self.specials += Instance("fk33_hbm_bist", **parameters)
        # One owner controls all five channels. No ownership change is possible
        # until both masters and the downstream native tracker are drained.
        for group, name, direction in mux_axi.layout_flat():
            target = getattr(getattr(mux_axi, group), name)
            cpu = getattr(getattr(cpu_axi, group), name)
            bist = getattr(getattr(mux_engine, group), name)
            if direction == DIR_M_TO_S:
                self.comb += target.eq(Mux(local_owner, bist, cpu))
            elif name in ("valid", "ready"):
                self.comb += [cpu.eq(target & ~local_owner), bist.eq(target & local_owner)]
            else:
                self.comb += [cpu.eq(target), bist.eq(target)]
        # Native control fields not exercised by this AXI3 subset.
        for group in (engine.aw, engine.ar):
            for name in ("lock", "cache", "prot", "qos", "region"):
                if hasattr(group, name): self.comb += getattr(group, name).eq(0)
        idle = Signal(name=f"hbm_bist_pc{channel:02d}_idle")
        # Busy persists through terminal response consumption and checking.
        # A native B/R terminal response requires all preceding requests to have
        # retired, so no queued BIST request can survive aggregate drain.
        # Pending dispatch is owned work even before engine busy rises.
        self.comb += idle.eq(~busy & ~self.dispatch_pending)
        return idle

    def get_csrs(self):
        return self._csrs + [csr for channel in sorted(self._port_csrs)
                            for csr in self._port_csrs[channel]]

    def do_finalize(self):
        if set(self._ports) != set(self.CHANNELS):
            raise ValueError("BIST ownership must connect all 32 physical ports")
