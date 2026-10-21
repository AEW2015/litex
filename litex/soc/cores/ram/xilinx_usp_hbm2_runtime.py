"""Admission and AXI drain controls for the FK33 32-port HBM runtime path.

The interposer closes Wishbone admission before conversion and tracks native
AXI transactions for all physical HBM ports. It does not control READY, AXI
resets, APB, or clocks.
"""
from pathlib import Path

from migen import Cat, ClockSignal, If, Instance, ResetSignal, Signal
from litex.gen import LiteXModule
from litex.soc.interconnect.csr import CSRStatus, CSRStorage


class WishboneAdmissionGate(LiteXModule):
    """Retire admitted Wishbone work and block new cycles during quiesce.

    A cycle rejected during quiesce or while HBM is not ready remains blocked
    until the master drops CYC, so release cannot revive a timed-out access.
    """
    def __init__(self, master, slave, ready, quiesce_request):
        if (master.data_width != slave.data_width or
                len(master.adr) != len(slave.adr) or
                master.addressing != slave.addressing):
            raise ValueError("Admission gate requires matching Wishbone interfaces")
        self.active = Signal()
        self.blocked = Signal()
        self.idle = Signal()
        admit = Signal()
        self.comb += [
            admit.eq(ready & (self.active | (~quiesce_request & ~self.blocked))),
            slave.cyc.eq(master.cyc & admit),
            self.idle.eq(~self.active & ~(slave.cyc & master.stb)),
            master.connect(slave, omit={"cyc"}),
        ]
        self.sync += [
            If(~master.cyc,
                self.active.eq(0), self.blocked.eq(0),
            ).Else(
                If(slave.ack | slave.err,
                    self.active.eq(0),
                ).Elif(slave.cyc & master.stb,
                    self.active.eq(1),
                ),
                If(~self.active & master.stb & (~ready | quiesce_request),
                    self.blocked.eq(1),
                ),
            ),
        ]


class HBMRuntimeQuiesce(LiteXModule):
    """CSR bank 8 adapter for all 32 HBM native AXI ports.

    CSR words: request=0, drained[31:0]=1, error[31:0]=2, status=3.
    Status bits are effective_request=0, all_drained=1, any_error=2. External
    faults (such as a rejected bank-selector change) contribute to status bit
    2 and hold admission closed. Per-port AXI accounting faults remain visible
    in the error vector. All faults are sticky until system reset.
    """
    CHANNELS = tuple(range(32))

    def __init__(self, platform, channels=CHANNELS, *, registered_request=False,
                 registered_completions=False):
        if tuple(channels) != self.CHANNELS:
            raise ValueError("Runtime interposer requires native channels 0 through 31")
        self.channels = tuple(channels)
        self.registered_request = bool(registered_request)
        self.registered_completions = bool(registered_completions)
        path = (Path(__file__).resolve().parent / "xilinx_usp_hbm2" /
                "fk33_hbm_quiesce.sv")
        if not path.is_file():
            raise FileNotFoundError(path)
        platform.add_source(str(path), language="systemverilog")

        self.request = CSRStorage(1, name="request",
            description="Block all four HBM windows and drain all 32 native ports.")
        self.drained = CSRStatus(32, name="drained",
            description="Per-port native AXI drain and associated host idle.")
        self.error = CSRStatus(32, name="error",
            description="Sticky per-port AXI accounting errors; reset required.")
        self.status = CSRStatus(3, name="status",
            description="Effective request, all 32 ports drained, any runtime fault.")

        self.quiesce_request = Signal()
        self.quiesced = Signal()
        self.all_drained = Signal()
        self.external_fault = Signal()
        self._ports = set()
        raw_request = Signal()
        self.comb += raw_request.eq(self.request.storage |
                                   (self.error.status != 0) | self.external_fault)
        # Optional global fanout cut. Local tracker errors still stop their
        # own port immediately; only cross-port/software requests gain a cycle.
        # Raw faults remain visible and prohibit a successful quiesced result.
        if self.registered_request:
            self.sync += self.quiesce_request.eq(raw_request)
        else:
            self.comb += self.quiesce_request.eq(raw_request)
        self.comb += [
            self.all_drained.eq(self.drained.status == 0xffffffff),
            self.quiesced.eq(self.quiesce_request & self.all_drained &
                             (self.error.status == 0) & ~self.external_fault),
            self.status.status.eq(Cat(self.quiesce_request, self.all_drained,
                                      (self.error.status != 0) | self.external_fault)),
        ]

    def add_port(self, channel, upstream, downstream, *, host_idle):
        if channel not in self.channels or channel in self._ports:
            raise ValueError("Runtime ports must be unique members of PC0-31")
        for interface in (upstream, downstream):
            if (interface.data_width, interface.address_width, interface.id_width) != (256, 33, 6):
                raise ValueError("Runtime interposer requires AXI256/address33/ID6")
            if interface.version != "axi4":
                raise ValueError("The FK33 mapping uses the ordered-write AXI4 shape of the AXI3 subset")
        self._ports.add(channel)
        native_drained = Signal(name=f"hbm_pc{channel}_axi_drained")
        port_request = Signal(name=f"hbm_pc{channel}_quiesce_request")
        if self.registered_request:
            self.sync += port_request.eq(self.quiesce_request & host_idle)
        else:
            self.comb += port_request.eq(self.quiesce_request & host_idle)
        self.comb += [
            self.drained.status[channel].eq(native_drained & host_idle),
            upstream.ar.connect(downstream.ar, omit={"valid", "ready"}),
            upstream.aw.connect(downstream.aw, omit={"valid", "ready"}),
            upstream.w.connect(downstream.w, omit={"valid", "ready"}),
            downstream.r.connect(upstream.r),
            downstream.b.connect(upstream.b),
        ]
        self.specials += Instance("fk33_hbm_quiesce",
            p_REGISTERED_COMPLETIONS=int(self.registered_completions),
            i_clk=ClockSignal("sys"), i_rst=ResetSignal("sys"),
            i_request=port_request,
            i_s_arvalid=upstream.ar.valid, i_s_arlen=upstream.ar.len,
            o_s_arready=upstream.ar.ready, o_m_arvalid=downstream.ar.valid,
            i_m_arready=downstream.ar.ready,
            i_s_awvalid=upstream.aw.valid, i_s_awlen=upstream.aw.len,
            o_s_awready=upstream.aw.ready, o_m_awvalid=downstream.aw.valid,
            i_m_awready=downstream.aw.ready,
            i_s_wvalid=upstream.w.valid, i_s_wlast=upstream.w.last,
            o_s_wready=upstream.w.ready, o_m_wvalid=downstream.w.valid,
            i_m_wready=downstream.w.ready,
            i_m_rvalid=downstream.r.valid, i_m_rlast=downstream.r.last,
            i_s_rready=upstream.r.ready,
            i_m_bvalid=downstream.b.valid, i_s_bready=upstream.b.ready,
            o_drained=native_drained, o_error=self.error.status[channel])

    def do_finalize(self):
        if self._ports != set(self.channels):
            missing = sorted(set(self.channels) - self._ports)
            raise ValueError(f"All 32 runtime ports must be connected; missing {missing}")
