"""Admission and AXI drain controls for the FK33 32-port HBM runtime path.

The interposer closes Wishbone admission before conversion and tracks native
AXI transactions for all physical HBM ports. It does not control READY, AXI
resets, APB, or clocks.
"""
from pathlib import Path

from migen import Cat, ClockSignal, If, Instance, Mux, ResetSignal, Signal
from migen.genlib.cdc import MultiReg
from litex.gen import LiteXModule
from litex.soc.interconnect.csr import CSRStatus, CSRStorage


class HBMClockMailbox(LiteXModule):
    """One outstanding bundled-data transfer, acknowledged after capture.

    Only toggle controls pass through synchronizers. The source payload is
    held from launch through the returned acknowledgement, including two
    destination clocks before capture. Both domains must share system reset;
    READY withdrawal must not reset either mailbox endpoint independently.
    """
    def __init__(self, width, source, destination):
        self.data = Signal(width)
        self.send = Signal()
        self.busy = Signal()
        self.output = Signal(width)
        self.update = Signal()
        held = Signal(width)
        request, acknowledge = Signal(), Signal()
        request_d, acknowledge_s = Signal(), Signal()
        self.specials += [MultiReg(request, request_d, destination),
                          MultiReg(acknowledge, acknowledge_s, source)]
        self.source_epoch = request
        self.destination_epoch = acknowledge
        self.comb += self.busy.eq(request != acknowledge_s)
        source_sync = getattr(self.sync, source)
        destination_sync = getattr(self.sync, destination)
        source_sync += If(self.send & ~self.busy,
            held.eq(self.data), request.eq(~request))
        destination_sync += [
            self.update.eq(0),
            If(request_d != acknowledge,
                self.output.eq(held), self.update.eq(1),
                acknowledge.eq(request_d)),
        ]


def hbm_status_snapshot(module, pairs, source):
    """Continuously capture an atomic native bundle into SYS shadows.

    A sequence counter permits software to retry multiword reads while busy.
    Counters remain stable after ownership is released, as in the original ABI.
    """
    mailbox = HBMClockMailbox(sum(len(native) for native, _ in pairs), source, "sys")
    module.submodules += mailbox
    module.comb += [mailbox.data.eq(Cat(*[native for native, _ in pairs])),
                    mailbox.send.eq(~mailbox.busy)]
    offset = 0
    for native, shadow in pairs:
        module.sync += If(mailbox.update,
                          shadow.eq(mailbox.output[offset:offset + len(native)]))
        offset += len(native)
    return mailbox


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

    Separate-domain mode keeps CSRs in SYS. STOP completion requires the
    current request generation, the board admission/idle barrier and a coherent
    returned native snapshot. An open idle observation does not require STOP.
    Optional registered status adds one source clock to the published snapshot;
    live native admission, drain and fault controls retain their existing paths.
    """
    CHANNELS = tuple(range(32))

    def __init__(self, platform, channels=CHANNELS, *, registered_request=False,
                 registered_completions=False, clock_domain="sys", sys_host_idle=None,
                 registered_status=False, local_status_pipeline=False,
                 status_publication_pipeline=False, native_tracker_tags=False):
        if tuple(channels) != self.CHANNELS:
            raise ValueError("Runtime interposer requires native channels 0 through 31")
        self.clock_domain = clock_domain
        native_sync = getattr(self.sync, clock_domain)
        self.channels = tuple(channels)
        self.registered_request = bool(registered_request)
        self.registered_completions = bool(registered_completions)
        self.local_status_pipeline = bool(local_status_pipeline)
        self.status_publication_pipeline = bool(status_publication_pipeline)
        self.native_tracker_tags = bool(native_tracker_tags)
        if status_publication_pipeline and not (registered_status and clock_domain != "sys"):
            raise ValueError("Pipelined publication requires registered status in a separate domain")
        if local_status_pipeline and not (registered_status and clock_domain != "sys"):
            raise ValueError("Local status publication requires registered status in a separate domain")
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
        self.native_quiesce_request = Signal()
        self.native_all_drained = Signal()
        self.native_quiesced = Signal()
        self._native_drained, self._native_error = Signal(32), Signal(32)
        self._native_fault = Signal()
        if self.local_status_pipeline:
            self._local_status_drained = Signal(32)
            self._local_status_host_idle = Signal(32)
        self.request_generation_sys = Signal(32)
        self.request_generation_native = Signal(32)
        self.host_barrier_generation_native = Signal(32)
        if sys_host_idle is None:
            sys_host_idle = 1
        self._quiesced_sys = Signal()
        self._all_drained_sys = Signal()
        if clock_domain != "sys":
            # Each SYS request transition advances the generation, even if it
            # occurs while a prior mailbox is busy. A rapid 1/0/1 sequence must
            # never reuse the drained snapshot from the preceding STOP.
            self.request_mailbox = HBMClockMailbox(33, "sys", clock_domain)
            observed_request = Signal()
            generation, sent_generation, snapshot_generation = Signal(32), Signal(32), Signal(32)
            self.request_generation = generation
            self.comb += [self.request_generation_sys.eq(generation),
                          self.request_generation_native.eq(self.request_mailbox.output[1:])]
            self.request_acknowledged = Signal()
            drained_snapshot = Signal(32)
            self.comb += [
                self.drained.status.eq(Mux(self.request_acknowledged & sys_host_idle,
                    (drained_snapshot & ~self.error.status) if status_publication_pipeline else drained_snapshot, 0)),
                self.request_mailbox.data.eq(Cat(self.quiesce_request, generation)),
                self.request_mailbox.send.eq(generation != sent_generation),
                self.native_quiesce_request.eq(self.request_mailbox.output[0]),
                self.request_acknowledged.eq(~self.request_mailbox.busy &
                    (observed_request == self.quiesce_request) &
                    (sent_generation == generation) & (snapshot_generation == generation)),
            ]
            self.sync += [
                If(observed_request != self.quiesce_request,
                    observed_request.eq(self.quiesce_request), generation.eq(generation + 1)),
                If(self.request_mailbox.send & ~self.request_mailbox.busy,
                    sent_generation.eq(generation)),
            ]
            self.specials += MultiReg(self.external_fault, self._native_fault, clock_domain)
            status_drained = self._native_drained
            status_error = self._native_error
            status_quiesced = self.native_quiesced
            status_all_drained = self.native_all_drained
            status_generation = self.request_mailbox.output[1:]
            if registered_status:
                # Capture all raw fields together before the global reductions.
                # Flags and their generation must describe this same image.
                self.status_image = status_image = Signal(99)
                if self.local_status_pipeline:
                    # Drain ingredients and their request/barrier epoch share one
                    # observation edge. Sticky errors retain their original
                    # publication cadence and can only suppress an older drain.
                    self.local_status_meta = status_meta = Signal(35)
                    native_sync += status_meta.eq(Cat(
                        self.request_generation_native, self.native_quiesce_request,
                        self._native_fault,
                        self.host_barrier_generation_native == self.request_generation_native))
                    native_sync += status_image.eq(Cat(
                        self._local_status_drained & self._local_status_host_idle & ~self._native_error,
                        self._native_error, status_meta))
                else:
                    native_sync += status_image.eq(Cat(
                        self._native_drained, self._native_error,
                        self.request_generation_native, self.native_quiesce_request,
                        self._native_fault,
                        self.host_barrier_generation_native == self.request_generation_native))
                status_drained = status_image[:32]
                status_error = status_image[32:64]
                status_generation = status_image[64:96]
                status_request = status_image[96]
                status_fault = status_image[97]
                status_barrier_current = status_image[98]
                status_all_drained = Signal()
                status_quiesced = Signal()
                self.comb += [
                    status_all_drained.eq((status_drained == 0xffffffff) &
                        (~status_request | status_barrier_current)),
                    status_quiesced.eq(status_request & status_all_drained &
                        (status_error == 0) & ~status_fault),
                ]
            if status_publication_pipeline:
                # Older non-error fields retain one coherent observation epoch.
                # Errors keep their original mailbox/admission publication cadence.
                self.status_publication_image = publication = Signal(66)
                native_sync += publication.eq(Cat(status_drained, status_quiesced,
                    status_all_drained, status_generation))
                status_drained = publication[:32]
                status_quiesced = publication[32]
                status_all_drained = publication[33]
                status_generation = publication[34:]
            hbm_status_snapshot(self, [(status_drained, drained_snapshot),
                                      (status_error, self.error.status),
                                      (status_quiesced, self._quiesced_sys),
                                      (status_all_drained, self._all_drained_sys),
                                      (status_generation, snapshot_generation)], clock_domain)
        else:
            self.request_acknowledged = Signal(reset=1)
            self.comb += [self.native_quiesce_request.eq(self.quiesce_request),
                          self._native_fault.eq(self.external_fault),
                          self._quiesced_sys.eq(self.native_quiesced),
                          self._all_drained_sys.eq(self.native_all_drained),
                          self.drained.status.eq(self._native_drained),
                          self.error.status.eq(self._native_error)]
        self.comb += [
            self.native_all_drained.eq((self._native_drained == 0xffffffff) &
                ((~self.native_quiesce_request |
                  (self.host_barrier_generation_native == self.request_generation_native))
                 if clock_domain != "sys" else 1)),
            self.native_quiesced.eq(self.native_quiesce_request & self.native_all_drained &
                                    (self._native_error == 0) & ~self._native_fault),
        ]
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
            self.all_drained.eq(self._all_drained_sys & (self.drained.status == 0xffffffff) & self.request_acknowledged & sys_host_idle),
            self.quiesced.eq(self.quiesce_request & self._quiesced_sys & self.all_drained &
                             (self.error.status == 0) & ~self.external_fault),
            self.status.status.eq(Cat(self.quiesce_request, self.all_drained,
                                      (self.error.status != 0) | self.external_fault)),
        ]

    def add_port(self, channel, upstream, downstream, *, host_idle, downstream_idle=1):
        if channel not in self.channels or channel in self._ports:
            raise ValueError("Runtime ports must be unique members of PC0-31")
        for interface in (upstream, downstream):
            if (interface.data_width, interface.address_width, interface.id_width) != (256, 33, 6):
                raise ValueError("Runtime interposer requires AXI256/address33/ID6")
            if interface.version != "axi4":
                raise ValueError("The FK33 mapping uses the ordered-write AXI4 shape of the AXI3 subset")
        self._ports.add(channel)
        native_sync = getattr(self.sync, self.clock_domain)
        native_drained = Signal(name=f"hbm_pc{channel}_axi_drained")
        if self.local_status_pipeline:
            local_drained = Signal(name=f"hbm_pc{channel}_status_drained")
            local_host_idle = Signal(name=f"hbm_pc{channel}_status_host_idle")
            native_sync += [
                local_drained.eq(native_drained & downstream_idle),
                local_host_idle.eq(host_idle),
            ]
            self.comb += [
                self._local_status_drained[channel].eq(local_drained),
                self._local_status_host_idle[channel].eq(local_host_idle),
            ]
        port_request = Signal(name=f"hbm_pc{channel}_quiesce_request")
        if self.registered_request:
            native_sync += port_request.eq(self.native_quiesce_request & host_idle)
        else:
            self.comb += port_request.eq(self.native_quiesce_request & host_idle)
        self.comb += [
            self._native_drained[channel].eq(native_drained & host_idle & downstream_idle),
            upstream.ar.connect(downstream.ar, omit={"valid", "ready"}),
            upstream.aw.connect(downstream.aw, omit={"valid", "ready"}),
            upstream.w.connect(downstream.w, omit={"valid", "ready"}),
            downstream.r.connect(upstream.r),
            downstream.b.connect(upstream.b),
        ]
        tagged_error = None
        if self.native_tracker_tags:
            tagged_error = Signal(name=f"hbm_pc{channel}_tracker_error",
                attr={("keep", "true"), ("hbm_tracker_pc", str(channel))})
            self.comb += self._native_error[channel].eq(tagged_error)
        self.specials += Instance("fk33_hbm_quiesce",
            p_REGISTERED_COMPLETIONS=int(self.registered_completions),
            i_clk=ClockSignal(self.clock_domain), i_rst=ResetSignal(self.clock_domain),
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
            o_drained=native_drained,
            o_error=tagged_error if self.native_tracker_tags else self._native_error[channel])

    def do_finalize(self):
        if self._ports != set(self.channels):
            missing = sorted(set(self.channels) - self._ports)
            raise ValueError(f"All 32 runtime ports must be connected; missing {missing}")
