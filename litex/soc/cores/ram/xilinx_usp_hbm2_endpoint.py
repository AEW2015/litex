"""Single ordered 512-bit application / 256-bit native AXI3 transport prototype.

This is fabric transport, not an HBM controller or full-bandwidth scheduler.
Exactly one write and one read may be outstanding independently; credits retire
only when the application consumes B or the final packed R word. Descriptor LEN
counts 256-bit native beats minus one (1..16), not 512-bit words. Addresses must
be 32-byte aligned and bursts must not cross 4KiB. No WID/RID interleaving or
reordering is supported. A write descriptor must be accepted before its data.

Application W.last marks ceil(native_beats/2) words. Native WLAST derives from
LEN, never from STRB: zero-strobe beats are real beats. For odd writes the high
half of the last application word is ignored. R.keep marks valid bytes; R.id
and R.resp contain independent low/high-half metadata. Invalid upper-half data
and metadata on an odd final word are unspecified; its keep is zero.

Both domains must share reset assertion, with clocks running during release;
independent reset of either live endpoint is unsupported. Three application
startup clocks allow the jointly reset asynchronous FIFO pointers to settle.
Quiesce closes descriptor admission but permits all previously accepted data
and responses to finish. Current-generation native acknowledgement plus app
credits authorize quiesced; synchronized FIFO empty levels never authorize it.
Detected ID/LAST mismatches raise sticky protocol_error and quiesced fails shut.
The peer must obey AXI response ordering/causality and emit no extra responses;
this is not an arbitrary malformed-peer protocol checker.
"""
from migen import Cat, ClockDomainsRenamer, If, Mux, Signal
from migen.genlib.cdc import MultiReg
from litex.gen import LiteXModule
from litex.soc.interconnect import axi, stream
from litex.soc.cores.ram.xilinx_usp_hbm2_runtime import HBMClockMailbox


class HBMWideEndpoint(LiteXModule):
    def __init__(self, *, app_domain="sys", native_domain="axi", address_width=33,
                 id_width=6, fifo_depth=4, gray_bus_prefix=None):
        if app_domain == native_domain or address_width < 12 or id_width < 1:
            raise ValueError("Endpoint requires distinct domains, address>=12 and ID>=1")
        if fifo_depth != 4:
            raise ValueError("The prototype uses bounded four-entry CDC FIFOs")
        desc = [("addr", address_width), ("len", 4), ("id", id_width)]
        wide_w = [("data", 512), ("strb", 64), ("keep", 64)]
        small_w = [("data", 256), ("strb", 32), ("keep", 32)]
        small_r = [("data", 256), ("keep", 32), ("id", id_width), ("resp", 2)]
        wide_r = [("data", 512), ("keep", 64), ("id", 2*id_width), ("resp", 4)]
        self.aw = stream.Endpoint(desc)
        self.ar = stream.Endpoint(desc)
        self.w = stream.Endpoint([("data", 512), ("strb", 64)])
        self.b = stream.Endpoint([("id", id_width), ("resp", 2)])
        self.r = stream.Endpoint(wide_r)
        self.native = n = axi.AXIInterface(data_width=256, address_width=address_width,
            id_width=id_width, version="axi3", clock_domain=native_domain)
        self.quiesce = Signal()
        self.drained = Signal()
        self.quiesced = Signal()
        self.protocol_error = Signal()
        self.write_pending = Signal()
        self.read_pending = Signal()
        self.link_ready = Signal()
        app = getattr(self.sync, app_domain)
        native = getattr(self.sync, native_domain)
        startup = Signal(2)
        app += If(startup != 3, startup.eq(startup + 1))
        self.comb += self.link_ready.eq(startup == 3)

        def crossing(name, layout, source, destination):
            prefix = None if gray_bus_prefix is None else gray_bus_prefix + "_" + name
            c = stream.ClockDomainCrossing(layout, cd_from=source, cd_to=destination,
                depth=4, buffered=True, with_common_rst=False, gray_bus_prefix=prefix)
            setattr(self.submodules, name + "_cdc", c)
            return c
        aw = crossing("aw", desc, app_domain, native_domain)
        ar = crossing("ar", desc, app_domain, native_domain)
        w = crossing("w", wide_w, app_domain, native_domain)
        b = crossing("b", self.b.description, native_domain, app_domain)
        r = crossing("r", wide_r, native_domain, app_domain)
        self.submodules.unpack = unpack = ClockDomainsRenamer(native_domain)(
            stream.StrideConverter(wide_w, small_w))
        self.submodules.pack = pack = ClockDomainsRenamer(native_domain)(
            stream.StrideConverter(small_r, wide_r))
        self.comb += [w.source.connect(unpack.sink), pack.source.connect(r.sink)]
        app_error, native_error, native_error_app = Signal(), Signal(), Signal()
        self.specials += MultiReg(native_error, native_error_app, app_domain)
        self.comb += self.protocol_error.eq(app_error | native_error_app)
        remaining_w = Signal(5)
        first_w = Signal()
        for ep, c, credit in [(self.aw, aw, self.write_pending), (self.ar, ar, self.read_pending)]:
            legal = Signal()
            self.comb += [
                legal.eq((ep.addr[:5] == 0) & ((ep.addr[:12] + ((ep.len + 1) << 5)) <= 4096)),
                ep.connect(c.sink, omit={"valid", "ready"}),
                c.sink.valid.eq(ep.valid & legal & ~credit & ~self.quiesce & self.link_ready & ~self.protocol_error),
                ep.ready.eq(c.sink.ready & legal & ~credit & ~self.quiesce & self.link_ready & ~self.protocol_error),
            ]
            app += If(ep.valid & ~legal & self.link_ready & ~self.quiesce, app_error.eq(1))
        aw_fire = self.aw.valid & self.aw.ready
        ar_fire = self.ar.valid & self.ar.ready
        w_fire = self.w.valid & self.w.ready
        self.comb += [
            w.sink.valid.eq(self.w.valid & self.write_pending & (remaining_w != 0)),
            self.w.ready.eq(w.sink.ready & self.write_pending & (remaining_w != 0)),
            w.sink.data.eq(self.w.data), w.sink.strb.eq(self.w.strb),
            w.sink.keep.eq(Mux(remaining_w == 1, (1 << 32)-1, (1 << 64)-1)),
            w.sink.first.eq(first_w), w.sink.last.eq(remaining_w <= 2),
            b.source.connect(self.b), r.source.connect(self.r),
        ]
        app += [
            If(aw_fire, self.write_pending.eq(1), remaining_w.eq(self.aw.len + 1), first_w.eq(1)),
            If(w_fire,
                first_w.eq(0), remaining_w.eq(Mux(remaining_w <= 2, 0, remaining_w - 2)),
                If(self.w.last != (remaining_w <= 2), app_error.eq(1))),
            If(self.b.valid & self.b.ready,
                self.write_pending.eq(0),
                If(~self.write_pending, app_error.eq(1))),
            If(ar_fire, self.read_pending.eq(1)),
            If(self.r.valid & self.r.ready & self.r.last,
                self.read_pending.eq(0),
                If(~self.read_pending, app_error.eq(1))),
        ]

        # Registered native boundary; AW and W acceptance remain independent.
        buffers = {}
        for name in ("aw", "w", "ar", "b", "r"):
            buf = ClockDomainsRenamer(native_domain)(stream.Buffer(getattr(n, name).description))
            setattr(self.submodules, name + "_buffer", buf)
            buffers[name] = buf
        self.comb += [buffers["aw"].source.connect(n.aw), buffers["w"].source.connect(n.w),
                      buffers["ar"].source.connect(n.ar)]
        write_active, read_active = Signal(), Signal()
        write_id, read_id = Signal(id_width), Signal(id_width)
        aw_queued, aw_done, ar_queued, ar_done = Signal(), Signal(), Signal(), Signal()
        write_left, write_retired, read_left = Signal(5), Signal(5), Signal(5)
        write_beats = Signal(5)
        for source, active, queued, buf in [(aw.source, write_active, aw_queued, buffers["aw"]),
                                           (ar.source, read_active, ar_queued, buffers["ar"])]:
            self.comb += [source.ready.eq(~active), buf.sink.valid.eq(active & ~queued),
                buf.sink.size.eq(5), buf.sink.burst.eq(1)]
            # Descriptor payload must outlive FIFO pop and native backpressure.
        write_addr, read_addr = Signal(address_width), Signal(address_width)
        write_len, read_len = Signal(4), Signal(4)
        self.comb += [buffers["aw"].sink.addr.eq(write_addr), buffers["aw"].sink.len.eq(write_len),
            buffers["aw"].sink.id.eq(write_id), buffers["ar"].sink.addr.eq(read_addr),
            buffers["ar"].sink.len.eq(read_len), buffers["ar"].sink.id.eq(read_id)]
        for name in ("aw", "ar"):
            for field in ("lock", "cache", "prot", "qos", "region"):
                self.comb += getattr(buffers[name].sink, field).eq(0)
        wb, rb, bb = buffers["w"], buffers["r"], buffers["b"]
        self.comb += [
            wb.sink.valid.eq(unpack.source.valid & write_active & (write_left != 0)),
            unpack.source.ready.eq(wb.sink.ready & write_active & (write_left != 0)),
            wb.sink.data.eq(unpack.source.data), wb.sink.strb.eq(unpack.source.strb),
            wb.sink.id.eq(write_id), wb.sink.last.eq(write_left == 1),
            n.b.ready.eq(bb.sink.ready & write_active & aw_done & (write_retired == write_beats)),
            bb.sink.valid.eq(n.b.valid & write_active & aw_done & (write_retired == write_beats)),
            bb.sink.id.eq(n.b.id), bb.sink.resp.eq(n.b.resp),
            b.sink.valid.eq(bb.source.valid), bb.source.ready.eq(b.sink.ready),
            b.sink.id.eq(bb.source.id), b.sink.resp.eq(bb.source.resp),
            n.r.ready.eq(rb.sink.ready & read_active & ar_done & (read_left != 0)),
            rb.sink.valid.eq(n.r.valid & read_active & ar_done & (read_left != 0)),
            rb.sink.data.eq(n.r.data), rb.sink.id.eq(n.r.id), rb.sink.resp.eq(n.r.resp),
            rb.sink.last.eq(read_left == 1), rb.sink.first.eq(read_left == (read_len + 1)),
            pack.sink.valid.eq(rb.source.valid), rb.source.ready.eq(pack.sink.ready),
            pack.sink.data.eq(rb.source.data), pack.sink.id.eq(rb.source.id),
            pack.sink.resp.eq(rb.source.resp), pack.sink.keep.eq((1 << 32)-1),
            pack.sink.first.eq(rb.source.first), pack.sink.last.eq(rb.source.last),
        ]
        native += [
            If(aw.source.valid & aw.source.ready,
                write_active.eq(1), aw_queued.eq(0), aw_done.eq(0),
                write_id.eq(aw.source.id), write_addr.eq(aw.source.addr), write_len.eq(aw.source.len),
                write_beats.eq(aw.source.len + 1), write_left.eq(aw.source.len + 1), write_retired.eq(0)),
            If(buffers["aw"].sink.valid & buffers["aw"].sink.ready, aw_queued.eq(1)),
            If(n.aw.valid & n.aw.ready, aw_done.eq(1)),
            If(wb.sink.valid & wb.sink.ready,
                write_left.eq(write_left - 1),
                If(unpack.source.last != (write_left == 1), native_error.eq(1))),
            If(n.w.valid & n.w.ready, write_retired.eq(write_retired + 1)),
            If(bb.source.valid & bb.source.ready,
                write_active.eq(0), If(bb.source.id != write_id, native_error.eq(1))),
            If(ar.source.valid & ar.source.ready,
                read_active.eq(1), ar_queued.eq(0), ar_done.eq(0),
                read_id.eq(ar.source.id), read_addr.eq(ar.source.addr), read_len.eq(ar.source.len),
                read_left.eq(ar.source.len + 1)),
            If(buffers["ar"].sink.valid & buffers["ar"].sink.ready, ar_queued.eq(1)),
            If(n.ar.valid & n.ar.ready, ar_done.eq(1)),
            If(n.r.valid & n.r.ready,
                read_left.eq(read_left - 1),
                If((n.r.last != (read_left == 1)) | (n.r.id != read_id), native_error.eq(1))),
            If(pack.source.valid & pack.source.ready & pack.source.last, read_active.eq(0)),
        ]

        # App credits cover commands not yet visible natively, packing halves,
        # and responses buffered anywhere until their final app consumption.
        self.comb += self.drained.eq(~self.write_pending & ~self.read_pending &
            (remaining_w == 0) & ~aw_fire & ~ar_fire)
        self.submodules.quiesce_request = request = HBMClockMailbox(33, app_domain, native_domain)
        self.submodules.quiesce_ack = ack = HBMClockMailbox(33, native_domain, app_domain)
        generation = Signal(32)
        previous_q = Signal()
        request_sent, ack_sent = Signal(33), Signal(33)
        request_valid, ack_valid = Signal(), Signal()
        payload = Cat(self.quiesce, generation)
        app += [previous_q.eq(self.quiesce),
            If(previous_q != self.quiesce, generation.eq(generation + 1)),
            If(request.send & ~request.busy, request_sent.eq(payload), request_valid.eq(1))]
        idle_native = Signal()
        self.comb += idle_native.eq(~write_active & ~read_active &
            ~aw.source.valid & ~ar.source.valid & ~w.source.valid &
            ~unpack.source.valid & ~pack.source.valid &
            ~buffers["aw"].source.valid & ~wb.source.valid &
            ~buffers["ar"].source.valid & ~bb.source.valid & ~rb.source.valid)
        self.comb += [request.data.eq(payload),
            request.send.eq((previous_q == self.quiesce) & ~request.busy &
                (~request_valid | (request_sent != payload))),
            ack.data.eq(request.output),
            ack.send.eq(idle_native & ~ack.busy & (~ack_valid | (ack_sent != request.output))),
            self.quiesced.eq(self.link_ready & self.quiesce & (previous_q == self.quiesce) &
                self.drained & (ack.output == payload) & ~self.protocol_error)]
        native += If(ack.send & ~ack.busy, ack_sent.eq(request.output), ack_valid.eq(1))
