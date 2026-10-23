"""Single ordered 512-bit application / 256-bit native AXI3 transport prototype.

This is fabric transport, not an HBM controller or full-bandwidth scheduler.
The default allows one write and one read independently; the opt-in four-command
window preserves order with one ID lock per direction. Credits retire
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
from migen import Cat, ClockDomainsRenamer, ClockSignal, If, Instance, Memory, Module, Mux, ResetSignal, Signal
from migen.genlib.cdc import GrayCounter, MultiReg
from litex.gen import LiteXModule
from litex.soc.interconnect import axi, stream
from litex.soc.cores.ram.xilinx_usp_hbm2_runtime import HBMClockMailbox


class HBMControlFF(Instance):
    """Dedicated synchronous reset/set control FF; simulation override is external."""
    def __init__(self, q, d, ce, *, domain, name, reset):
        self.q, self.d, self.ce = q, d, ce
        self.domain, self.reset_value = domain, reset
        reset_pin = {"i_S" if reset else "i_R": ResetSignal(domain)}
        super().__init__("FDSE" if reset else "FDRE", name=name,
            p_INIT=reset, i_C=ClockSignal(domain), i_CE=ce, i_D=d, o_Q=q,
            attr={("hbm_direct_reset", name)}, **reset_pin)


class HBMSynchronousControlRegister(Module):
    """Keep functional clears on D/CE and the domain reset on dedicated R/S."""
    def __init__(self, q, d, ce, *, domain, name):
        for bit in range(len(q)):
            self.specials += HBMControlFF(q[bit], d[bit], ce, domain=domain,
                name="hbm_control_{}_bit{}".format(name, bit),
                reset=(q.reset.value >> bit) & 1)


class HBMRegisteredReadCDC(LiteXModule):
    """Four-slot ownership FIFO with current-address asynchronous RAM read.

    Ownership/full/empty equations are adapted from Migen genlib.fifo.AsyncFIFO
    (Migen contributors, BSD-2-Clause). GrayCounter, MultiReg, Memory and Buffer
    are public library primitives; no Migen or stream internals are patched.
    A slot is released only when its payload enters the destination Buffer.
    Producer synchronization affects valid/consume enable, never RAM address.
    Both domains must assert reset together; independent live reset is unsafe.
    """
    def __init__(self, layout, *, cd_from, cd_to, gray_bus_prefix=None):
        if cd_from == cd_to:
            raise ValueError("The ownership FIFO requires distinct clock domains")
        if gray_bus_prefix is not None and (not isinstance(gray_bus_prefix, str) or not gray_bus_prefix):
            raise ValueError("Gray prefix must be a nonempty string")
        self.sink = sink = stream.Endpoint(layout)
        self.source = source = stream.Endpoint(layout)
        description = sink.description
        self.submodules.produce = produce = ClockDomainsRenamer(cd_from)(GrayCounter(3))
        self.submodules.consume = consume = ClockDomainsRenamer(cd_to)(GrayCounter(3))
        self.produce_seen = produce_seen = Signal(3)
        self.consume_seen = consume_seen = Signal(3)
        produce.q.attr.add("no_retiming")
        consume.q.attr.add("no_retiming")
        if gray_bus_prefix is not None:
            produce.q.attr.add(("hbm_gray_bus", gray_bus_prefix + "_produce"))
            consume.q.attr.add(("hbm_gray_bus", gray_bus_prefix + "_consume"))
        self.specials += [MultiReg(produce.q, produce_seen, cd_to),
                          MultiReg(consume.q, consume_seen, cd_from)]
        self.writable = writable = Signal()
        self.readable = readable = Signal()
        self.comb += [
            writable.eq((produce.q[2] == consume_seen[2]) |
                        (produce.q[1] == consume_seen[1]) |
                        (produce.q[0] != consume_seen[0])),
            readable.eq(consume.q != produce_seen), sink.ready.eq(writable),
            produce.ce.eq(sink.valid & writable),
        ]
        payload_width = len(sink.payload.raw_bits())
        param_width = len(sink.param.raw_bits())
        packed_fields = [sink.payload.raw_bits()]
        if param_width:
            packed_fields.append(sink.param.raw_bits())
        packed = Cat(*packed_fields, sink.first, sink.last)
        self.storage = storage = Memory(len(packed), 4)
        self.specials += storage
        self.write_port = write_port = storage.get_port(write_capable=True, clock_domain=cd_from)
        self.read_port = read_port = storage.get_port(async_read=True, clock_domain=cd_to)
        self.specials += [write_port, read_port]
        self.submodules.output_buffer = output_buffer = ClockDomainsRenamer(cd_to)(stream.Buffer(description))
        self.comb += [
            write_port.adr.eq(produce.q_binary[:2]), write_port.dat_w.eq(packed),
            write_port.we.eq(produce.ce),
            # CURRENT registered pointer: no readable/re/Gray lookahead here.
            read_port.adr.eq(consume.q_binary[:2]),
            output_buffer.sink.valid.eq(readable),
            output_buffer.sink.payload.raw_bits().eq(read_port.dat_r[:payload_width]),
            output_buffer.sink.first.eq(read_port.dat_r[payload_width + param_width]),
            output_buffer.sink.last.eq(read_port.dat_r[payload_width + param_width + 1]),
            consume.ce.eq(readable & output_buffer.sink.ready),
            output_buffer.source.connect(source),
        ]
        if param_width:
            self.comb += output_buffer.sink.param.raw_bits().eq(
                read_port.dat_r[payload_width:payload_width + param_width])


class HBMWideEndpoint(LiteXModule):
    def __init__(self, *, app_domain="sys", native_domain="axi", address_width=33,
                 id_width=6, fifo_depth=4, gray_bus_prefix=None, max_outstanding=1):
        self.max_outstanding = max_outstanding
        if max_outstanding not in (1, 4):
            raise ValueError("The endpoint supports one or four outstanding descriptors")
        if max_outstanding == 4:
            self.submodules.window = window = _HBMWindowFourEndpoint(
                app_domain=app_domain, native_domain=native_domain,
                address_width=address_width, id_width=id_width,
                fifo_depth=fifo_depth, gray_bus_prefix=gray_bus_prefix)
            for name in ("aw", "ar", "w", "b", "r", "native", "quiesce", "drained",
                         "quiesced", "protocol_error", "write_pending", "read_pending",
                         "link_ready", "write_credits", "read_credits"):
                setattr(self, name, getattr(window, name))
            return
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
            c = HBMRegisteredReadCDC(layout, cd_from=source, cd_to=destination,
                gray_bus_prefix=prefix)
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
        # The CDC already registers data. Cut the gearbox-ready to FIFO
        # read-address loop with a standard one-word elastic skid stage.
        self.submodules.write_elastic = write_elastic = ClockDomainsRenamer(native_domain)(
            stream.Buffer(wide_w, pipe_valid=False, pipe_ready=True))
        self.comb += [w.source.connect(write_elastic.sink),
                      write_elastic.source.connect(unpack.sink), pack.source.connect(r.sink)]
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
        write_descriptor_fire = aw.source.valid & aw.source.ready
        native_write_fire = n.w.valid & n.w.ready
        write_response_fire = buffers["b"].source.valid & buffers["b"].source.ready
        read_descriptor_fire = ar.source.valid & ar.source.ready
        packed_read_last_fire = pack.source.valid & pack.source.ready & pack.source.last
        self.submodules.write_retired_control = HBMSynchronousControlRegister(
            write_retired, Mux(native_write_fire, write_retired + 1, 0),
            write_descriptor_fire | native_write_fire, domain=native_domain, name="endpoint_write_retired")
        self.submodules.write_active_control = HBMSynchronousControlRegister(
            write_active, Mux(write_response_fire, 0, 1),
            write_descriptor_fire | write_response_fire, domain=native_domain, name="endpoint_write_active")
        self.submodules.read_active_control = HBMSynchronousControlRegister(
            read_active, Mux(packed_read_last_fire, 0, 1),
            read_descriptor_fire | packed_read_last_fire, domain=native_domain, name="endpoint_read_active")
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
                aw_queued.eq(0), aw_done.eq(0),
                write_id.eq(aw.source.id), write_addr.eq(aw.source.addr), write_len.eq(aw.source.len),
                write_beats.eq(aw.source.len + 1), write_left.eq(aw.source.len + 1)),
            If(buffers["aw"].sink.valid & buffers["aw"].sink.ready, aw_queued.eq(1)),
            If(n.aw.valid & n.aw.ready, aw_done.eq(1)),
            If(wb.sink.valid & wb.sink.ready,
                write_left.eq(write_left - 1),
                If(unpack.source.last != (write_left == 1), native_error.eq(1))),
            If(bb.source.valid & bb.source.ready,
                If(bb.source.id != write_id, native_error.eq(1))),
            If(ar.source.valid & ar.source.ready,
                ar_queued.eq(0), ar_done.eq(0),
                read_id.eq(ar.source.id), read_addr.eq(ar.source.addr), read_len.eq(ar.source.len),
                read_left.eq(ar.source.len + 1)),
            If(buffers["ar"].sink.valid & buffers["ar"].sink.ready, ar_queued.eq(1)),
            If(n.ar.valid & n.ar.ready, ar_done.eq(1)),
            If(n.r.valid & n.r.ready,
                read_left.eq(read_left - 1),
                If((n.r.last != (read_left == 1)) | (n.r.id != read_id), native_error.eq(1))),
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
            ~write_elastic.source.valid & ~unpack.source.valid & ~pack.source.valid &
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


class _HBMWindowFourEndpoint(LiteXModule):
    """Four ordered commands per direction; one ID lock per application window.

    Native issue and acceptance have separate ownership queues. Credits retire
    only at application consumption. This is not an AXI reordering engine.
    """
    def __init__(self, *, app_domain, native_domain, address_width, id_width,
                 fifo_depth, gray_bus_prefix):
        if app_domain == native_domain or address_width < 12 or id_width < 1 or fifo_depth != 4:
            raise ValueError("Window four requires distinct domains, address>=12, ID>=1 and depth four")
        desc = [("addr", address_width), ("len", 4), ("id", id_width)]
        wide_w = [("data", 512), ("strb", 64), ("keep", 64)]
        small_w = [("data", 256), ("strb", 32), ("keep", 32)]
        small_r = [("data", 256), ("keep", 32), ("id", id_width), ("resp", 2)]
        wide_r = [("data", 512), ("keep", 64), ("id", 2*id_width), ("resp", 4)]
        self.aw, self.ar = stream.Endpoint(desc), stream.Endpoint(desc)
        self.w = stream.Endpoint([("data", 512), ("strb", 64)])
        self.b, self.r = stream.Endpoint([("id", id_width), ("resp", 2)]), stream.Endpoint(wide_r)
        self.native = n = axi.AXIInterface(data_width=256, address_width=address_width,
            id_width=id_width, version="axi3", clock_domain=native_domain)
        for name in ("quiesce", "drained", "quiesced", "protocol_error", "write_pending", "read_pending", "link_ready"):
            setattr(self, name, Signal())
        self.write_credits, self.read_credits = Signal(3), Signal(3)
        app, native = getattr(self.sync, app_domain), getattr(self.sync, native_domain)
        startup = Signal(2)
        app += If(startup != 3, startup.eq(startup + 1))
        self.comb += self.link_ready.eq(startup == 3)
        app_error, native_error, native_error_app = Signal(), Signal(), Signal()
        self.specials += MultiReg(native_error, native_error_app, app_domain)
        self.comb += self.protocol_error.eq(app_error | native_error_app)
        def crossing(name, layout, source, destination):
            prefix = None if gray_bus_prefix is None else gray_bus_prefix + "_" + name
            c = HBMRegisteredReadCDC(layout, cd_from=source, cd_to=destination, gray_bus_prefix=prefix)
            setattr(self.submodules, name + "_cdc", c)
            return c
        def queue(name, layout, domain):
            q = ClockDomainsRenamer(domain)(stream.SyncFIFO(layout, 4, buffered=True))
            setattr(self.submodules, name, q)
            return q
        aw, ar = crossing("aw", desc, app_domain, native_domain), crossing("ar", desc, app_domain, native_domain)
        w = crossing("w", wide_w, app_domain, native_domain)
        b, r = crossing("b", self.b.description, native_domain, app_domain), crossing("r", wide_r, native_domain, app_domain)
        lengths = queue("application_write_lengths", [("len", 4)], app_domain)
        write_dispatch = queue("application_write_dispatch", desc, app_domain)
        # A registered dispatch head retains descriptor ownership until each
        # branch has acknowledged it; neither branch can duplicate a command.
        locks = [Signal(id_width), Signal(id_width)]
        fires = []
        for ep, c, count, lock in [(self.aw, aw, self.write_credits, locks[0]), (self.ar, ar, self.read_credits, locks[1])]:
            legal, admission = Signal(), Signal()
            retire = (self.b.valid & self.b.ready) if ep is self.aw else (self.r.valid & self.r.ready & self.r.last)
            self.comb += [legal.eq((ep.addr[:5] == 0) & ((ep.addr[:12] + ((ep.len + 1) << 5)) <= 4096)),
                admission.eq(legal & ((count < 4) | retire) & ((count == 0) | (ep.id == lock)) &
                    ~self.quiesce & self.link_ready & ~self.protocol_error),
                ep.connect(write_dispatch.sink if ep is self.aw else c.sink, omit={"valid", "ready"})]
            sink = write_dispatch.sink if ep is self.aw else c.sink
            self.comb += [sink.valid.eq(ep.valid & admission), ep.ready.eq(sink.ready & admission)]
            fire = ep.valid & ep.ready
            fires.append(fire)
            app += [If(fire & (count == 0), lock.eq(ep.id)),
                If(ep.valid & ~legal & self.link_ready & ~self.quiesce, app_error.eq(1))]
        sent_aw, sent_length = Signal(), Signal()
        dispatch = write_dispatch.source
        dispatch_done = dispatch.valid & (sent_aw | aw.sink.ready) & (sent_length | lengths.sink.ready)
        self.comb += [dispatch.connect(aw.sink, omit={"valid", "ready"}),
            aw.sink.valid.eq(dispatch.valid & ~sent_aw), lengths.sink.valid.eq(dispatch.valid & ~sent_length),
            lengths.sink.len.eq(dispatch.len), dispatch.ready.eq((sent_aw | aw.sink.ready) & (sent_length | lengths.sink.ready))]
        app += [If(aw.sink.valid & aw.sink.ready, sent_aw.eq(1)),
            If(lengths.sink.valid & lengths.sink.ready, sent_length.eq(1)),
            If(dispatch_done, sent_aw.eq(0), sent_length.eq(0))]
        aw_fire, ar_fire = fires
        self.comb += [b.source.connect(self.b), r.source.connect(self.r),
            self.write_pending.eq(self.write_credits != 0), self.read_pending.eq(self.read_credits != 0)]
        for count, take, retire in [(self.write_credits, aw_fire, self.b.valid & self.b.ready),
                                  (self.read_credits, ar_fire, self.r.valid & self.r.ready & self.r.last)]:
            app += [If(take & ~retire, count.eq(count + 1)).Elif(retire & ~take, count.eq(count - 1)),
                If(retire & (count == 0), app_error.eq(1))]
        app_w_active, app_left, app_first = Signal(), Signal(5), Signal()
        app_w_fire = self.w.valid & self.w.ready
        app_terminal = app_w_fire & (app_left <= 2)
        load_length = lengths.source.valid & lengths.source.ready
        self.comb += [lengths.source.ready.eq(~app_w_active | app_terminal),
            w.sink.valid.eq(self.w.valid & app_w_active), self.w.ready.eq(w.sink.ready & app_w_active),
            w.sink.data.eq(self.w.data), w.sink.strb.eq(self.w.strb),
            w.sink.keep.eq(Mux(app_left == 1, (1 << 32)-1, (1 << 64)-1)),
            w.sink.first.eq(app_first), w.sink.last.eq(app_left <= 2)]
        app += [If(app_w_fire, app_first.eq(0), app_left.eq(app_left - 2),
                    If(self.w.last != (app_left <= 2), app_error.eq(1))),
                If(app_terminal, app_w_active.eq(0), app_left.eq(0)),
                If(load_length, app_w_active.eq(1), app_left.eq(lengths.source.len + 1), app_first.eq(1))]
        self.submodules.unpack = unpack = ClockDomainsRenamer(native_domain)(stream.StrideConverter(wide_w, small_w))
        self.submodules.pack = pack = ClockDomainsRenamer(native_domain)(stream.StrideConverter(small_r, wide_r))
        self.submodules.write_elastic = elastic = ClockDomainsRenamer(native_domain)(stream.Buffer(wide_w, pipe_valid=False, pipe_ready=True))
        self.comb += [w.source.connect(elastic.sink), elastic.source.connect(unpack.sink), pack.source.connect(r.sink)]
        aw_issue, w_issue, b_expect = [queue(name, desc, native_domain) for name in ("aw_issue", "w_issue", "b_expect")]
        ar_issue, r_expect = [queue(name, desc, native_domain) for name in ("ar_issue", "r_expect")]
        dispatch_states = []
        for role, src, destinations in [("write_dispatch", aw.source, [aw_issue.sink, w_issue.sink, b_expect.sink]),
                                        ("read_dispatch", ar.source, [ar_issue.sink, r_expect.sink])]:
            accepted = Signal(len(destinations))
            dispatch_states.append(accepted)
            done = src.valid
            updates = []
            for bit, dst in enumerate(destinations):
                done = done & (accepted[bit] | dst.ready)
                self.comb += [src.connect(dst, omit={"valid", "ready"}), dst.valid.eq(src.valid & ~accepted[bit])]
                updates.append(accepted[bit] | (dst.valid & dst.ready))
            self.comb += src.ready.eq(done)
            setattr(self.submodules, role + "_control", HBMSynchronousControlRegister(
                accepted, Mux(done, 0, Cat(*updates)), 1, domain=native_domain, name="window_" + role))
        tokens = {name:queue(name + "_accepted", [("token", 1)], native_domain) for name in ("aw", "w", "ar")}
        buffers = {}
        for name in ("aw", "w", "ar", "b", "r"):
            buffers[name] = ClockDomainsRenamer(native_domain)(stream.Buffer(getattr(n, name).description))
            setattr(self.submodules, name + "_buffer", buffers[name])
        for name, commands in [("aw", aw_issue), ("ar", ar_issue)]:
            buf, port, token = buffers[name], getattr(n, name), tokens[name]
            self.comb += [buf.sink.valid.eq(commands.source.valid), commands.source.ready.eq(buf.sink.ready),
                buf.sink.addr.eq(commands.source.addr), buf.sink.len.eq(commands.source.len), buf.sink.id.eq(commands.source.id),
                buf.sink.size.eq(5), buf.sink.burst.eq(1),
                buf.source.connect(port, omit={"valid", "ready"}),
                port.valid.eq(buf.source.valid & token.sink.ready), buf.source.ready.eq(port.ready & token.sink.ready),
                token.sink.valid.eq(port.valid & port.ready), token.sink.token.eq(1)]
            for field in ("lock", "cache", "prot", "qos", "region"): self.comb += getattr(buf.sink, field).eq(0)
        wa, wl, wid = Signal(), Signal(5), Signal(id_width)
        wb, rb, bb = buffers["w"], buffers["r"], buffers["b"]
        w_issue_fire = wb.sink.valid & wb.sink.ready
        w_terminal = w_issue_fire & (wl == 1)
        w_load = w_issue.source.valid & w_issue.source.ready
        self.comb += [w_issue.source.ready.eq(~wa | w_terminal),
            wb.sink.valid.eq(unpack.source.valid & wa), unpack.source.ready.eq(wb.sink.ready & wa),
            wb.sink.data.eq(unpack.source.data), wb.sink.strb.eq(unpack.source.strb), wb.sink.id.eq(wid), wb.sink.last.eq(wl == 1),
            wb.source.connect(n.w, omit={"valid", "ready"}),
            n.w.valid.eq(wb.source.valid & (~wb.source.last | tokens["w"].sink.ready)),
            wb.source.ready.eq(n.w.ready & (~wb.source.last | tokens["w"].sink.ready)),
            tokens["w"].sink.valid.eq(n.w.valid & n.w.ready & n.w.last), tokens["w"].sink.token.eq(1)]
        def control(q, d, ce, name):
            setattr(self.submodules, name + "_control", HBMSynchronousControlRegister(q, d, ce, domain=native_domain, name="window_" + name))
        control(wa, Mux(w_load, 1, 0), w_load | w_terminal, "write_active")
        control(wl, Mux(w_load, w_issue.source.len + 1, wl - 1), w_load | w_issue_fire, "write_left")
        control(wid, w_issue.source.id, w_load, "write_id")
        native += If(w_issue_fire & (unpack.source.last != (wl == 1)), native_error.eq(1))
        complete = b_expect.source.valid & tokens["aw"].source.valid & tokens["w"].source.valid
        b_fire = n.b.valid & n.b.ready
        self.comb += [n.b.ready.eq(bb.sink.ready & complete), bb.sink.valid.eq(n.b.valid & complete),
            bb.sink.id.eq(n.b.id), bb.sink.resp.eq(n.b.resp),
            b_expect.source.ready.eq(b_fire), tokens["aw"].source.ready.eq(b_fire), tokens["w"].source.ready.eq(b_fire),
            b.sink.valid.eq(bb.source.valid), bb.source.ready.eq(b.sink.ready), b.sink.id.eq(bb.source.id), b.sink.resp.eq(bb.source.resp)]
        native += If(b_fire & (n.b.id != b_expect.source.id), native_error.eq(1))
        ra, rl, rbeats, rid = Signal(), Signal(5), Signal(5), Signal(id_width)
        read_fire = n.r.valid & n.r.ready
        r_terminal = read_fire & (rl == 1)
        r_load = r_expect.source.valid & r_expect.source.ready
        self.comb += [r_expect.source.ready.eq((~ra | r_terminal) & tokens["ar"].source.valid),
            tokens["ar"].source.ready.eq(r_load),
            n.r.ready.eq(rb.sink.ready & ra), rb.sink.valid.eq(n.r.valid & ra),
            rb.sink.data.eq(n.r.data), rb.sink.id.eq(n.r.id), rb.sink.resp.eq(n.r.resp),
            rb.sink.first.eq(rl == rbeats), rb.sink.last.eq(rl == 1),
            pack.sink.valid.eq(rb.source.valid), rb.source.ready.eq(pack.sink.ready),
            pack.sink.data.eq(rb.source.data), pack.sink.id.eq(rb.source.id), pack.sink.resp.eq(rb.source.resp),
            pack.sink.keep.eq((1 << 32)-1), pack.sink.first.eq(rb.source.first), pack.sink.last.eq(rb.source.last)]
        control(ra, Mux(r_load, 1, 0), r_load | r_terminal, "read_active")
        control(rl, Mux(r_load, r_expect.source.len + 1, rl - 1), r_load | read_fire, "read_left")
        control(rbeats, r_expect.source.len + 1, r_load, "read_beats")
        control(rid, r_expect.source.id, r_load, "read_id")
        native += If(read_fire & ((n.r.id != rid) | (n.r.last != (rl == 1))), native_error.eq(1))
        self.comb += self.drained.eq((self.write_credits == 0) & (self.read_credits == 0) & ~app_w_active &
            (lengths.level == 0) & ~lengths.source.valid & (write_dispatch.level == 0) & ~write_dispatch.source.valid &
            ~sent_aw & ~sent_length & ~aw_fire & ~ar_fire)
        self.submodules.quiesce_request = request = HBMClockMailbox(33, app_domain, native_domain)
        self.submodules.quiesce_ack = ack = HBMClockMailbox(33, native_domain, app_domain)
        generation, previous = Signal(32), Signal()
        request_sent, ack_sent, request_valid, ack_valid = Signal(33), Signal(33), Signal(), Signal()
        payload = Cat(self.quiesce, generation)
        app += [previous.eq(self.quiesce), If(previous != self.quiesce, generation.eq(generation + 1)),
            If(request.send & ~request.busy, request_sent.eq(payload), request_valid.eq(1))]
        idle = ~wa & ~ra & ~aw.source.valid & ~ar.source.valid & ~w.source.valid & ~elastic.source.valid & ~unpack.source.valid & ~pack.source.valid
        for q in [aw_issue, w_issue, b_expect, ar_issue, r_expect] + list(tokens.values()): idle = idle & (q.level == 0) & ~q.source.valid
        for buf in buffers.values(): idle = idle & ~buf.source.valid
        for accepted in dispatch_states: idle = idle & (accepted == 0)
        self.comb += [request.data.eq(payload), request.send.eq((previous == self.quiesce) & ~request.busy & (~request_valid | (request_sent != payload))),
            ack.data.eq(request.output), ack.send.eq(idle & ~ack.busy & (~ack_valid | (ack_sent != request.output))),
            self.quiesced.eq(self.link_ready & self.quiesce & (previous == self.quiesce) & self.drained & (ack.output == payload) & ~self.protocol_error)]
        native += If(ack.send & ~ack.busy, ack_sent.eq(request.output), ack_valid.eq(1))
