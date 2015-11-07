from migen import *
from migen.genlib.record import *
from migen.genlib import fifo


def _make_m2s(layout):
    r = []
    for f in layout:
        if isinstance(f[1], (int, tuple)):
            r.append((f[0], f[1], DIR_M_TO_S))
        else:
            r.append((f[0], _make_m2s(f[1])))
    return r


class EndpointDescription:
    def __init__(self, payload_layout, packetized=False):
        self.payload_layout = payload_layout
        self.packetized = packetized

    def get_full_layout(self):
        reserved = {"stb", "ack", "payload", "sop", "eop", "description"}
        attributed = set()
        for f in self.payload_layout:
            if f[0] in attributed:
                raise ValueError(f[0] + " already attributed in payload layout")
            if f[0] in reserved:
                raise ValueError(f[0] + " cannot be used in endpoint layout")
            attributed.add(f[0])

        full_layout = [
            ("payload", _make_m2s(self.payload_layout)),
            ("stb", 1, DIR_M_TO_S),
            ("ack", 1, DIR_S_TO_M)
        ]
        if self.packetized:
            full_layout += [
                ("sop", 1, DIR_M_TO_S),
                ("eop", 1, DIR_M_TO_S)
            ]
        return full_layout


class _Endpoint(Record):
    def __init__(self, description_or_layout):
        if isinstance(description_or_layout, EndpointDescription):
            self.description = description_or_layout
        else:
            self.description = EndpointDescription(description_or_layout)
        Record.__init__(self, self.description.get_full_layout())

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, "payload"), name)


class Source(_Endpoint):
    def connect(self, sink):
        return Record.connect(self, sink)


class Sink(_Endpoint):
    def connect(self, source):
        return source.connect(self)


class _FIFOWrapper(Module):
    def __init__(self, fifo_class, layout, depth):
        self.sink = Sink(layout)
        self.source = Source(layout)
        self.busy = Signal()

        ###

        description = self.sink.description
        fifo_layout = [("payload", description.payload_layout)]
        if description.packetized:
            fifo_layout += [("sop", 1), ("eop", 1)]

        self.submodules.fifo = fifo_class(layout_len(fifo_layout), depth)
        fifo_in = Record(fifo_layout)
        fifo_out = Record(fifo_layout)
        self.comb += [
            self.fifo.din.eq(fifo_in.raw_bits()),
            fifo_out.raw_bits().eq(self.fifo.dout)
        ]

        self.comb += [
            self.sink.ack.eq(self.fifo.writable),
            self.fifo.we.eq(self.sink.stb),
            fifo_in.payload.eq(self.sink.payload),

            self.source.stb.eq(self.fifo.readable),
            self.source.payload.eq(fifo_out.payload),
            self.fifo.re.eq(self.source.ack)
        ]
        if description.packetized:
            self.comb += [
                fifo_in.sop.eq(self.sink.sop),
                fifo_in.eop.eq(self.sink.eop),
                self.source.sop.eq(fifo_out.sop),
                self.source.eop.eq(fifo_out.eop)
            ]


class SyncFIFO(_FIFOWrapper):
    def __init__(self, layout, depth, buffered=False):
        _FIFOWrapper.__init__(
            self,
            fifo.SyncFIFOBuffered if buffered else fifo.SyncFIFO,
            layout, depth)


class AsyncFIFO(_FIFOWrapper):
    def __init__(self, layout, depth):
        _FIFOWrapper.__init__(self, fifo.AsyncFIFO, layout, depth)


class Multiplexer(Module):
    def __init__(self, layout, n):
        self.source = Source(layout)
        sinks = []
        for i in range(n):
            sink = Sink(layout)
            setattr(self, "sink"+str(i), sink)
            sinks.append(sink)
        self.sel = Signal(max=n)

        # # #

        cases = {}
        for i, sink in enumerate(sinks):
            cases[i] = Record.connect(sink, self.source)
        self.comb += Case(self.sel, cases)


class Demultiplexer(Module):
    def __init__(self, layout, n):
        self.sink = Sink(layout)
        sources = []
        for i in range(n):
            source = Source(layout)
            setattr(self, "source"+str(i), source)
            sources.append(source)
        self.sel = Signal(max=n)

        # # #

        cases = {}
        for i, source in enumerate(sources):
            cases[i] = Record.connect(self.sink, source)
        self.comb += Case(self.sel, cases)

# TODO: clean up code below
# XXX

from copy import copy
from migen.util.misc import xdir

def pack_layout(l, n):
    return [("chunk"+str(i), l) for i in range(n)]

def get_endpoints(obj, filt=_Endpoint):
    if hasattr(obj, "get_endpoints") and callable(obj.get_endpoints):
        return obj.get_endpoints(filt)
    r = dict()
    for k, v in xdir(obj, True):
        if isinstance(v, filt):
            r[k] = v
    return r

def get_single_ep(obj, filt):
    eps = get_endpoints(obj, filt)
    if len(eps) != 1:
        raise ValueError("More than one endpoint")
    return list(eps.items())[0]


class BinaryActor(Module):
    def __init__(self, *args, **kwargs):
        self.busy = Signal()
        sink = get_single_ep(self, Sink)[1]
        source = get_single_ep(self, Source)[1]
        self.build_binary_control(sink, source, *args, **kwargs)

    def build_binary_control(self, sink, source):
        raise NotImplementedError("Binary actor classes must overload build_binary_control_fragment")


class CombinatorialActor(BinaryActor):
    def build_binary_control(self, sink, source):
        self.comb += [
            source.stb.eq(sink.stb),
            sink.ack.eq(source.ack),
            self.busy.eq(0)
        ]
        if sink.description.packetized:
            self.comb += [
                source.sop.eq(sink.sop),
                source.eop.eq(sink.eop)
            ]


class Unpack(Module):
    def __init__(self, n, layout_to, reverse=False):
        self.source = source = Source(layout_to)
        description_from = copy(source.description)
        description_from.payload_layout = pack_layout(description_from.payload_layout, n)
        self.sink = sink = Sink(description_from)

        self.busy = Signal()

        ###

        mux = Signal(max=n)
        first = Signal()
        last = Signal()
        self.comb += [
            first.eq(mux == 0),
            last.eq(mux == (n-1)),
            source.stb.eq(sink.stb),
            sink.ack.eq(last & source.ack)
        ]
        self.sync += [
            If(source.stb & source.ack,
                If(last,
                    mux.eq(0)
                ).Else(
                    mux.eq(mux + 1)
                )
            )
        ]
        cases = {}
        for i in range(n):
            chunk = n-i-1 if reverse else i
            cases[i] = [source.payload.raw_bits().eq(getattr(sink.payload, "chunk"+str(chunk)).raw_bits())]
        self.comb += Case(mux, cases).makedefault()

        if description_from.packetized:
            self.comb += [
                source.sop.eq(sink.sop & first),
                source.eop.eq(sink.eop & last)
            ]


class Pack(Module):
    def __init__(self, layout_from, n, reverse=False):
        self.sink = sink = Sink(layout_from)
        description_to = copy(sink.description)
        description_to.payload_layout = pack_layout(description_to.payload_layout, n)
        self.source = source = Source(description_to)
        self.busy = Signal()

        ###

        demux = Signal(max=n)

        load_part = Signal()
        strobe_all = Signal()
        cases = {}
        for i in range(n):
            chunk = n-i-1 if reverse else i
            cases[i] = [getattr(source.payload, "chunk"+str(chunk)).raw_bits().eq(sink.payload.raw_bits())]
        self.comb += [
            self.busy.eq(strobe_all),
            sink.ack.eq(~strobe_all | source.ack),
            source.stb.eq(strobe_all),
            load_part.eq(sink.stb & sink.ack)
        ]

        if description_to.packetized:
            demux_last = ((demux == (n - 1)) | sink.eop)
        else:
            demux_last = (demux == (n - 1))

        self.sync += [
            If(source.ack, strobe_all.eq(0)),
            If(load_part,
                Case(demux, cases),
                If(demux_last,
                    demux.eq(0),
                    strobe_all.eq(1)
                ).Else(
                    demux.eq(demux + 1)
                )
            )
        ]

        if description_to.packetized:
            self.sync += [
                If(source.stb & source.ack,
                    source.sop.eq(sink.sop),
                    source.eop.eq(sink.eop),
                ).Elif(sink.stb & sink.ack,
                    source.sop.eq(sink.sop | source.sop),
                    source.eop.eq(sink.eop | source.eop)
                )
            ]


class Chunkerize(CombinatorialActor):
    def __init__(self, layout_from, layout_to, n, reverse=False):
        self.sink = Sink(layout_from)
        if isinstance(layout_to, EndpointDescription):
            layout_to = copy(layout_to)
            layout_to.payload_layout = pack_layout(layout_to.payload_layout, n)
        else:
            layout_to = pack_layout(layout_to, n)
        self.source = Source(layout_to)
        CombinatorialActor.__init__(self)

        ###

        for i in range(n):
            chunk = n-i-1 if reverse else i
            for f in self.sink.description.payload_layout:
                src = getattr(self.sink, f[0])
                dst = getattr(getattr(self.source, "chunk"+str(chunk)), f[0])
                self.comb += dst.eq(src[i*len(src)//n:(i+1)*len(src)//n])


class Unchunkerize(CombinatorialActor):
    def __init__(self, layout_from, n, layout_to, reverse=False):
        if isinstance(layout_from, EndpointDescription):
            fields = layout_from.payload_layout
            layout_from = copy(layout_from)
            layout_from.payload_layout = pack_layout(layout_from.payload_layout, n)
        else:
            fields = layout_from
            layout_from = pack_layout(layout_from, n)
        self.sink = Sink(layout_from)
        self.source = Source(layout_to)
        CombinatorialActor.__init__(self)

        ###

        for i in range(n):
            chunk = n-i-1 if reverse else i
            for f in fields:
                src = getattr(getattr(self.sink, "chunk"+str(chunk)), f[0])
                dst = getattr(self.source, f[0])
                self.comb += dst[i*len(dst)//n:(i+1)*len(dst)//n].eq(src)


class Converter(Module):
    def __init__(self, layout_from, layout_to, reverse=False):
        self.sink = Sink(layout_from)
        self.source = Source(layout_to)
        self.busy = Signal()

        ###

        width_from = len(self.sink.payload.raw_bits())
        width_to = len(self.source.payload.raw_bits())

        # downconverter
        if width_from > width_to:
            if width_from % width_to:
                raise ValueError
            ratio = width_from//width_to
            self.submodules.chunkerize = Chunkerize(layout_from, layout_to, ratio, reverse)
            self.submodules.unpack = Unpack(ratio, layout_to)

            self.comb += [
                Record.connect(self.sink, self.chunkerize.sink),
                Record.connect(self.chunkerize.source, self.unpack.sink),
                Record.connect(self.unpack.source, self.source),
                self.busy.eq(self.unpack.busy)
            ]
        # upconverter
        elif width_to > width_from:
            if width_to % width_from:
                raise ValueError
            ratio = width_to//width_from
            self.submodules.pack = Pack(layout_from, ratio)
            self.submodules.unchunkerize = Unchunkerize(layout_from, ratio, layout_to, reverse)

            self.comb += [
                Record.connect(self.sink, self.pack.sink),
                Record.connect(self.pack.source, self.unchunkerize.sink),
                Record.connect(self.unchunkerize.source, self.source),
                self.busy.eq(self.pack.busy)
            ]
        # direct connection
        else:
            self.comb += Record.connect(self.sink, self.source)

# XXX
