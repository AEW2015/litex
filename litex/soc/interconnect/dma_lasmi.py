from litex.gen import *
from litex.gen.genlib.fifo import SyncFIFO

from litex.soc.interconnect import stream

class Reader(Module):
    def __init__(self, lasmim, fifo_depth=None):
        self.sink = sink = stream.Endpoint([("address", lasmim.aw)])
        self.source = source = stream.Endpoint([("data", lasmim.dw)])
        self.busy = Signal()

        # # #

        if fifo_depth is None:
            fifo_depth = lasmim.req_queue_size + lasmim.read_latency + 2

        # request issuance
        request_enable = Signal()
        request_issued = Signal()

        self.comb += [
            lasmim.we.eq(0),
            lasmim.stb.eq(sink.valid & request_enable),
            lasmim.adr.eq(sink.address),
            sink.ready.eq(lasmim.req_ack & request_enable),
            request_issued.eq(lasmim.stb & lasmim.req_ack)
        ]

        # FIFO reservation level counter
        # incremented when data is planned to be queued
        # decremented when data is dequeued
        data_dequeued = Signal()
        rsv_level = Signal(max=fifo_depth+1)
        self.sync += [
            If(request_issued,
                If(~data_dequeued, rsv_level.eq(rsv_level + 1))
            ).Elif(data_dequeued,
                rsv_level.eq(rsv_level - 1)
            )
        ]
        self.comb += [
            self.busy.eq(rsv_level != 0),
            request_enable.eq(rsv_level != fifo_depth)
        ]

        # FIFO
        fifo = SyncFIFO(lasmim.dw, fifo_depth)
        self.submodules += fifo

        self.comb += [
            fifo.din.eq(lasmim.dat_r),
            fifo.we.eq(lasmim.dat_r_ack),

            source.valid.eq(fifo.readable),
            fifo.re.eq(source.ready),
            source.data.eq(fifo.dout),
            data_dequeued.eq(source.valid & source.ready)
        ]


class Writer(Module):
    def __init__(self, lasmim, fifo_depth=None):
        self.source = source = stream.Endpoint([("address", lasmim.aw),
                                                ("data", lasmim.dw)])
        self.busy = Signal()

        # # #

        if fifo_depth is None:
            fifo_depth = lasmim.req_queue_size + lasmim.write_latency + 2

        fifo = SyncFIFO(lasmim.dw, fifo_depth)
        self.submodules += fifo

        self.comb += [
            lasmim.we.eq(1),
            lasmim.stb.eq(fifo.writable & source.valid),
            lasmim.adr.eq(source.address),
            source.ready.eq(fifo.writable & lasmim.req_ack),
            fifo.we.eq(source.valid & lasmim.req_ack),
            fifo.din.eq(source.data)
        ]

        self.comb += [
            If(lasmim.dat_w_ack,
                fifo.re.eq(1),
                lasmim.dat_we.eq(2**(lasmim.dw//8)-1),
                lasmim.dat_w.eq(fifo.dout)
            ),
            self.busy.eq(fifo.readable)
        ]
