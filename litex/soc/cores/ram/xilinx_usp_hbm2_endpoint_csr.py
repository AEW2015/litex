"""Processor-owned application streams for the four-command HBM endpoint.

CSR staging writes never alter an enqueued command. Submit/pop writes use bit0;
rejected operations set sticky errors and do not consume anything. Data lane0 is
bits0..31. B/R remain at the endpoint until an explicit valid pop, including the
last R word that releases its endpoint credit.

STOP closes new descriptors immediately. Previously accepted descriptors must
leave the application queues before endpoint_quiesce closes endpoint admission.
Their W words and responses can still complete during STOP or READY loss. The
board must keep native admission open until this adapter has drained, then use
the endpoint's current-generation quiescence acknowledgement before retuning.
"""

from migen import Cat, If, Mux, Signal

from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStatus, CSRStorage


class HBMEndpointCSR(LiteXModule):
    def __init__(self, endpoint):
        if endpoint.max_outstanding != 4:
            raise ValueError("The CSR adapter requires the four-command endpoint")
        if len(endpoint.aw.addr) != 33 or len(endpoint.aw.id) != 6:
            raise ValueError("The CSR adapter requires 33-bit addresses and 6-bit IDs")
        self.admission_enable = Signal()
        self.stop = Signal()
        self.endpoint_quiesce = Signal()
        self.pending = Signal()
        self.drained = Signal()
        self.idle = self.drained
        self.quiesced = Signal()
        self._csrs = []

        def storage(name, width):
            csr = CSRStorage(width, name=name)
            setattr(self, name, csr)
            self._csrs.append(csr)
            return csr

        def status(name, width):
            csr = CSRStatus(width, name=name)
            setattr(self, name, csr)
            self._csrs.append(csr)
            return csr

        for direction in ("aw", "ar"):
            for field, width in (("addr_lo", 32), ("addr_hi", 1), ("len", 4),
                                 ("id", 6), ("submit", 1)):
                storage(direction + "_" + field, width)
        for lane in range(16):
            storage("w_data{}".format(lane), 32)
        for field, width in (("strb_lo", 32), ("strb_hi", 32), ("last", 1), ("submit", 1)):
            storage("w_" + field, width)
        status("b_id", 6)
        status("b_resp", 2)
        storage("b_pop", 1)
        for lane in range(16):
            status("r_data{}".format(lane), 32)
        for field, width in (("keep_lo", 32), ("keep_hi", 32), ("id", 12),
                             ("resp", 4), ("first", 1), ("last", 1)):
            status("r_" + field, width)
        storage("r_pop", 1)
        status("status", 32)
        status("errors", 5)
        storage("clear_errors", 5)
        for name, width in (("aw_level", 3), ("ar_level", 3), ("w_level", 6),
                            ("write_pending", 4), ("read_pending", 4),
                            ("write_credits", 3), ("read_credits", 3)):
            status(name, width)

        # # #

        desc = [("addr", 33), ("len", 4), ("id", 6)]
        self.aw_queue = aw_queue = stream.SyncFIFO(desc, 4)
        self.ar_queue = ar_queue = stream.SyncFIFO(desc, 4)
        self.w_queue = w_queue = stream.SyncFIFO([("data", 512), ("strb", 64)], 32)
        # At most four queued plus four endpoint-owned descriptors. Lengths
        # persist independently of AW dispatch until all CPU W words arrive.
        self.length_queue = lengths = stream.SyncFIFO([("words_minus_one", 3)], 8)
        write_pending = Signal(4)
        read_pending = Signal(4)
        word_index = Signal(3)
        admission_open = Signal()
        aw_accept, ar_accept, w_accept = Signal(), Signal(), Signal()
        aw_request = self.aw_submit.re & self.aw_submit.storage
        ar_request = self.ar_submit.re & self.ar_submit.storage
        w_request = self.w_submit.re & self.w_submit.storage
        b_request = self.b_pop.re & self.b_pop.storage
        r_request = self.r_pop.re & self.r_pop.storage
        self.comb += [
            admission_open.eq(self.admission_enable & ~self.stop & endpoint.link_ready & ~endpoint.protocol_error),
            aw_queue.source.connect(endpoint.aw),
            ar_queue.source.connect(endpoint.ar),
            w_queue.source.connect(endpoint.w),
            self.endpoint_quiesce.eq(self.stop & ~aw_queue.source.valid & ~ar_queue.source.valid),
        ]
        capacities = []
        for direction, queue, request, accepted, count in (
                ("aw", aw_queue, aw_request, aw_accept, write_pending),
                ("ar", ar_queue, ar_request, ar_accept, read_pending)):
            address = Cat(getattr(self, direction + "_addr_lo").storage,
                          getattr(self, direction + "_addr_hi").storage)
            length = getattr(self, direction + "_len").storage
            legal = (address[:5] == 0) & ((address[:12] + ((length + 1) << 5)) <= 4096)
            capacity = queue.sink.ready & (count < 8)
            if direction == "aw":
                capacity = capacity & lengths.sink.ready
            capacities.append(capacity)
            self.comb += [
                accepted.eq(request & admission_open & capacity & legal),
                queue.sink.valid.eq(accepted), queue.sink.addr.eq(address),
                queue.sink.len.eq(length),
                queue.sink.id.eq(getattr(self, direction + "_id").storage),
            ]
        last_word = word_index == lengths.source.words_minus_one
        w_capacity = w_queue.sink.ready & lengths.source.valid
        self.comb += [
            lengths.sink.valid.eq(aw_accept),
            lengths.sink.words_minus_one.eq(self.aw_len.storage >> 1),
            lengths.source.ready.eq(w_accept & last_word),
            w_accept.eq(w_request & w_capacity & (self.w_last.storage == last_word)),
            w_queue.sink.valid.eq(w_accept),
            w_queue.sink.data.eq(Cat(*[getattr(self, "w_data{}".format(i)).storage for i in range(16)])),
            w_queue.sink.strb.eq(Cat(self.w_strb_lo.storage, self.w_strb_hi.storage)),
            w_queue.sink.last.eq(self.w_last.storage),
            w_queue.sink.first.eq(word_index == 0),
            endpoint.b.ready.eq(b_request & endpoint.b.valid),
            endpoint.r.ready.eq(r_request & endpoint.r.valid),
        ]
        b_pop = endpoint.b.valid & endpoint.b.ready
        r_done = endpoint.r.valid & endpoint.r.ready & endpoint.r.last
        for count, accepted, retired in ((write_pending, aw_accept, b_pop),
                                         (read_pending, ar_accept, r_done)):
            self.sync += If(accepted & ~retired, count.eq(count + 1)).Elif(
                retired & ~accepted, count.eq(count - 1))
        self.sync += If(w_accept,
            If(last_word, word_index.eq(0)).Else(word_index.eq(word_index + 1)))
        rejected = Cat(aw_request & ~aw_accept, ar_request & ~ar_accept,
                       w_request & ~w_accept, b_request & ~endpoint.b.valid,
                       r_request & ~endpoint.r.valid)
        clear = Signal(5)
        self.comb += clear.eq(Mux(self.clear_errors.re, self.clear_errors.storage, 0))
        self.sync += self.errors.status.eq((self.errors.status & ~clear) | rejected)

        self.comb += [
            self.pending.eq(aw_queue.source.valid | ar_queue.source.valid | w_queue.source.valid |
                lengths.source.valid | (write_pending != 0) | (read_pending != 0) | ~endpoint.drained),
            self.drained.eq(~self.pending & ~endpoint.protocol_error),
            self.quiesced.eq(self.drained & endpoint.quiesced),
            self.status.status.eq(Cat(admission_open, capacities[0], capacities[1], w_capacity,
                endpoint.b.valid, endpoint.r.valid, self.pending, self.drained, self.quiesced,
                self.stop, endpoint.protocol_error, endpoint.link_ready)),
            self.aw_level.status.eq(aw_queue.level), self.ar_level.status.eq(ar_queue.level),
            self.w_level.status.eq(w_queue.level),
            self.write_pending.status.eq(write_pending), self.read_pending.status.eq(read_pending),
            self.write_credits.status.eq(endpoint.write_credits),
            self.read_credits.status.eq(endpoint.read_credits),
            self.b_id.status.eq(endpoint.b.id), self.b_resp.status.eq(endpoint.b.resp),
            self.r_keep_lo.status.eq(endpoint.r.keep[:32]), self.r_keep_hi.status.eq(endpoint.r.keep[32:]),
            self.r_id.status.eq(endpoint.r.id), self.r_resp.status.eq(endpoint.r.resp),
            self.r_first.status.eq(endpoint.r.first), self.r_last.status.eq(endpoint.r.last),
        ]
        for lane in range(16):
            self.comb += getattr(self, "r_data{}".format(lane)).status.eq(endpoint.r.data[32*lane:32*(lane + 1)])

    def get_csrs(self):
        return self._csrs
