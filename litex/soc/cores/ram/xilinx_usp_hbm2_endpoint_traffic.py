"""Application-side traffic and data checking for the ordered HBM endpoint.

The default region is PC31 local 0x200000..0x3fffff. The legacy profile uses
16 native beats and eight 512-bit application words per command. SEED writes the whole region once;
WRITE, READ and MIXED offer commands for a captured number of SYS clocks. READ
and MIXED require a completed SEED with the same seed. A mixed write stores the
same address-dependent value as SEED, so either read/write ordering is valid.

STOP, time expiry and READY loss close new offers. An already offered AW/AR
remains stable until accepted; its data and all responses finish normally.
The board must use endpoint_quiesce and retain native admission until drained.
Elapsed cycles include draining, and latency measures application acceptance
through B or final packed R consumption. Event counters are 32-bit and enabled
write bytes use 64 bits. They saturate on overflow and set an error; other byte
totals belong in 64-bit firmware/host arithmetic.
IDs are captured seed bits 5:0 XOR 0x15 for writes and XOR 0x2a for reads;
they remain constant within a run, preserving the endpoint's one-ID lock.

The optional random profile adds variable-length timed traffic, a finite
MASK_EPOCH and full-region VERIFY. SEED establishes the hashed baseline;
MASK_EPOCH complements enabled bytes once, leaving disabled bytes and tails
unchanged. VERIFY always reads every byte. Timed writes preserve this known
image, making simultaneous reads race-free. Partial epochs invalidate it.
"""

from migen import Case, Cat, If, Mux, Replicate, Signal

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

        for name, width in (("config_mode", 3), ("config_cycles", 32), ("config_seed", 32),
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
        # Append-only extension: the original 24 CSR offsets remain unchanged.
        storage("config_random", 1)
        enabled_bytes = Signal(64)
        self.comb += [status("write_enabled_bytes_lo", 32).status.eq(enabled_bytes[:32]),
                      status("write_enabled_bytes_hi", 32).status.eq(enabled_bytes[32:])]
        for name in ("aw_native_beats", "ar_native_beats", "aw_packed_words", "ar_packed_words"):
            counters[name] = Signal(32)
            self.comb += status(name, 32).status.eq(counters[name])
        aw_seen = Signal(16)
        ar_seen = Signal(16)
        self.comb += [status("aw_length_seen", 16).status.eq(aw_seen),
                      status("ar_length_seen", 16).status.eq(ar_seen)]

        # # #

        mode = Signal(3)
        duration = Signal(32)
        seed = Signal(32)
        random = Signal()
        seeded = Signal()
        seeded_value = Signal(32)
        seeded_random = Signal()
        masked = Signal()
        verified = Signal()
        active_masked = Signal()
        seed_allowed = Signal()
        done = Signal()
        aw_offer, ar_offer = Signal(), Signal()
        aw_index, ar_index = Signal(12), Signal(12)
        aw_address, ar_address = Signal(33), Signal(33)
        aw_length, ar_length = Signal(4), Signal(4)
        aw_random, ar_random = Signal(32), Signal(32)
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
        requested = self.config_mode.storage
        matching = seeded & (self.config_seed.storage == seeded_value) & (self.config_random.storage == seeded_random)
        finite = (mode == 0) | (mode == 4) | (mode == 5)
        config_valid = ((requested == 0) |
            ((requested >= 1) & (requested <= 3) & (self.config_cycles.storage != 0) &
                (self.config_cycles.storage <= (1 << 28))) |
            (((requested == 4) | (requested == 5)) & self.config_random.storage &
                (self.config_cycles.storage == 0)))
        seed_valid = ((requested == 0) | (matching & ((requested != 4) | ~masked)) |
            (~self.config_random.storage & (requested == 1)))
        self.comb += [
            start_accept.eq(start_request & ~busy & self.drained & self.admission_enable &
                ~self.stop & endpoint.link_ready & ~endpoint.protocol_error &
                (self.errors.status == 0) & config_valid & seed_valid),
            closing.eq(stop_pending | self.stop | ~self.admission_enable | (fatal != 0) |
                (~finite & (elapsed >= duration)) |
                (((mode == 0) | (mode == 4)) & (counters["aw_count"] == region_bursts)) |
                ((mode == 5) & (counters["ar_count"] == region_bursts))),
            issue.eq(busy & ~closing),
        ]
        self.write_queue = writes = stream.SyncFIFO([("addr", 33), ("len", 4)], 4)
        self.write_times = write_times = stream.SyncFIFO([("time", 32)], 4)
        self.read_queue = reads = stream.SyncFIFO([("addr", 33), ("len", 4), ("time", 32)], 4)
        aw_fire = endpoint.aw.valid & endpoint.aw.ready
        ar_fire = endpoint.ar.valid & endpoint.ar.ready
        w_fire = endpoint.w.valid & endpoint.w.ready
        b_fire = endpoint.b.valid & endpoint.b.ready
        r_fire = endpoint.r.valid & endpoint.r.ready
        r_last = r_fire & endpoint.r.last
        self.comb += [
            endpoint.aw.valid.eq(aw_offer), endpoint.aw.addr.eq(aw_address),
            endpoint.aw.len.eq(aw_length), endpoint.aw.id.eq(seed[:6] ^ 0x15),
            endpoint.ar.valid.eq(ar_offer), endpoint.ar.addr.eq(ar_address),
            endpoint.ar.len.eq(ar_length), endpoint.ar.id.eq(seed[:6] ^ 0x2a),
            writes.sink.valid.eq(aw_fire), writes.sink.addr.eq(aw_address),
            writes.sink.len.eq(aw_length),
            write_times.sink.valid.eq(aw_fire), write_times.sink.time.eq(elapsed),
            reads.sink.valid.eq(ar_fire), reads.sink.addr.eq(ar_address), reads.sink.time.eq(elapsed),
            reads.sink.len.eq(ar_length),
            endpoint.w.valid.eq(writes.source.valid),
            endpoint.w.first.eq(write_index == 0), endpoint.w.last.eq(write_index == (writes.source.len >> 1)),
            writes.source.ready.eq(w_fire & endpoint.w.last),
            endpoint.b.ready.eq(1), endpoint.r.ready.eq(1),
            write_times.source.ready.eq(b_fire & write_times.source.valid),
            reads.source.ready.eq(r_last & reads.source.valid),
            self.endpoint_quiesce.eq(self.stop & ~aw_offer & ~ar_offer),
        ]
        write_word = Signal(32)
        read_word = Signal(32)
        self.comb += [write_word.eq((writes.source.addr >> 2) | (write_index << 4)),
                      read_word.eq((reads.source.addr >> 2) | (read_index << 4))]

        def hash32(value):
            a, b, c = Signal(32), Signal(32), Signal(32)
            self.comb += [a.eq(value ^ (value << 13)), b.eq(a ^ (a >> 17)), c.eq(b ^ (b << 5))]
            return c

        def mask_length(slot):
            return (slot ^ (slot >> 4) ^ (slot >> 8) ^ seed)[:4]

        def byte_mask(address):
            slot = address[9:21]
            native_word = address >> 5
            selector = slot[:3]
            onehot = Signal(32)
            result = Signal(32)
            self.comb += [onehot.eq(1 << ((native_word ^ seed)[:5])),
                Case(selector, {0: result.eq(0), 1: result.eq(0xffffffff),
                    2: result.eq(0xaaaaaaaa), 3: result.eq(0x55555555),
                    4: result.eq(onehot), 5: result.eq(~onehot),
                    6: result.eq(hash32(seed ^ native_word ^ 0x51f15e5d)),
                    7: result.eq(Mux(native_word[0], 0xffff0000, 0x0000ffff))})]
            return result

        write_upper = (write_index != (writes.source.len >> 1)) | writes.source.len[0]
        read_upper = (read_index != (reads.source.len >> 1)) | reads.source.len[0]
        write_masks, read_masks = [], []
        for half in range(2):
            wa, ra = Signal(33), Signal(33)
            self.comb += [wa.eq(writes.source.addr | (write_index << 6) | (half << 5)),
                          ra.eq(reads.source.addr | (read_index << 6) | (half << 5))]
            wm, rm = byte_mask(wa), byte_mask(ra)
            write_masks.append(wm)
            read_masks.append(rm)
            self.comb += endpoint.w.strb[32*half:32*(half + 1)].eq(
                Mux((half == 0) | write_upper,
                    Mux(random & (mode != 0), wm, 0xffffffff), 0))
        mismatch = Signal()
        bad_data = 0
        for lane in range(16):
            half = lane // 8
            wb, rb = Signal(32), Signal(32)
            self.comb += [wb.eq(Mux(random, hash32(seed ^ write_word ^ lane), seed ^ write_word ^ lane)),
                          rb.eq(Mux(random, hash32(seed ^ read_word ^ lane), seed ^ read_word ^ lane))]
            w_enable = random & active_masked & (((write_index << 1) | half) <= mask_length(writes.source.addr[9:21]))
            r_enable = random & active_masked & (((read_index << 1) | half) <= mask_length(reads.source.addr[9:21]))
            w_flip = Cat(*[Replicate(write_masks[half][4*(lane % 8) + byte] & w_enable, 8) for byte in range(4)])
            r_flip = Cat(*[Replicate(read_masks[half][4*(lane % 8) + byte] & r_enable, 8) for byte in range(4)])
            enabled = Cat(*[Replicate(write_masks[half][4*(lane % 8) + byte], 8) for byte in range(4)])
            expected = rb ^ r_flip
            # Poison disabled bytes during idempotent random writes: an
            # accidentally widened strobe must alter the known image.
            self.comb += endpoint.w.data[32*lane:32*(lane + 1)].eq(
                Mux(mode == 4, ~wb, (wb ^ w_flip) ^ Mux(random & (mode != 0), ~enabled, 0)))
            bad_data = bad_data | ((endpoint.r.data[32*lane:32*(lane + 1)] != expected) &
                ((half == 0) | read_upper))
        self.comb += mismatch.eq(bad_data)
        read_id = seed[:6] ^ 0x2a
        expected_keep = Cat(0xffffffff, Mux(read_upper, 0xffffffff, 0))
        metadata = ((endpoint.r.keep != expected_keep) | (endpoint.r.first != (read_index == 0)) |
            (endpoint.r.last != (read_index == (reads.source.len >> 1))) |
            (endpoint.r.id[:6] != read_id) | (read_upper & (endpoint.r.id[6:] != read_id)))
        unexpected = ((b_fire & ~write_times.source.valid) | (r_fire & ~reads.source.valid))
        failures = Cat(r_fire & reads.source.valid & mismatch,
            r_fire & reads.source.valid & metadata,
            (b_fire & ((endpoint.b.id != (seed[:6] ^ 0x15)) | (endpoint.b.resp != 0))) |
                (r_fire & ((endpoint.r.resp[:2] != 0) | (read_upper & (endpoint.r.resp[2:] != 0)))),
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
                self.drained, self.quiesced, endpoint.link_ready, endpoint.protocol_error, masked, verified)),
        ]
        bits = max(1, (region_bursts - 1).bit_length())
        permutation = Cat(*[aw_index[bits - 1 - i] for i in range(bits)]) ^ (seed & (region_bursts - 1))
        aw_slot, ar_slot = Signal(12), Signal(12)
        aw_pick_length, ar_pick_length = Signal(4), Signal(4)
        self.comb += [
            aw_slot.eq(Mux(random & (mode != 0), Mux(mode == 4, permutation,
                aw_random & (region_bursts - 1)), aw_index)),
            ar_slot.eq(Mux(random & (mode != 5), ar_random & (region_bursts - 1), ar_index)),
            aw_pick_length.eq(Mux(random & (mode != 0),
                Mux(mode == 4, mask_length(aw_slot), aw_random[12:16]), 15)),
            ar_pick_length.eq(Mux(random & (mode != 5), ar_random[12:16], 15)),
        ]
        next_aw, next_ar = hash32(aw_random), hash32(ar_random)
        initial_aw = self.config_seed.storage ^ 0x9e3779b9
        initial_ar = self.config_seed.storage ^ 0x243f6a88
        self.sync += [
            If(start_accept,
                mode.eq(self.config_mode.storage), duration.eq(self.config_cycles.storage),
                seed.eq(self.config_seed.storage), busy.eq(1), done.eq(0), seed_allowed.eq(1),
                random.eq(self.config_random.storage), active_masked.eq(masked),
                aw_random.eq(Mux(initial_aw == 0, 1, initial_aw)),
                ar_random.eq(Mux(initial_ar == 0, 1, initial_ar)),
                stop_pending.eq(0),
                aw_index.eq(0), ar_index.eq(0), write_index.eq(0), read_index.eq(0),
                If((self.config_mode.storage == 0) |
                    ((self.config_mode.storage == 1) & ~matching),
                    seeded.eq(0), masked.eq(0), active_masked.eq(0)),
                If((requested == 0) | (requested == 1) | (requested == 3) | (requested == 4), verified.eq(0))),
            If(~self.admission_enable | self.stop | (fatal != 0),
                seeded.eq(0), seed_allowed.eq(0), masked.eq(0), verified.eq(0)),
            If(busy & closing, stop_pending.eq(1)),
            If(issue & ((mode == 0) | (mode == 1) | (mode == 3) | (mode == 4)) &
                ~aw_offer & writes.sink.ready & write_times.sink.ready,
                aw_offer.eq(1), aw_address.eq((31 << 28) + 0x200000 + (aw_slot << 9)), aw_length.eq(aw_pick_length)),
            If(issue & ((mode == 2) | (mode == 3) | (mode == 5)) & ~ar_offer & reads.sink.ready,
                ar_offer.eq(1), ar_address.eq((31 << 28) + 0x200000 + (ar_slot << 9)), ar_length.eq(ar_pick_length)),
            If(aw_fire, aw_offer.eq(0), aw_random.eq(next_aw),
                If(aw_index == region_bursts - 1, aw_index.eq(0)).Else(aw_index.eq(aw_index + 1))),
            If(ar_fire, ar_offer.eq(0), ar_random.eq(next_ar),
                If(ar_index == region_bursts - 1, ar_index.eq(0)).Else(ar_index.eq(ar_index + 1))),
            If(w_fire, If(endpoint.w.last, write_index.eq(0)).Else(write_index.eq(write_index + 1))),
            If(r_fire, If(endpoint.r.last, read_index.eq(0)).Else(read_index.eq(read_index + 1))),
            If(finish,
                busy.eq(0), done.eq(1),
                If((mode == 0) & (counters["write_bursts"] == region_bursts) &
                    (fatal == 0) & seed_allowed & self.admission_enable & ~self.stop,
                    seeded.eq(1), seeded_value.eq(seed), seeded_random.eq(random)),
                If((mode == 4) & (counters["write_bursts"] == region_bursts) &
                    (fatal == 0) & seed_allowed & self.admission_enable & ~self.stop, masked.eq(1)),
                If((mode == 5) & (counters["read_bursts"] == region_bursts) &
                    (fatal == 0) & seed_allowed & self.admission_enable & ~self.stop, verified.eq(1))),
        ]
        for count, take, retire in ((write_outstanding, aw_fire, b_fire),
                                     (read_outstanding, ar_fire, r_last)):
            self.sync += If(take & ~retire, count.eq(count + 1)).Elif(retire & ~take,
                If(count != 0, count.eq(count - 1)))

        events = {"cycles": (busy, 1), "aw_count": (aw_fire, 1), "ar_count": (ar_fire, 1),
            "write_beats": (w_fire, Mux(write_upper, 2, 1)),
            "read_beats": (r_fire, Mux(read_upper, 2, 1)),
            "packed_w": (w_fire, 1), "packed_r": (r_fire, 1),
            "write_bursts": (b_fire, 1), "read_bursts": (r_last, 1),
            "aw_stalls": (endpoint.aw.valid & ~endpoint.aw.ready, 1),
            "w_stalls": (endpoint.w.valid & ~endpoint.w.ready, 1),
            "ar_stalls": (endpoint.ar.valid & ~endpoint.ar.ready, 1),
            "read_wait": ((read_outstanding != 0) & ~r_fire, 1),
            "aw_native_beats": (aw_fire, aw_length + 1),
            "ar_native_beats": (ar_fire, ar_length + 1),
            "aw_packed_words": (aw_fire, (aw_length >> 1) + 1),
            "ar_packed_words": (ar_fire, (ar_length >> 1) + 1)}
        # Overflow is an event; the CSR holds its sticky error. A retained
        # event level would prevent an idle W1C from clearing that error.
        self.sync += overflow.eq(0)
        for name, (event, step) in events.items():
            count = counters[name]
            self.sync += If(start_accept, count.eq(0)).Elif(busy & event,
                If(count <= 0xffffffff - step, count.eq(count + step)).Else(count.eq(0xffffffff), overflow.eq(1)))
        # Balanced popcount avoids a serial chain of 64 additions. Count only
        # actual W handshakes, including zero strobes and invalid odd-tail zero.
        terms = [endpoint.w.strb[bit] for bit in range(64)]
        while len(terms) > 1:
            terms = [terms[i] + terms[i + 1] for i in range(0, len(terms), 2)]
        byte_count = terms[0]
        self.sync += [
            If(start_accept, enabled_bytes.eq(0), aw_seen.eq(0), ar_seen.eq(0)).Else(
                If(w_fire,
                    If(enabled_bytes <= (1 << 64) - 1 - byte_count,
                        enabled_bytes.eq(enabled_bytes + byte_count)).Else(
                        enabled_bytes.eq((1 << 64) - 1), overflow.eq(1))),
                If(aw_fire, aw_seen.eq(aw_seen | (1 << aw_length))),
                If(ar_fire, ar_seen.eq(ar_seen | (1 << ar_length)))),
        ]
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
