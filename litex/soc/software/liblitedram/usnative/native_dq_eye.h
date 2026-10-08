/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Opt-in destructive per-DQ window measurement for generated USNative maps.
 * The caller must ensure this code, stack and buffers run outside the reserved
 * scratch range. The range is deliberately limited to a finite point count. */
#ifndef USNATIVE_DQ_EYE_H
#define USNATIVE_DQ_EYE_H

#include <generated/sdram_phy.h>
#include <libbase/memtest.h>
#include <libbase/lfsr.h>
#include <system.h>
#include "mapping.h"
#include "native_data_map.h"
#include "native_status_io.h"

#define NDQ_EYE_SCRATCH_OFFSET 0x01000000u
#define NDQ_EYE_MAX_POINTS 128u
#define NDQ_EYE_MAX_REPEATS 3u
#define NDQ_EYE_WORDS (MEMTEST_DATA_SIZE / sizeof(unsigned))
#define NDQ_EYE_MAX_TAP (SDRAM_PHY_DELAYS - 1u)
#define NDQ_EYE_DQ_COUNT SDRAM_PHY_DATABITS
#define NDQ_EYE_TAP_CHUNK 4u
#define NDQ_EYE_VTC_TOLERANCE 1u

#define NDQ_EYE_FIFO_WORDS ((SDRAM_PHY_USNATIVE_TAP_COUNT + 31u) / 32u)
#define NDQ_EYE_FIFO_UNAVAILABLE (-1)

/* DQ words are the read-data participants. DQS/DM and control FIFO status
 * stays in the snapshot, but sideband FIFOs need not be empty: normal lane
 * draining is qualified from DQ availability. */
static int ndq_eye_wait_dq_fifo_empty(unsigned words[NDQ_EYE_FIFO_WORDS])
{
#if defined(CSR_DDRPHY_FIFO_EMPTY_ADDR) && defined(CSR_DDRPHY_FIFO_EMPTY_SIZE)
    unsigned stable = 0;
    if (CSR_DDRPHY_FIFO_EMPTY_SIZE < NDQ_EYE_FIFO_WORDS ||
        SDRAM_PHY_USNATIVE_DQ_TAPS_COUNT != SDRAM_PHY_DATABITS) {
        for (unsigned word=0; word<NDQ_EYE_FIFO_WORDS; ++word) words[word] = 0;
        return NDQ_EYE_FIFO_UNAVAILABLE;
    }
    for (unsigned attempt=0; attempt<1000; ++attempt) {
        unsigned all_empty = 1;
        for (unsigned word=0; word<NDQ_EYE_FIFO_WORDS; ++word)
            words[word] = csr_read_simple(CSR_DDRPHY_FIFO_EMPTY_ADDR + 4u*word);
#if defined(CONFIG_CSR_ORDERING_BIG)
        #define NDQ_EYE_FIFO_WORD_INDEX(tap) (NDQ_EYE_FIFO_WORDS - 1u - ((tap) / 32u))
#else
        #define NDQ_EYE_FIFO_WORD_INDEX(tap) ((tap) / 32u)
#endif
        for (unsigned dq=0; dq<SDRAM_PHY_USNATIVE_DQ_TAPS_COUNT; ++dq) {
            unsigned tap = usnative_dq_taps[dq];
            if (tap >= SDRAM_PHY_USNATIVE_TAP_COUNT)
                return NDQ_EYE_FIFO_UNAVAILABLE;
            unsigned word = NDQ_EYE_FIFO_WORD_INDEX(tap);
            if (word >= NDQ_EYE_FIFO_WORDS) return NDQ_EYE_FIFO_UNAVAILABLE;
            if (!(words[word] & (1u << (tap % 32u)))) all_empty = 0;
        }
#undef NDQ_EYE_FIFO_WORD_INDEX
        if (all_empty) {
            if (++stable == 3u) return 1;
        } else {
            stable = 0;
        }
        cdelay(100);
    }
    return 0;
#else
    for (unsigned word=0; word<NDQ_EYE_FIFO_WORDS; ++word) words[word] = 0;
    return NDQ_EYE_FIFO_UNAVAILABLE;
#endif
}

static void ndq_eye_print_fifo_empty(const char *where,
    const unsigned words[NDQ_EYE_FIFO_WORDS])
{
    printf("DQWIN_FIFO_EMPTY,where=%s,words=", where);
    for (unsigned word=0; word<NDQ_EYE_FIFO_WORDS; ++word)
        printf("%s%08x", word ? ":" : "", words[word]);
    printf("\n");
}

enum ndq_eye_pattern {
    NDQ_EYE_PATTERN_PRBS32_SEEDED,
    NDQ_EYE_PATTERN_COMPLEMENTARY,
    NDQ_EYE_PATTERN_WALKING_BIT,
    NDQ_EYE_PATTERN_AGGRESSOR,
};

static int ndq_eye_tap_close(unsigned observed, unsigned requested)
{
    unsigned delta = observed > requested ? observed - requested : requested - observed;
    return delta <= NDQ_EYE_VTC_TOLERANCE;
}

/* A baseline guard validates the full word before a DQ-specific sweep. */
static int ndq_eye_guard_pass(unsigned errors, unsigned other_errors)
{
    return errors == 0 && other_errors == 0;
}

static int ndq_eye_wait_vtc(void)
{
    for (unsigned attempt=0; attempt<1000; ++attempt) {
        if (ddrphy_ready_read() && ddrphy_vtc_rdy_read()==USNATIVE_CONTROL_MASK) return 1;
        cdelay(1000);
    }
    return 0;
}

static unsigned ndq_eye_seed(unsigned tap, unsigned repeat)
{
    unsigned seed = 1u + ((tap * NDQ_EYE_MAX_REPEATS + repeat) & 0xffffu);
    return seed ? seed : 1u;
}

/* A 32-bit CPU word contains two x16 beats, but only one x32 beat. Map a
 * physical DQ to every corresponding CPU-word bit for either PHY width. */
static unsigned ndq_eye_dq_word_mask(unsigned dq, unsigned word_index)
{
    return usnative_cpu_word_dq_mask(dq, word_index);
}

static const char *ndq_eye_pattern_name(enum ndq_eye_pattern pattern)
{
    switch (pattern) {
    case NDQ_EYE_PATTERN_PRBS32_SEEDED: return "prbs32_seeded";
    case NDQ_EYE_PATTERN_COMPLEMENTARY: return "complementary";
    case NDQ_EYE_PATTERN_WALKING_BIT: return "walking_bit";
    case NDQ_EYE_PATTERN_AGGRESSOR: return "aggressor";
    default: return "invalid";
    }
}

/* Generate deterministic words for complementary, walking-one/zero, and
 * target-versus-aggressor traffic. For x16, one logical DQ maps to the same
 * bit position in both halves of the 32-bit CPU word. */
static unsigned ndq_eye_pattern_word(enum ndq_eye_pattern pattern, unsigned dq,
    unsigned index, unsigned seed, unsigned *prbs_state)
{
    switch (pattern) {
    case NDQ_EYE_PATTERN_PRBS32_SEEDED:
        *prbs_state = lfsr(32, *prbs_state);
        return *prbs_state;
    case NDQ_EYE_PATTERN_COMPLEMENTARY:
        return ((index + seed) & 1u) ? 0xaaaaaaaau : 0x55555555u;
    case NDQ_EYE_PATTERN_WALKING_BIT: {
        unsigned bit = (index / 2u + seed) & 31u;
        return (index & 1u) ? 1u << bit : 0u;
    }
    case NDQ_EYE_PATTERN_AGGRESSOR: {
        unsigned target = ndq_eye_dq_word_mask(dq, index);
        unsigned target_value = ((index + seed) & 1u) ? target : 0u;
        unsigned aggressors = ((index / 2u + seed) & 1u) ? ~target : 0u;
        return target_value | aggressors;
    }
    default:
        return 0u;
    }
}

static void ndq_eye_write_pattern(enum ndq_eye_pattern pattern, unsigned dq,
    unsigned seed)
{
    volatile unsigned *memory = (volatile unsigned *)(MAIN_RAM_BASE_VA + NDQ_EYE_SCRATCH_OFFSET);
    unsigned prbs_state = seed;
    for (unsigned i=0; i<NDQ_EYE_WORDS; ++i)
        memory[i] = ndq_eye_pattern_word(pattern, dq, i, seed, &prbs_state);
    flush_cpu_dcache_range((void *)memory, MEMTEST_DATA_SIZE);
    flush_l2_cache(); cdelay(10000);
}

/* One physical read per word, with no memtest retry. Return selected-DQ bit
 * errors and report other-bit errors separately for crosstalk diagnosis. */
static unsigned ndq_eye_check(unsigned dq, enum ndq_eye_pattern pattern,
    unsigned seed, unsigned *other_errors)
{
    volatile unsigned *memory = (volatile unsigned *)(MAIN_RAM_BASE_VA + NDQ_EYE_SCRATCH_OFFSET);
    unsigned errors = 0;
    *other_errors = 0;
    unsigned prbs_state = seed;
    flush_cpu_dcache_range((void *)memory, MEMTEST_DATA_SIZE);
    flush_l2_cache();
    invd_cpu_dcache_range((void *)memory, MEMTEST_DATA_SIZE);
    for (unsigned i=0; i<NDQ_EYE_WORDS; ++i) {
        unsigned expected = ndq_eye_pattern_word(pattern, dq, i, seed, &prbs_state);
        unsigned diff = memory[i] ^ expected;
        unsigned dq_mask = ndq_eye_dq_word_mask(dq, i);
        if (diff & dq_mask) ++errors;
        if (diff & ~dq_mask) ++*other_errors;
    }
    return errors;
}

static unsigned ndq_eye_check_all(unsigned seed)
{
    volatile unsigned *memory = (volatile unsigned *)(MAIN_RAM_BASE_VA + NDQ_EYE_SCRATCH_OFFSET);
    unsigned errors = 0;
    flush_cpu_dcache_range((void *)memory, MEMTEST_DATA_SIZE);
    flush_l2_cache();
    invd_cpu_dcache_range((void *)memory, MEMTEST_DATA_SIZE);
    unsigned prbs_state = seed;
    for (unsigned i=0; i<NDQ_EYE_WORDS; ++i) {
        unsigned expected = ndq_eye_pattern_word(NDQ_EYE_PATTERN_PRBS32_SEEDED,
            0, i, seed, &prbs_state);
        if (memory[i] != expected) ++errors;
    }
    return errors;
}

/* Emit detail only when a debug scan cannot program/read a requested tap. */
#if defined(CSR_NATIVE_DFI_ACTIVITY_REFRESH_COMMANDS_ADDR)
static int ndq_eye_tap_fail(unsigned dq, unsigned tap, int tx,
    const char *phase, unsigned current, unsigned refresh_before)
{
    printf("DQWIN_TAP_FAIL,dq=%u,dir=%s,target=%u,phase=%s,selector=%u,observed=%u,"
        "status_valid=%u,allowed=%u,control=0x%x,vtc=%u,ready=%u,vtc_rdy=0x%x,"
        "refresh_before=%u,refresh_now=%u\n",
        dq, tx ? "tx" : "rx", tap, phase, ddrphy_tap_select_read(), current,
        ddrphy_tap_status_valid_read(), ddrphy_tap_allowed_read(),
        sdram_dfii_control_read(), ddrphy_en_vtc_read(), ddrphy_ready_read(),
        ddrphy_vtc_rdy_read(), refresh_before,
        native_dfi_activity_refresh_commands_read());
    return 0;
}
#endif

static void ndq_eye_abort(unsigned dq, int tx, const char *phase,
    unsigned tap, unsigned observed)
{
    printf("DQWIN_ABORT,dq=%u,dir=%s,stage=%s,tap=%u,observed=%u\n",
        dq, tx ? "tx" : "rx", phase, tap, observed);
}

static int ndq_eye_set_tap(unsigned dq, unsigned tap, int tx)
{
#if !defined(CSR_NATIVE_DFI_ACTIVITY_REFRESH_COMMANDS_ADDR)
    (void)dq; (void)tap; (void)tx;
    return 0;
#else
    unsigned lane = dq / 8;
    unsigned selector = usnative_dq_taps[dq];
    unsigned refresh_before = 0;
    if (tap > NDQ_EYE_MAX_TAP || lane >= SDRAM_PHY_USNATIVE_LANE_COUNT)
        return ndq_eye_tap_fail(dq, tap, tx, "bounds", 0, refresh_before);
    ddrphy_tap_select_write(selector);
    if (!native_tap_wait())
        return ndq_eye_tap_fail(dq, tap, tx, "initial_status", 0, refresh_before);
    unsigned current = tx ? ddrphy_tap_tx_count_read() : ddrphy_tap_rx_count_read();
    unsigned control_saved = sdram_dfii_control_read();
    unsigned vtc_saved = ddrphy_en_vtc_read();
    if (!(control_saved & DFII_CONTROL_SEL) || !vtc_saved)
        return ndq_eye_tap_fail(dq, tap, tx, "ownership", current, refresh_before);
    if (ndq_eye_tap_close(current, tap)) return 1;
    unsigned reset_pending = tap < current;
    while (current != tap || reset_pending) {
        unsigned refresh_snapshot_valid = 0;
        int chunk_ok = 1;
        const char *chunk_failure = "chunk_status";
        sdram_dfii_control_write(DFII_CONTROL_CKE|DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
        ddrphy_en_vtc_write(0);
        ddrphy_dly_sel_write(1u << lane);
        ddrphy_tap_select_write(selector);
        if (!native_tap_wait() || !ddrphy_tap_allowed_read()) {
            current = tx ? ddrphy_tap_tx_count_read() : ddrphy_tap_rx_count_read();
            if (!ddrphy_tap_status_valid_read())
                chunk_failure = "status_invalid";
            else if (!ddrphy_tap_allowed_read())
                chunk_failure = "tap_not_allowed";
            else
                chunk_failure = "status_timeout";
            chunk_ok = 0;
        } else {
            /* Snapshot after the PHY confirms software ownership, so a refresh
             * during the mux transition cannot satisfy the post-chunk wait. */
            refresh_before = native_dfi_activity_refresh_commands_read();
            refresh_snapshot_valid = 1;
            /* Disabling VTC can change CNTVALUEOUT. Plan this chunk from the
             * fresh manual-mode count, not the earlier VTC-enabled sample. */
            current = tx ? ddrphy_tap_tx_count_read() : ddrphy_tap_rx_count_read();
            reset_pending = tap < current;
            unsigned chunk = reset_pending ? tap : tap - current;
            if (chunk > NDQ_EYE_TAP_CHUNK) chunk = NDQ_EYE_TAP_CHUNK;
            if (reset_pending) {
                if (tx) ddrphy_tap_tx_rst_write(1); else ddrphy_tap_rx_rst_write(1);
                if (!native_tap_wait()) {
                    chunk_ok = 0;
                    chunk_failure = "reset_status";
                } else {
                    current = tx ? ddrphy_tap_tx_count_read() : ddrphy_tap_rx_count_read();
                    if (current != 0) {
                        chunk_ok = 0;
                        chunk_failure = "reset_mismatch";
                    } else {
                        reset_pending = 0;
                    }
                }
            }
            for (unsigned i=0; chunk_ok && i<chunk; ++i) {
                unsigned previous = current;
                if (tx) ddrphy_tap_tx_inc_write(1); else ddrphy_tap_rx_inc_write(1);
                if (!native_tap_wait()) {
                    chunk_ok = 0;
                    chunk_failure = "increment_status";
                    break;
                }
                current = tx ? ddrphy_tap_tx_count_read() : ddrphy_tap_rx_count_read();
                if (current != previous + 1) {
                    chunk_ok = 0;
                    chunk_failure = "increment_mismatch";
                }
            }
        }
        /* Return the controller and VTC before waiting for the next refresh. */
        sdram_dfii_control_write(control_saved);
        ddrphy_en_vtc_write(vtc_saved);
        cdelay(10000);
        if (!ndq_eye_wait_vtc())
            return ndq_eye_tap_fail(dq, tap, tx, "vtc_restore", current, refresh_before);
        if (refresh_snapshot_valid) {
            unsigned refreshed = 0;
            for (unsigned timeout=0; timeout<100000; ++timeout) {
                if (native_dfi_activity_refresh_commands_read() != refresh_before) {
                    refreshed = 1;
                    break;
                }
                cdelay(100);
            }
            if (!refreshed)
                return ndq_eye_tap_fail(dq, tap, tx, "refresh_timeout", current, refresh_before);
        }
        if (!chunk_ok)
            return ndq_eye_tap_fail(dq, tap, tx, chunk_failure, current, refresh_before);
        ddrphy_tap_select_write(selector);
        if (!native_tap_wait())
            return ndq_eye_tap_fail(dq, tap, tx, "readback_status", current, refresh_before);
        current = tx ? ddrphy_tap_tx_count_read() : ddrphy_tap_rx_count_read();
        /* With VTC enabled, CNTVALUEOUT may move one count after an exact
         * manual update. Score the settled observed count, not the request. */
        if (ndq_eye_tap_close(current, tap)) return 1;
        if (current > tap)
            return ndq_eye_tap_fail(dq, tap, tx, "readback_overshoot", current, refresh_before);
        if (current < tap) reset_pending = 0;
    }
    if (!ndq_eye_tap_close(current, tap))
        return ndq_eye_tap_fail(dq, tap, tx, "readback_mismatch", current, refresh_before);
    return 1;
#endif
}

/* CNTVALUEOUT can move after the first VTC-enabled readback. Confirm the
 * settled value at the scan boundary and retry a bounded number of times. */
static int ndq_eye_set_tap_stable(unsigned dq, unsigned tap, int tx,
    unsigned *observed)
{
    for (unsigned attempt=0; attempt<3; ++attempt) {
        if (!ndq_eye_set_tap(dq, tap, tx)) return 0;
        ddrphy_tap_select_write(usnative_dq_taps[dq]);
        if (!native_tap_wait()) return 0;
        *observed = tx ? ddrphy_tap_tx_count_read() : ddrphy_tap_rx_count_read();
        if (ndq_eye_tap_close(*observed, tap)) return 1;
        printf("DQWIN_TAP_RETRY,dq=%u,dir=%s,target=%u,observed=%u,attempt=%u\n",
            dq, tx ? "tx" : "rx", tap, *observed, attempt + 1);
    }
    return 0;
}

/* TX mode writes while the candidate is selected, drains cache writes, then
 * restores baseline TX before reading the saved contents through fixed RX. */

static int ndq_eye_one(unsigned dq, int tx, unsigned first, unsigned last,
    unsigned step, unsigned repeats, enum ndq_eye_pattern pattern)
{
    unsigned selector_saved = ddrphy_tap_select_read();
    unsigned dly_saved = ddrphy_dly_sel_read();
    unsigned vtc_saved = ddrphy_en_vtc_read();
    unsigned baseline = 0;
    unsigned best_first = 0, best_last = 0, points = 0, islands = 0;
    unsigned run = 0, run_first = 0, best_width = 0;
    unsigned first_pass = 0, last_pass = 0;
    int failed = 0, restore_failed = 0;
    ddrphy_tap_select_write(usnative_dq_taps[dq]);
    if (!native_tap_wait()) {
        ndq_eye_abort(dq, tx, "baseline_status", 0, 0);
        failed = 1; goto restore_selection;
    }
    baseline = tx ? ddrphy_tap_tx_count_read() : ddrphy_tap_rx_count_read();
    if (baseline > NDQ_EYE_MAX_TAP) {
        ndq_eye_abort(dq, tx, "baseline_bounds", baseline, baseline);
        failed = 1; goto restore_selection;
    }

    /* Establish that the selected bit's receiver and opposite direction can
     * pass at the reference setting before scoring candidate taps. */
    unsigned observed = 0;
    if (!ndq_eye_set_tap_stable(dq, baseline, tx, &observed)) {
        ndq_eye_abort(dq, tx, "baseline_set", baseline, 0);
        failed = 1; goto restore_selection;
    }
    ndq_eye_write_pattern(NDQ_EYE_PATTERN_PRBS32_SEEDED, dq, 1);
    unsigned guard_other = 0;
    unsigned guard_errors = ndq_eye_check(dq, NDQ_EYE_PATTERN_PRBS32_SEEDED, 1, &guard_other);
    printf("DQWIN_GUARD,dir=%s,dq=%u,baseline=%u,errors=%u,other_errors=%u,status=%s\n",
        tx ? "tx" : "rx", dq, baseline, guard_errors, guard_other,
        ndq_eye_guard_pass(guard_errors, guard_other) ? "pass" : "fail");
    if (!ndq_eye_guard_pass(guard_errors, guard_other)) {
        failed = 1; goto restore_selection;
    }

    for (unsigned tap=first; tap<=last; tap+=step) {
        unsigned errors = 0, other_errors = 0;
        unsigned candidate_observed;
        if (!ndq_eye_set_tap_stable(dq, tap, tx, &candidate_observed)) {
            ndq_eye_abort(dq, tx, "candidate_set", tap, 0);
            failed = 1; break;
        }
        for (unsigned repeat=1; repeat<=repeats; ++repeat) {
            unsigned seed = ndq_eye_seed(tap, repeat);
            if (tx) {
                ndq_eye_write_pattern(pattern, dq, seed);
                if (!ndq_eye_set_tap_stable(dq, baseline, 1, &observed)) {
                    ndq_eye_abort(dq, tx, "repeat_tx_restore", baseline, 0);
                    failed = 1; break;
                }
            } else {
                if (!ndq_eye_set_tap_stable(dq, baseline, 0, &observed)) {
                    ndq_eye_abort(dq, tx, "repeat_rx_restore", baseline, 0);
                    failed = 1; break;
                }
                ndq_eye_write_pattern(pattern, dq, seed);
                if (!ndq_eye_set_tap_stable(dq, tap, 0, &observed)) {
                    ndq_eye_abort(dq, tx, "repeat_rx_candidate", tap, 0);
                    failed = 1; break;
                }
            }
            unsigned other = 0;
            errors += ndq_eye_check(dq, pattern, seed, &other);
            other_errors += other;
            if (tx && repeat != repeats &&
                !ndq_eye_set_tap_stable(dq, tap, 1, &observed)) {
                ndq_eye_abort(dq, tx, "repeat_tx_candidate", tap, 0);
                failed = 1; break;
            }
        }
        if (failed) break;
        unsigned pass = errors == 0;
        if (tap == first) first_pass = pass;
        if (tap == last || last-tap < step) last_pass = pass;
        printf("DQWIN,dir=%s,dq=%u,baseline=%u,requested=%u,observed=%u,pass=%u,errors=%u,other_errors=%u,samples=%u,pattern=%s,repeat=%u,step=%u,clipped_low=%u,clipped_high=%u\n",
            tx ? "tx" : "rx", dq, baseline, tap, candidate_observed, pass,
            errors, other_errors, NDQ_EYE_WORDS * repeats, ndq_eye_pattern_name(pattern), repeats,
            step, tap == first && pass, (tap == last || last-tap < step) && pass);
        ++points;
        if (pass) {
            if (!run) { ++islands; run_first = tap; }
            ++run;
        } else {
            if (run > best_width) { best_width = run; best_first = run_first; best_last = tap-step; }
            run = 0;
        }
        if (last - tap < step) break; /* avoid unsigned wrap */
    }
    if (run > best_width) {
        best_width = run;
        best_first = run_first;
        best_last = run_first + (run - 1) * step;
    }

    if (!ndq_eye_set_tap_stable(dq, baseline, tx, &observed)) {
        failed = 1; restore_failed = 1;
    }
restore_selection:
    ddrphy_tap_select_write(selector_saved); cdelay(100);
    ddrphy_dly_sel_write(dly_saved);
    ddrphy_en_vtc_write(vtc_saved);
    if (!ndq_eye_wait_vtc()) { failed = 1; restore_failed = 1; }
    printf("DQWIN_SUMMARY,dir=%s,dq=%u,baseline=%u,first=%u,last=%u,points=%u,islands=%u,step=%u,coarse=%u,clipped_low=%u,clipped_high=%u,status=%s\n",
        tx ? "tx" : "rx", dq, baseline, best_first, best_last, points, islands,
        step, step > 1, first_pass, last_pass,
        restore_failed ? "restore_fail" : (failed ? "scan_fail" : "complete"));
    return !failed;
}

static int ndq_eye_snapshot(unsigned rx[NDQ_EYE_DQ_COUNT], unsigned tx[NDQ_EYE_DQ_COUNT])
{
    unsigned selector = ddrphy_tap_select_read();
    for (unsigned dq=0; dq<NDQ_EYE_DQ_COUNT; ++dq) {
        ddrphy_tap_select_write(usnative_dq_taps[dq]);
        if (!native_tap_wait()) {
            ddrphy_tap_select_write(selector);
            return 0;
        }
        rx[dq] = ddrphy_tap_rx_count_read();
        tx[dq] = ddrphy_tap_tx_count_read();
        if (rx[dq] > NDQ_EYE_MAX_TAP || tx[dq] > NDQ_EYE_MAX_TAP) {
            ddrphy_tap_select_write(selector);
            return 0;
        }
    }
    ddrphy_tap_select_write(selector);
    return native_tap_wait();
}

static int ndq_eye_restore(unsigned rx[NDQ_EYE_DQ_COUNT], unsigned tx[NDQ_EYE_DQ_COUNT],
    unsigned selector, unsigned dly, unsigned vtc)
{
    int ok = 1;
    for (unsigned dq=0; dq<NDQ_EYE_DQ_COUNT; ++dq) {
        ddrphy_tap_select_write(usnative_dq_taps[dq]);
        if (!native_tap_wait()) { ok = 0; continue; }
        unsigned observed = ddrphy_tap_rx_count_read();
        if (!ndq_eye_tap_close(observed, rx[dq]) &&
            !ndq_eye_set_tap_stable(dq, rx[dq], 0, &observed)) ok = 0;
        observed = ddrphy_tap_tx_count_read();
        if (!ndq_eye_tap_close(observed, tx[dq]) &&
            !ndq_eye_set_tap_stable(dq, tx[dq], 1, &observed)) ok = 0;
    }
    ddrphy_tap_select_write(selector); cdelay(100);
    ddrphy_dly_sel_write(dly);
    ddrphy_en_vtc_write(vtc);
    if (vtc && !ndq_eye_wait_vtc()) ok = 0;
    for (unsigned dq=0; dq<NDQ_EYE_DQ_COUNT; ++dq) {
        ddrphy_tap_select_write(usnative_dq_taps[dq]);
        if (!native_tap_wait()) { ok = 0; continue; }
        unsigned rx_observed = ddrphy_tap_rx_count_read();
        unsigned tx_observed = ddrphy_tap_tx_count_read();
        if (!ndq_eye_tap_close(rx_observed, rx[dq]) ||
            !ndq_eye_tap_close(tx_observed, tx[dq])) {
            printf("DQWIN_RESTORE_TAP_FAIL,dq=%u,rx_target=%u,rx_observed=%u,tx_target=%u,tx_observed=%u\n",
                dq, rx[dq], rx_observed, tx[dq], tx_observed);
            ok = 0;
        }
    }
    ddrphy_tap_select_write(selector);
    if (!native_tap_wait()) ok = 0;
    return ok;
}

static void usnative_eye_handler(int nb_params, char **params)
{
#if !defined(CSR_NATIVE_DFI_ACTIVITY_REFRESH_COMMANDS_ADDR)
    (void)nb_params; (void)params;
    printf("Error: usnative_eye requires native DFI refresh monitoring\n");
    return;
#else
    char *end;
    unsigned first, last, step, repeats, dq = 0;
    int tx, all;
    enum ndq_eye_pattern pattern;
    if (nb_params != 7 || (strcmp(params[0], "rx") && strcmp(params[0], "tx"))) {
        printf("usnative_eye rx|tx <dq|all> <first> <last> <step> <prbs32_seeded|complementary|walking_bit|aggressor> <repeats>\n");
        return;
    }
    tx = !strcmp(params[0], "tx");
    all = !strcmp(params[1], "all");
    if (!all) { dq = strtoul(params[1], &end, 0); if (*end || dq >= SDRAM_PHY_DATABITS) { printf("Error: invalid DQ\n"); return; } }
    first = strtoul(params[2], &end, 0); if (*end) { printf("Error: invalid first tap\n"); return; }
    last = strtoul(params[3], &end, 0); if (*end) { printf("Error: invalid last tap\n"); return; }
    step = strtoul(params[4], &end, 0); if (*end) { printf("Error: invalid step\n"); return; }
    if (!strcmp(params[5], "prbs32_seeded")) pattern = NDQ_EYE_PATTERN_PRBS32_SEEDED;
    else if (!strcmp(params[5], "complementary")) pattern = NDQ_EYE_PATTERN_COMPLEMENTARY;
    else if (!strcmp(params[5], "walking_bit")) pattern = NDQ_EYE_PATTERN_WALKING_BIT;
    else if (!strcmp(params[5], "aggressor")) pattern = NDQ_EYE_PATTERN_AGGRESSOR;
    else { printf("Error: unsupported eye pattern\n"); return; }
    repeats = strtoul(params[6], &end, 0); if (*end) { printf("Error: invalid repeats\n"); return; }
    if ((SDRAM_PHY_DATABITS != 16 && SDRAM_PHY_DATABITS != 32 && SDRAM_PHY_DATABITS != 64) ||
        SDRAM_PHY_USNATIVE_DQ_TAPS_COUNT != SDRAM_PHY_DATABITS ||
        first > last || last > NDQ_EYE_MAX_TAP || !step ||
        (last-first)/step+1 > NDQ_EYE_MAX_POINTS || !repeats || repeats > NDQ_EYE_MAX_REPEATS) {
        printf("Error: unsupported geometry or scan bounds\n"); return;
    }
    if (MAIN_RAM_SIZE < NDQ_EYE_SCRATCH_OFFSET + MEMTEST_DATA_SIZE ||
        !ddrphy_ready_read() || ddrphy_vtc_rdy_read() != USNATIVE_CONTROL_MASK ||
        (sdram_dfii_control_read() & DFII_CONTROL_SEL) == 0) {
        printf("Error: PHY not ready, controller not owned, or scratch range unavailable\n"); return;
    }
#ifdef CSR_DMA_BENCH_BUSY_ADDR
    if (dma_bench_busy_read() || dma_bench_fault_read() >= 3u) {
        printf("DQWIN_ABORT,reason=dma_busy_or_fatal; reconfigure FPGA before DDR diagnostics\n");
        return;
    }
#endif
#if defined(CSR_DDRPHY_SCHEDULED_FIFO_MODE_ADDR)
    if (ddrphy_scheduled_fifo_mode_read()) {
        printf("DQWIN_ABORT,reason=scheduled_fifo_mode_active; run full sdram_init before DDR diagnostics\n");
        return;
    }
#endif
    unsigned fifo_empty_words[NDQ_EYE_FIFO_WORDS];
    int fifo_state = ndq_eye_wait_dq_fifo_empty(fifo_empty_words);
    if (fifo_state != 1) {
        ndq_eye_print_fifo_empty("entry", fifo_empty_words);
        printf("DQWIN_FIFO_EMPTY_CHECK,status=%s,participants=dq\n",
            fifo_state == NDQ_EYE_FIFO_UNAVAILABLE ? "unavailable" : "not_empty");
        printf("DQWIN_ABORT,reason=%s; no taps changed\n",
            fifo_state == NDQ_EYE_FIFO_UNAVAILABLE ? "rx_fifo_status_unavailable" : "rx_fifo_not_empty");
        return;
    }
    ndq_eye_print_fifo_empty("entry", fifo_empty_words);
    printf("DQWIN_FIFO_EMPTY_CHECK,status=pass,participants=dq\n");
#ifdef CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION
    unsigned was_ready = dma_bench_software_ready_read();
    dma_bench_software_ready_write(0);
#else
    unsigned was_ready = 0;
#endif
    sdram_invalidate_dma();
    unsigned rx_baseline[NDQ_EYE_DQ_COUNT], tx_baseline[NDQ_EYE_DQ_COUNT];
    unsigned selector_saved = ddrphy_tap_select_read();
    unsigned dly_saved = ddrphy_dly_sel_read();
    unsigned vtc_saved = ddrphy_en_vtc_read();
    if (!ndq_eye_snapshot(rx_baseline, tx_baseline)) {
        printf("DQWIN_ADMISSION,status=closed,reason=snapshot_fail,reenable=full_sdram_init_and_memtest\n");
        printf("DQWIN_DONE,status=snapshot_fail\n");
        return;
    }
    printf("DQWIN_BEGIN,dir=%s,dq=%s,first=%u,last=%u,step=%u,repeats=%u,pattern=%s,scratch=%08x,size=%u,vtc_tap_tolerance=%u\n",
        tx ? "tx" : "rx", all ? "all" : params[1], first, last, step, repeats,
        ndq_eye_pattern_name(pattern),
        MAIN_RAM_BASE_VA + NDQ_EYE_SCRATCH_OFFSET, MEMTEST_DATA_SIZE,
        NDQ_EYE_VTC_TOLERANCE);
    int scan_ok = 1;
    for (unsigned bit=all ? 0 : dq; bit<(all ? SDRAM_PHY_DATABITS : dq+1); ++bit)
        if (!ndq_eye_one(bit, tx, first, last, step, repeats, pattern)) { scan_ok = 0; break; }

    /* Restoration is evaluated independently from scan success. A tap-step
     * failure must not skip the final whole-word memory check. */
    int taps_restored = ndq_eye_restore(rx_baseline, tx_baseline,
        selector_saved, dly_saved, vtc_saved);
    printf("DQWIN_RESTORE_TAPS,status=%s\n", taps_restored ? "pass" : "fail");
    unsigned fifo_quiet = 0, restore_errors = 0, data_checked = 0;
    if (taps_restored) {
        fifo_state = ndq_eye_wait_dq_fifo_empty(fifo_empty_words);
        fifo_quiet = fifo_state == 1;
        ndq_eye_print_fifo_empty("restore", fifo_empty_words);
        printf("DQWIN_FIFO_EMPTY_CHECK,status=%s,where=restore,participants=dq\n",
            fifo_state == 1 ? "pass" :
            (fifo_state == NDQ_EYE_FIFO_UNAVAILABLE ? "unavailable" : "not_empty"));
        if (fifo_quiet) {
            /* Validate a known whole-word pattern at restored taps. */
            ndq_eye_write_pattern(NDQ_EYE_PATTERN_PRBS32_SEEDED, 0, 1);
            restore_errors = ndq_eye_check_all(1);
            data_checked = 1;
        }
    }
    int restore_ok = taps_restored && fifo_quiet && data_checked && !restore_errors;
    if (restore_ok) {
        printf("DQWIN_RESTORE,status=pass,errors=0\n");
    } else {
        const char *reason = !taps_restored ? "tap_restore" :
            (fifo_state == NDQ_EYE_FIFO_UNAVAILABLE ? "rx_fifo_status_unavailable" :
            (!fifo_quiet ? "rx_fifo_not_empty" :
            (!data_checked ? "data_check_skipped" : "whole_word_mismatch")));
        printf("DQWIN_RESTORE,status=fail,reason=%s,errors=%u,data_checked=%u\n",
            reason, restore_errors, data_checked);
        printf("DQWIN_RECOVERY_REQUIRED,action=sdram_init,admission=closed,original_failure=sticky\n");
    }
    if (!scan_ok && restore_ok)
        printf("DQWIN_ADMISSION,status=closed,reason=scan_failed,reenable=full_sdram_init_and_memtest\n");
#ifdef CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION
    if (scan_ok && restore_ok && was_ready) dma_bench_software_ready_write(1);
#endif
    int ok = scan_ok && restore_ok;
    printf("DQWIN_DONE,status=%s,scan=%s,restore=%s\n", ok ? "pass" : "fail",
        scan_ok ? "pass" : "fail", restore_ok ? "pass" : "fail");
#endif
}

#endif /* USNATIVE_DQ_EYE_H */
