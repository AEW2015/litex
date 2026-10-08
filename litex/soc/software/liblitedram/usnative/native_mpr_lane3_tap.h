/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */
#ifndef USNATIVE_NATIVE_MPR_LANE3_TAP_H
#define USNATIVE_NATIVE_MPR_LANE3_TAP_H

#define USNATIVE_MPR_LANE3_TAP_POINTS 13u
#define USNATIVE_MPR_LANE3_TAP_MAX   96u
#define USNATIVE_MPR_LANE3_TAP_STEP  8u
#define USNATIVE_MPR_LANE3_READS     12u

typedef int (*usnative_mpr_tap_restore_fn)(unsigned dq, unsigned tap, void *context);
typedef int (*usnative_mpr_tap_read_fn)(unsigned dq, unsigned *tap, void *context);

/* Restore every DQ independently, and verify each value after programming. */
static int usnative_mpr_restore_lane_taps(unsigned lane, const unsigned taps[8],
    unsigned tap_count, usnative_mpr_tap_restore_fn restore,
    usnative_mpr_tap_read_fn readback, void *context)
{
    if (lane>=4u || tap_count!=8u || !taps || !restore || !readback) return 0;
    int restored=1;
    for (unsigned bit=0; bit<8u; ++bit) {
        unsigned actual=~0u;
        const unsigned dq=lane*8u+bit;
        if (!restore(dq,taps[bit],context)) {
            restored=0;
            continue;
        }
        if (!readback(dq,&actual,context) || actual!=taps[bit]) restored=0;
    }
    return restored;
}

static int usnative_mpr_lane3_tap_candidate(unsigned epoch_ok,
    unsigned target_fresh, unsigned exact_edgepop_reads,
    unsigned coherent_lane_mask)
{
    return epoch_ok && target_fresh==USNATIVE_MPR_LANE3_READS &&
        exact_edgepop_reads==USNATIVE_MPR_LANE3_READS &&
        (coherent_lane_mask&(1u<<3))!=0u;
}

static int usnative_mpr_lane3_joint_confirmed(unsigned epoch_ok,
    unsigned target_fresh, unsigned joint_fresh, unsigned exact_edgepop_reads,
    unsigned coherent_lane_mask)
{
    return epoch_ok && target_fresh==USNATIVE_MPR_LANE3_READS &&
        joint_fresh==USNATIVE_MPR_LANE3_READS &&
        exact_edgepop_reads==USNATIVE_MPR_LANE3_READS &&
        (coherent_lane_mask&0x0fu)==0x0fu;
}

/* Pick the midpoint of the widest contiguous passing RX-tap run. On ties,
 * prefer the midpoint nearest the average saved per-DQ tap. */
static int usnative_mpr_lane3_choose_tap(const unsigned passing[USNATIVE_MPR_LANE3_TAP_POINTS],
    unsigned saved_average, unsigned *tap, unsigned *first, unsigned *last,
    unsigned *samples)
{
    unsigned best_run=0,best_start=0,best_end=0;
    if (!passing || !tap || !first || !last || !samples) return 0;
    for (unsigned start=0; start<USNATIVE_MPR_LANE3_TAP_POINTS; ++start) {
        if (!passing[start]) continue;
        unsigned end=start;
        while (end+1u<USNATIVE_MPR_LANE3_TAP_POINTS && passing[end+1u]) ++end;
        const unsigned run=end-start+1u;
        const unsigned center=USNATIVE_MPR_LANE3_TAP_STEP*((start+end)/2u);
        const unsigned old_distance=center>saved_average ? center-saved_average : saved_average-center;
        const unsigned best_center=USNATIVE_MPR_LANE3_TAP_STEP*((best_start+best_end)/2u);
        const unsigned best_distance=best_center>saved_average ? best_center-saved_average : saved_average-best_center;
        if (run>best_run || (run==best_run && old_distance<best_distance)) {
            best_run=run; best_start=start; best_end=end;
        }
        start=end;
    }
    if (!best_run) return 0;
    *first=USNATIVE_MPR_LANE3_TAP_STEP*best_start;
    *last=USNATIVE_MPR_LANE3_TAP_STEP*best_end;
    *tap=USNATIVE_MPR_LANE3_TAP_STEP*((best_start+best_end)/2u);
    *samples=best_run;
    return 1;
}

#endif
