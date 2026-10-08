/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Pure helpers for diagnosing byte rotation from changing DDR4 MPR locations.
 * A returned bit r means the captured byte is rotate-right(expected, r).
 * NativeRXBitslip itself also rotates right, so its correction is (8-r)&7.
 * `expected` is a chronological UI byte: DDR4 serial MPR sends register bit 7
 * on UI0, then bit 6, through bit 0 on UI7 (Micron EDY4016A Table 30).
 * This establishes UI rotation only; it does not establish freshness, capture
 * timing, or permission to change production calibration settings. */
#ifndef USNATIVE_NATIVE_MPR_ROTATION_H
#define USNATIVE_NATIVE_MPR_ROTATION_H

#include <stdint.h>

static inline unsigned usnative_mpr_rotation_single_bit(uint8_t mask)
{
    return mask && !(mask & (uint8_t)(mask - 1u));
}

static inline uint8_t usnative_mpr_rotate_right8(uint8_t value,
                                                 unsigned rotation)
{
    rotation &= 7u;
    return (uint8_t)((value >> rotation) | (value << ((8u - rotation) & 7u)));
}

/* Convert the MPR register's [7:0] presentation into UI0..UI7 order. */
static inline uint8_t usnative_mpr_serial_ui_byte(uint8_t register_byte)
{
    uint8_t serial = 0;
    for (unsigned ui = 0; ui < 8u; ++ui)
        serial |= (uint8_t)(((register_byte >> (7u-ui)) & 1u) << ui);
    return serial;
}

/* Return every right-rotation compatible with one expected/captured byte. */
static inline uint8_t usnative_mpr_rotation_mask_for_pattern(uint8_t expected,
                                                              uint8_t captured)
{
    uint8_t mask = 0;
    for (unsigned rotation = 0; rotation < 8u; ++rotation)
        if (usnative_mpr_rotate_right8(expected, rotation) == captured)
            mask |= (uint8_t)(1u << rotation);
    return mask;
}

/* The probe's 12 locations are an ordered freshness signature. Require the
 * complete sequence and intersect rotation masks for every returned byte. */
static inline int usnative_mpr_rotation_cohort_mask(
    const uint8_t captured[12], const uint8_t locations[12], uint8_t *mask_out)
{
    static const uint8_t expected_locations[12] =
        {0,1,2,0,2,1,0,1,2,1,0,2};
    /* Table 29 gives register bytes 55/33/0f; Table 30 sends MSB first. */
    static const uint8_t patterns[3] = {0xaau,0xccu,0xf0u};
    uint8_t mask = 0xffu;
    if (!captured || !locations || !mask_out) return 0;
    for (unsigned read = 0; read < 12u; ++read) {
        if (locations[read] != expected_locations[read]) {
            *mask_out = 0;
            return 0;
        }
        mask &= usnative_mpr_rotation_mask_for_pattern(
            patterns[locations[read]], captured[read]);
    }
    *mask_out = mask;
    return usnative_mpr_rotation_single_bit(mask);
}

/* A byte lane is aligned only when all eight DQs prove the same unique
 * rotation. A zero/multi-bit or differing per-DQ result is rejected. */
static inline int usnative_mpr_rotation_lane_mask(const uint8_t dq_masks[8],
                                                   uint8_t *mask_out)
{
    uint8_t common = 0xffu;
    if (!dq_masks || !mask_out) return 0;
    for (unsigned dq = 0; dq < 8u; ++dq) {
        if (!usnative_mpr_rotation_single_bit(dq_masks[dq])) {
            *mask_out = 0;
            return 0;
        }
        common &= dq_masks[dq];
    }
    *mask_out = common;
    return usnative_mpr_rotation_single_bit(common);
}

#endif /* USNATIVE_NATIVE_MPR_ROTATION_H */
