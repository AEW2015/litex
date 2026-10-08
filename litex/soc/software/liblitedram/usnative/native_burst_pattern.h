/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef USNATIVE_BURST_PATTERN_H
#define USNATIVE_BURST_PATTERN_H

#include <stdint.h>

/* Build one x32 native DFI phase: two independently patterned 32-bit edges. */
static inline uint64_t usnative_burst_phase_pattern(unsigned phase, unsigned iteration)
{
    static const uint32_t seeds[8] = {
        0x12345678u, 0x89abcdefu, 0xa55a5aa5u, 0x76543210u,
        0xdeadbeefu, 0x0f1e2d3cu, 0x55aa00ffu, 0xf0e1d2c3u
    };
    static const uint32_t steps[8] = {
        0x01020305u, 0x0507090bu, 0x11131517u, 0x1d1f2329u,
        0x2b2f3539u, 0x3b3d4347u, 0x4f535559u, 0x5b616367u
    };
    unsigned first_beat = phase*2;
    uint32_t first = seeds[first_beat] ^ (iteration * steps[first_beat]);
    uint32_t second = seeds[first_beat + 1] ^ (iteration * steps[first_beat + 1]);
    return ((uint64_t)second << 32) | first;
}

/* One x64 physical beat occupies each 64-bit phase on the reduced-width
 * 1:8 profile. Keep independent, changing values on DQ[31:0] and DQ[63:32]. */
static inline uint64_t usnative_x64_phase_pattern(unsigned phase, unsigned iteration)
{
    static const uint32_t seeds[16] = {
        0x12345678u, 0x89abcdefu, 0xa55a5aa5u, 0x76543210u,
        0xdeadbeefu, 0x0f1e2d3cu, 0x55aa00ffu, 0xf0e1d2c3u,
        0x13579bdfu, 0x2468ace0u, 0x55c33c5au, 0xa5691e4bu,
        0xf00f6996u, 0x8731c6e2u, 0x3c5aa5c3u, 0x1e4b78d2u
    };
    static const uint32_t steps[16] = {
        0x01020305u, 0x0507090bu, 0x11131517u, 0x1d1f2329u,
        0x2b2f3539u, 0x3b3d4347u, 0x4f535559u, 0x5b616367u,
        0x07111d2bu, 0x0d172331u, 0x1927333fu, 0x25313d4bu,
        0x2f3b4753u, 0x414d5965u, 0x53616d79u, 0x65717d89u
    };
    unsigned index = phase * 2u;
    uint32_t low  = seeds[index]     ^ (iteration * steps[index]);
    uint32_t high = seeds[index + 1] ^ (iteration * steps[index + 1]);
    return ((uint64_t)high << 32) | low;
}

static inline void usnative_x64_1to8_patterns(uint64_t phase_data[8],
    unsigned iteration)
{
    for (unsigned phase=0; phase<8; ++phase)
        phase_data[phase] = usnative_x64_phase_pattern(phase, iteration);
}

#endif /* USNATIVE_BURST_PATTERN_H */
