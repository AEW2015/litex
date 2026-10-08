/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef USNATIVE_DATA_MAP_H
#define USNATIVE_DATA_MAP_H

#include <stdint.h>

#define USNATIVE_BL8_CPU_WORDS (SDRAM_PHY_DATABITS / 4u)

/* Map a physical DQ bit to its bit position in a 32-bit CPU word. The x64
 * controller interface presents two adjacent CPU words for each 64-bit
 * channel beat: the low-address word carries DQ[31:0], then DQ[63:32]. */
static inline uint32_t usnative_cpu_word_dq_mask(unsigned dq,
    unsigned cpu_word_index)
{
#if SDRAM_PHY_DATABITS == 16
    return (1u << dq) | (1u << (dq + 16));
#elif SDRAM_PHY_DATABITS == 32
    (void)cpu_word_index;
    return 1u << dq;
#elif SDRAM_PHY_DATABITS == 64
    if ((dq >= 64u) || ((cpu_word_index & 1u) != (dq >> 5))) return 0;
    return 1u << (dq & 31u);
#else
#error "USNative CPU DQ mapping supports x16, x32, and x64 PHYs"
#endif
}

#endif /* USNATIVE_DATA_MAP_H */
