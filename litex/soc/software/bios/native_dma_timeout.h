/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef __BIOS_NATIVE_DMA_TIMEOUT_H
#define __BIOS_NATIVE_DMA_TIMEOUT_H

#include <limits.h>
#include <stdint.h>

#define NATIVE_DMA_HW_TIMEOUT_CYCLES 1000000000u
#define NATIVE_DMA_SETTLE_US         1000000u

/* Keep the current two-phase allowance. Repeated-read mode adds one bounded
 * read phase and its requested idle dwell; the hardware watchdog does not
 * cover that dwell. The one-second settle budget covers CSR/reporting latency. */
static inline unsigned int native_dma_completion_timeout_us(unsigned int repeat_verify,
	unsigned int repeat_idle_cycles)
{
#if CONFIG_CLOCK_FREQUENCY == 0
	/* A generated SoC must have a real clock; saturate safely if misconfigured. */
	return UINT_MAX;
#else
	uint64_t cycles = 2ull * NATIVE_DMA_HW_TIMEOUT_CYCLES;
	uint64_t timeout_us;
	if (repeat_verify)
		cycles += NATIVE_DMA_HW_TIMEOUT_CYCLES;
	if (repeat_verify)
		cycles += repeat_idle_cycles;
	/* Divide/remainder avoids overflow in the round-up addition. */
	timeout_us = (cycles / CONFIG_CLOCK_FREQUENCY) * 1000000ull;
	uint64_t remainder = cycles % CONFIG_CLOCK_FREQUENCY;
	if (remainder)
		timeout_us += (remainder * 1000000ull + CONFIG_CLOCK_FREQUENCY - 1u) /
			CONFIG_CLOCK_FREQUENCY;
	timeout_us += NATIVE_DMA_SETTLE_US;
	if (timeout_us > UINT_MAX)
		return UINT_MAX;
	return (unsigned int)timeout_us;
#endif
}

#endif /* __BIOS_NATIVE_DMA_TIMEOUT_H */
