/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef __BIOS_NATIVE_DMA_ADMISSION_H
#define __BIOS_NATIVE_DMA_ADMISSION_H

/* Keep the benchmark command independent of PHY-specific CSR layouts.  The
 * USNative PHY exposes hardware training state.  Component PHY builds instead
 * use a software-owned bit that sdram_init grants after standard leveling and
 * a successful final controller-path memory test. */
static inline int native_dma_admission_ready(void)
{
#ifdef CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION
	return dma_bench_software_ready_read();
#elif defined(CONFIG_SDRAM_USNATIVE_XEM8320)
	return ddrphy_ready_read() && ddrphy_training_stage_read() == 5 &&
		!ddrphy_training_error_read() && !ddrphy_bisc_only_read();
#else
#error "Native DMA requires a supported SDRAM readiness contract"
#endif
}

#endif /* __BIOS_NATIVE_DMA_ADMISSION_H */
