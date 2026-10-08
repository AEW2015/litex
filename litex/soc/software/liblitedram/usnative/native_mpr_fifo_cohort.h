/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Diagnostic-only cohort sweep for the registered native FIFO-pop path.
 * Include after native_mpr_probe.h and the DFII helpers.  This changes only
 * the software-selected scheduled-pop mode and its delay; it does not adjust
 * taps, gates, VTC, or reset.  A scheduled pop is not a promise that RXTX Q is
 * valid. */
#ifndef USNATIVE_NATIVE_MPR_FIFO_COHORT_H
#define USNATIVE_NATIVE_MPR_FIFO_COHORT_H

#ifdef CONFIG_SDRAM_USNATIVE_MPR_FIFO_COHORT

#if !defined(CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC)
#error "The scheduled FIFO cohort requires the native MPR diagnostic helpers"
#endif
#if !defined(CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR)
#error "The scheduled FIFO cohort requires per-lane DQS write-clock counters"
#endif
#if !defined(CSR_DDRPHY_SCHEDULED_FIFO_MODE_ADDR) || \
    !defined(CSR_DDRPHY_SCHEDULED_FIFO_DELAY_ADDR) || \
    !defined(CSR_DDRPHY_SCHEDULED_FIFO_UNDERFLOWS_ADDR) || \
    !defined(CSR_DDRPHY_SCHEDULED_FIFO_OVERLAPS_ADDR) || \
    !defined(CSR_DDRPHY_SCHEDULED_FIFO_MISSING_LAST_ADDR)
#error "The scheduled FIFO cohort requires the scheduled-pop diagnostic CSRs"
#endif

#if !defined(SDRAM_PHY_DDR4) || SDRAM_PHY_DATABITS != 32 || \
    SDRAM_PHY_DFI_DATABITS != 64 || SDRAM_PHY_PHASES != 4 || \
    CONFIG_CLOCK_FREQUENCY != 200000000 || SDRAM_PHY_CL != 11 || \
    SDRAM_PHY_CWL != 9
#error "The MPR FIFO cohort is restricted to AES-KU40 native x32 DDR4-1600 1:4"
#endif

#include <stdint.h>

struct nb_mpr_fifo_cohort_sample {
    unsigned edge_mask;
    unsigned pop_mask;
    unsigned valid_delta;
    unsigned exact_fresh_mask;
    unsigned underflow_delta;
    unsigned overlap_delta;
    unsigned missing_last;
};

static unsigned nb_mpr_fifo_cohort_fifo_reads(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_fifo_reads0_read();
    case 1: return ddrphy_fifo_reads1_read();
    case 2: return ddrphy_fifo_reads2_read();
    default: return ddrphy_fifo_reads3_read();
    }
}

static unsigned nb_mpr_fifo_cohort_dqs_edges(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_dqs_wrclk_edges0_read();
    case 1: return ddrphy_dqs_wrclk_edges1_read();
    case 2: return ddrphy_dqs_wrclk_edges2_read();
    default: return ddrphy_dqs_wrclk_edges3_read();
    }
}

static unsigned nb_mpr_fifo_cohort_valid_count(void)
{
    return native_dfi_activity_rddata_valid_cycles_read();
}

static void nb_mpr_fifo_cohort_read(uint64_t words[4])
{
    /* Identical page-0/MPR0 READ on every warmup and measured sample. */
    sdram_dfii_pi0_address_write(0);
    sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
    cdelay(100);
    words[0]=sdram_dfii_pi0_rddata_read();
    words[1]=sdram_dfii_pi1_rddata_read();
    words[2]=sdram_dfii_pi2_rddata_read();
    words[3]=sdram_dfii_pi3_rddata_read();
}

static void nb_mpr_fifo_cohort(void)
{
    static const unsigned first_delay=4u;
    static const unsigned last_delay=16u;
    static const unsigned warmup_reads=4u;
    static const unsigned measured_reads=16u;
    /* Seven unsigned fields per row: 448 bytes total on the target ABI. */
    struct nb_mpr_fifo_cohort_sample samples[16];
    const unsigned saved_mode=ddrphy_scheduled_fifo_mode_read();
    const unsigned saved_delay=ddrphy_scheduled_fifo_delay_read();

    printf("DDR4 MPR scheduled FIFO cohort: delay=4..16, warmup=4 unqualified, measured=16; request-to-RD_EN launch latency=delay+1 PHY clocks\n");
    ddrphy_scheduled_fifo_mode_write(1);
    cdelay(100);

    /* Caller already owns DFII and enabled MR3 MPR mode. Do not issue
     * another mode-register command or change DRAM state in this probe. */

    for (unsigned delay=first_delay; delay<=last_delay; ++delay) {
        ddrphy_scheduled_fifo_delay_write(delay);
        cdelay(100);

        /* Keep warmup distinct from scoring so startup/stale FIFO contents
         * cannot qualify a cohort. No UART traffic occurs during these reads. */
        for (unsigned read=0; read<warmup_reads; ++read) {
            uint64_t words[4];
            nb_mpr_fifo_cohort_read(words);
            cdelay(100);
        }

        for (unsigned read=0; read<measured_reads; ++read) {
            uint64_t words[4];
            unsigned edge_before[4],fifo_before[4];
            unsigned valid_before,underflow_before,overlap_before;
            struct nb_mpr_fifo_cohort_sample *sample=&samples[read];

            for (unsigned lane=0; lane<4; ++lane) {
                edge_before[lane]=nb_mpr_fifo_cohort_dqs_edges(lane);
                fifo_before[lane]=nb_mpr_fifo_cohort_fifo_reads(lane);
            }
            valid_before=nb_mpr_fifo_cohort_valid_count();
            underflow_before=ddrphy_scheduled_fifo_underflows_read();
            overlap_before=ddrphy_scheduled_fifo_overlaps_read();
            nb_mpr_fifo_cohort_read(words);
            cdelay(100);

            sample->edge_mask=0;
            sample->pop_mask=0;
            sample->exact_fresh_mask=0;
            for (unsigned lane=0; lane<4; ++lane) {
                const unsigned edge_delta=
                    nb_mpr_fifo_cohort_dqs_edges(lane)-edge_before[lane];
                const unsigned fifo_delta=
                    nb_mpr_fifo_cohort_fifo_reads(lane)-fifo_before[lane];
                const unsigned exact=(unsigned)nb_mpr_lane_word_matches(words,lane);
                sample->edge_mask |= (edge_delta==1u)<<lane;
                sample->pop_mask |= (fifo_delta==1u)<<lane;
                sample->exact_fresh_mask |=
                    (exact && fifo_delta==1u && edge_delta==1u)<<lane;
            }
            sample->valid_delta=nb_mpr_fifo_cohort_valid_count()-valid_before;
            sample->underflow_delta=
                ddrphy_scheduled_fifo_underflows_read()-underflow_before;
            sample->overlap_delta=
                ddrphy_scheduled_fifo_overlaps_read()-overlap_before;
            sample->missing_last=ddrphy_scheduled_fifo_missing_last_read() & 0xfu;
        }

        /* UART reporting is deferred until all 16 identical commands finish. */
        for (unsigned read=0; read<measured_reads; ++read) {
            const struct nb_mpr_fifo_cohort_sample *sample=&samples[read];
            printf("DDR4 MPR scheduled FIFO: delay=%u launch_latency=%u read=%u edge=%x pop=%x valid=%u exact_fresh=%x underflow_delta=%u overlap_delta=%u missing=%x\n",
                delay,delay+1u,read,sample->edge_mask,sample->pop_mask,
                sample->valid_delta,sample->exact_fresh_mask,
                sample->underflow_delta,sample->overlap_delta,
                sample->missing_last);
        }
    }

    ddrphy_scheduled_fifo_delay_write(saved_delay);
    ddrphy_scheduled_fifo_mode_write(saved_mode);
    cdelay(100);
    printf("DDR4 MPR scheduled FIFO cohort complete: restored mode=%u delay=%u; taps/gates/VTC/reset untouched\n",
        saved_mode,saved_delay);
}

#endif /* CONFIG_SDRAM_USNATIVE_MPR_FIFO_COHORT */
#endif /* USNATIVE_NATIVE_MPR_FIFO_COHORT_H */
