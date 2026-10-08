/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Optional fixed-delay / receive-gate-width screen for native DDR4 MPR0.
 * Include after native_mpr_probe.h's MPR helpers. This does not alter CK,
 * RX/TX taps, phase, VTC, or MR3. It records diagnostic returns only. */
#ifndef USNATIVE_NATIVE_MPR_GATE_COHORT_H
#define USNATIVE_NATIVE_MPR_GATE_COHORT_H

#ifdef CONFIG_SDRAM_USNATIVE_MPR_GATE_COHORT

#if !defined(CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP) || \
    !defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TAP_SCAN) || \
    !defined(CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR)
#error "The MPR gate cohort requires lane gate controls, lane sweep and DQS clock monitoring"
#endif

/* Call only while MR3 MPR mode is enabled and with DFII software ownership.
 * Four warm-up READs precede each setting; only the following 16 are scored.
 * Measurements stay buffered until that 16-read cohort ends, keeping UART
 * output out of its cadence. */
static void nb_mpr_gate_cohort(void)
{
    struct gate_cohort_sample {
        unsigned edge_mask, pop_mask, exact_mask, fresh_exact_mask;
        unsigned extra_pop_mask;
        unsigned edge_delta[4], fifo_delta[4];
    };
    static const unsigned widths[]={1,2,3,4,5,6,7,8};
    static const unsigned delays[]={0,1,2,3,4,5,6,7};
    const unsigned saved_override=ddrphy_gate_override_read();
    const unsigned saved_delay[4]={ddrphy_gate_delay0_read(),
        ddrphy_gate_delay1_read(),ddrphy_gate_delay2_read(),
        ddrphy_gate_delay3_read()};
    const unsigned saved_width[4]={ddrphy_gate_width0_read(),
        ddrphy_gate_width1_read(),ddrphy_gate_width2_read(),
        ddrphy_gate_width3_read()};
    struct gate_cohort_sample samples[16];

    ddrphy_gate_override_write(0xfu);
    cdelay(100);
    for (unsigned wi=0; wi<sizeof(widths)/sizeof(widths[0]); ++wi) {
        for (unsigned di=0; di<sizeof(delays)/sizeof(delays[0]); ++di) {
            for (unsigned lane=0; lane<4; ++lane)
                nb_mpr_lane_gate_width(lane,delays[di],widths[wi]);
            cdelay(100);

            for (unsigned warmup=0; warmup<4; ++warmup) {
                uint64_t words[4];
                nb_mpr_lane_sweep_read(words);
            }

            for (unsigned read=0; read<16; ++read) {
                uint64_t words[4];
                unsigned edge_before[4],fifo_before[4];
                for (unsigned lane=0; lane<4; ++lane) {
                    edge_before[lane]=nb_mpr_dqs_wrclk_edges(lane);
                    fifo_before[lane]=nb_mpr_lane_fifo_reads(lane);
                }
                nb_mpr_lane_sweep_read(words);
                cdelay(100);

                struct gate_cohort_sample *sample=&samples[read];
                sample->edge_mask=sample->pop_mask=sample->exact_mask=0;
                sample->fresh_exact_mask=sample->extra_pop_mask=0;
                for (unsigned lane=0; lane<4; ++lane) {
                    sample->edge_delta[lane]=nb_mpr_dqs_wrclk_edges(lane)-edge_before[lane];
                    sample->fifo_delta[lane]=nb_mpr_lane_fifo_reads(lane)-fifo_before[lane];
                    sample->edge_mask |= (sample->edge_delta[lane]==1u)<<lane;
                    sample->pop_mask |= (sample->fifo_delta[lane]==1u)<<lane;
                    sample->extra_pop_mask |= (sample->fifo_delta[lane]>1u)<<lane;
                    const unsigned exact=(unsigned)nb_mpr_lane_word_matches(words,lane);
                    sample->exact_mask |= exact<<lane;
                    sample->fresh_exact_mask |= (exact &&
                        sample->fifo_delta[lane]==1u &&
                        sample->edge_delta[lane]==1u)<<lane;
                }
            }

            printf("DDR4 MPR gate cohort: width=%u delay=%u warmup=4 measured=16\n",
                widths[wi],delays[di]);
            for (unsigned read=0; read<16; ++read) {
                const struct gate_cohort_sample *sample=&samples[read];
                printf("DDR4 MPR gate cohort row: width=%u delay=%u read=%u edge=%x pop=%x exact=%x fresh_exact=%x extra_pop=%x edge_delta=%u,%u,%u,%u fifo_delta=%u,%u,%u,%u\n",
                    widths[wi],delays[di],read,sample->edge_mask,
                    sample->pop_mask,sample->exact_mask,sample->fresh_exact_mask,
                    sample->extra_pop_mask,
                    sample->edge_delta[0],sample->edge_delta[1],
                    sample->edge_delta[2],sample->edge_delta[3],
                    sample->fifo_delta[0],sample->fifo_delta[1],
                    sample->fifo_delta[2],sample->fifo_delta[3]);
            }
        }
    }

    /* Restore programmed values before restoring the override selector. */
    for (unsigned lane=0; lane<4; ++lane)
        nb_mpr_lane_gate_width(lane,saved_delay[lane],saved_width[lane]);
    cdelay(100);
    ddrphy_gate_override_write(saved_override);
    cdelay(100);
    printf("DDR4 MPR gate cohort complete: restored override=%x delays=%u,%u,%u,%u widths=%u,%u,%u,%u\n",
        saved_override,saved_delay[0],saved_delay[1],saved_delay[2],saved_delay[3],
        saved_width[0],saved_width[1],saved_width[2],saved_width[3]);
}

#endif /* CONFIG_SDRAM_USNATIVE_MPR_GATE_COHORT */
#endif /* USNATIVE_NATIVE_MPR_GATE_COHORT_H */
