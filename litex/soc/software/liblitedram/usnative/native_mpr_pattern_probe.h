/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Fresh-return diagnostic for EDY4016A DDR4 page-0 serial MPR patterns.
 * The caller owns DFII and has already enabled MPR mode. Locations 0, 1
 * and 2 contain register bytes 0x55, 0x33 and 0x0f (Table 29); serial UI0..7
 * sends bit 7 first (Table 30), so the chronological UI bytes are aa/cc/f0.
 * The freshness matcher accepts circular rotations of the register byte,
 * which also accepts its bit-reversed serial form. The separate rotation
 * decoder must compare against aa/cc/f0. Rotation diagnoses framing; it does
 * not prove UI alignment. Printed pattern= values remain register bytes.
 */
#ifndef USNATIVE_NATIVE_MPR_PATTERN_PROBE_H
#define USNATIVE_NATIVE_MPR_PATTERN_PROBE_H

#ifdef CONFIG_SDRAM_USNATIVE_MPR_RIU_GATE_COHORT

static int nb_mpr_pattern_lane_matches(const uint64_t words[4],
                                       unsigned lane, unsigned pattern)
{
    for (unsigned dq=8*lane; dq<8*lane+8; ++dq) {
        unsigned captured=0, matched=0;
        for (unsigned phase=0; phase<4; ++phase) {
            captured |= ((unsigned)(words[phase] >> dq) & 1u) << (2*phase);
            captured |= ((unsigned)(words[phase] >> (dq+32)) & 1u) << (2*phase+1);
        }
        for (unsigned rotation=0; rotation<8; ++rotation) {
            unsigned rotated=((pattern << rotation) |
                (pattern >> (8-rotation))) & 0xffu;
            if (captured == rotated) matched=1;
        }
        if (!matched) return 0;
    }
    return 1;
}

struct nb_mpr_pattern_sample {
    unsigned location, edge_mask, pop_mask, pattern_mask, fresh_mask;
    unsigned edges[4], pops[4];
    uint64_t words[4];
};

typedef int (*nb_mpr_pattern_prepare_fn)(void *context,
    unsigned read, unsigned location);

/* No UART traffic occurs inside the twelve-read cohort. Distinct patterns
 * provide a freshness tag even if an old alternating word remains in Q.
 * Write-clock counters are DQS proxies and pop counters count selected DQ
 * tokens; the raw per-bit signature is therefore also mandatory. */
static int nb_mpr_pattern_probe_prepared(unsigned delay, unsigned width,
    unsigned coarse, nb_mpr_pattern_prepare_fn prepare, void *context,
    unsigned *completed_reads)
{
    static const unsigned patterns[3]={0x55u,0x33u,0x0fu};
    static const unsigned locations[12]={0,1,2,0,2,1,0,1,2,1,0,2};
    struct nb_mpr_pattern_sample samples[12];
    unsigned completed=0;
    for (unsigned read=0; read<12; ++read) {
        unsigned before_edges[4], before_pops[4];
        struct nb_mpr_pattern_sample *sample=&samples[read];
        sample->location=locations[read];
        if (prepare && !prepare(context,read,sample->location)) break;
        sample->edge_mask=sample->pop_mask=sample->pattern_mask=sample->fresh_mask=0;
        for (unsigned lane=0; lane<4; ++lane) {
            before_edges[lane]=nb_mpr_dqs_wrclk_edges(lane);
            before_pops[lane]=nb_mpr_lane_fifo_reads(lane);
        }
        nb_mpr_lane_location_read(sample->words,sample->location);
        cdelay(100);
        for (unsigned lane=0; lane<4; ++lane) {
            sample->edges[lane]=nb_mpr_dqs_wrclk_edges(lane)-before_edges[lane];
            sample->pops[lane]=nb_mpr_lane_fifo_reads(lane)-before_pops[lane];
            unsigned pattern_ok=nb_mpr_pattern_lane_matches(sample->words,
                lane,patterns[sample->location]);
            sample->edge_mask |= (sample->edges[lane]==1u) << lane;
            sample->pop_mask |= (sample->pops[lane]==1u) << lane;
            sample->pattern_mask |= pattern_ok << lane;
            sample->fresh_mask |= (pattern_ok && sample->edges[lane]==1u &&
                sample->pops[lane]==1u) << lane;
        }
        completed++;
    }
    if (completed_reads) *completed_reads=completed;
    for (unsigned read=0; read<completed; ++read) {
        const struct nb_mpr_pattern_sample *s=&samples[read];
        printf("DDR4 MPR pattern row: delay=%u width=%u coarse=%u read=%u location=%u pattern=%02x edge=%x pop=%x match=%x fresh=%x edge_delta=%u,%u,%u,%u fifo_delta=%u,%u,%u,%u words=%08x%08x,%08x%08x,%08x%08x,%08x%08x\n",
            delay,width,coarse,read,s->location,patterns[s->location],
            s->edge_mask,s->pop_mask,s->pattern_mask,s->fresh_mask,
            s->edges[0],s->edges[1],s->edges[2],s->edges[3],
            s->pops[0],s->pops[1],s->pops[2],s->pops[3],
            (unsigned)(s->words[0]>>32),(unsigned)s->words[0],
            (unsigned)(s->words[1]>>32),(unsigned)s->words[1],
            (unsigned)(s->words[2]>>32),(unsigned)s->words[2],
            (unsigned)(s->words[3]>>32),(unsigned)s->words[3]);
    }
    return completed==12u;
}

/* Keep the established unprepared probe call and its output intact. */
static void nb_mpr_pattern_probe(unsigned delay, unsigned width,
                                  unsigned coarse)
{
    (void)nb_mpr_pattern_probe_prepared(delay,width,coarse,0,0,0);
}

#endif /* CONFIG_SDRAM_USNATIVE_MPR_RIU_GATE_COHORT */
#endif /* USNATIVE_NATIVE_MPR_PATTERN_PROBE_H */
