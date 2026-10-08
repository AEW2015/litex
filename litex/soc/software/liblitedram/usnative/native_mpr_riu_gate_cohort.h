/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Controlled RIU DQS-gate experiment for native x32 DDR4 MPR0.
 *
 * Include after native_mpr_probe.h, mapping.h and native_status_io.h.
 * Caller must already own DFII in software mode and have MR3 MPR0 enabled.
 * The probe is invoked from the MPR bootstrap, before the later 2D scan
 * changes RX taps or gate phases, so each RIU cohort starts from that
 * bootstrap state and restores only the settings it changes.
 * No array writes are issued. This is diagnostic-only: the RL_DLY_RNK
 * register is documented as reserved for MIG; see AMD UG861:
 * https://docs.amd.com/r/en-US/ug861-ultrascale-selectio/Register-Definitions-and-Addresses
 * NIBBLE_CTRL0.GT_STATUS/CLR_GATE are used only as documented observations
 * and training reset controls. Coarse settings are experimental candidates,
 * not assumed additive to the external gate launch time.
 */
#ifndef USNATIVE_NATIVE_MPR_RIU_GATE_COHORT_H
#define USNATIVE_NATIVE_MPR_RIU_GATE_COHORT_H

#ifdef CONFIG_SDRAM_USNATIVE_MPR_RIU_GATE_COHORT

#if !defined(CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC) || \
    !defined(CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP) || \
    !defined(CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR)
#error "RIU gate cohort requires MPR lane reads and DQS edge monitoring"
#endif

#if !defined(SDRAM_PHY_DDR4) || SDRAM_PHY_DATABITS != 32 || \
    SDRAM_PHY_DFI_DATABITS != 64 || SDRAM_PHY_PHASES != 4 || \
    CONFIG_CLOCK_FREQUENCY != 200000000 || SDRAM_PHY_CL != 11 || \
    SDRAM_PHY_CWL != 9 || SDRAM_PHY_USNATIVE_LANE_COUNT != 4 || \
    SDRAM_PHY_USNATIVE_DATA_CONTROLS_COUNT != 8
#error "RIU gate cohort is restricted to AES-KU40 native x32 DDR4-1600 1:4"
#endif

#include <stdint.h>
#include "native_mpr_rotation.h"
#include "native_mpr_lane3_tap.h"

#if !defined(SDRAM_PHY_USNATIVE_CONTROL_COUNT) || \
    SDRAM_PHY_USNATIVE_CONTROL_COUNT > 32
#error "RIU gate cohort requires a valid logical control mapping"
#endif

#define NB_MPR_RIU_CTRL0          0x00u
#define NB_MPR_RIU_RL_DLY_RNK0    0x30u
#define NB_MPR_RIU_GT_STATUS      (1u << 9)
#define NB_MPR_RIU_CLR_GATE       (1u << 8)
#define NB_MPR_RIU_CTRL0_WRITABLE 0x0d7fu /* exclude RO bit 9 and reserved bit 7 */
#define NB_MPR_RIU_RL_WRITABLE    0x1fffu

#if defined(CSR_DDRPHY_GATE_PHASE0_ADDR) && defined(CSR_DDRPHY_GATE_PHASE1_ADDR) && \
    defined(CSR_DDRPHY_GATE_PHASE2_ADDR) && defined(CSR_DDRPHY_GATE_PHASE3_ADDR) && \
    defined(CSR_DDRPHY_GATE_PHASE_UPPER0_ADDR) && defined(CSR_DDRPHY_GATE_PHASE_UPPER1_ADDR) && \
    defined(CSR_DDRPHY_GATE_PHASE_UPPER2_ADDR) && defined(CSR_DDRPHY_GATE_PHASE_UPPER3_ADDR)
#define NB_MPR_RIU_HAS_LANE_PHASE_CSRS
static unsigned nb_mpr_riu_phase_read(unsigned lane, unsigned upper)
{
    if (upper) {
        switch (lane) {
        case 0: return ddrphy_gate_phase_upper0_read();
        case 1: return ddrphy_gate_phase_upper1_read();
        case 2: return ddrphy_gate_phase_upper2_read();
        default: return ddrphy_gate_phase_upper3_read();
        }
    }
    switch (lane) {
    case 0: return ddrphy_gate_phase0_read();
    case 1: return ddrphy_gate_phase1_read();
    case 2: return ddrphy_gate_phase2_read();
    default: return ddrphy_gate_phase3_read();
    }
}
static void nb_mpr_riu_phase_write(unsigned lane, unsigned upper, unsigned value)
{
    if (upper) {
        switch (lane) {
        case 0: ddrphy_gate_phase_upper0_write(value); break;
        case 1: ddrphy_gate_phase_upper1_write(value); break;
        case 2: ddrphy_gate_phase_upper2_write(value); break;
        default: ddrphy_gate_phase_upper3_write(value); break;
        }
    } else {
        switch (lane) {
        case 0: ddrphy_gate_phase0_write(value); break;
        case 1: ddrphy_gate_phase1_write(value); break;
        case 2: ddrphy_gate_phase2_write(value); break;
        default: ddrphy_gate_phase3_write(value); break;
        }
    }
}
#endif

#ifdef CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN
#if !defined(CSR_DDRPHY_SCHEDULED_FIFO_MODE_ADDR) || \
    !defined(CSR_DDRPHY_SCHEDULED_FIFO_DELAY_ADDR) || \
    !defined(CSR_DDRPHY_SCHEDULED_FIFO_UNDERFLOWS_ADDR) || \
    !defined(CSR_DDRPHY_SCHEDULED_FIFO_OVERLAPS_ADDR) || \
    !defined(CSR_DDRPHY_SCHEDULED_FIFO_MISSING_LAST_ADDR)
#error "Scheduled-return RIU cohort requires scheduled FIFO mode/status CSRs"
#endif
#if !defined(CSR_DDRPHY_FIFO_EMPTY_ADDR) || !defined(CSR_DDRPHY_FIFO_EMPTY_SIZE) || \
    CSR_DDRPHY_FIFO_EMPTY_SIZE < ((SDRAM_PHY_USNATIVE_TAP_COUNT + 31) / 32)
#error "Scheduled-return RIU cohort requires the complete native FIFO_EMPTY CSR snapshot"
#endif

#if defined(CSR_DDRPHY_FIFO_EPOCH_FLUSH_ADDR) && \
    defined(CSR_DDRPHY_FIFO_EPOCH_FLUSH_COUNT_ADDR)
#define NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
#endif

/* Build an all-DQ EMPTY bitmap using the generated physical tap mapping.
 * Other FIFO taps (DM/DQS/address/control) are deliberately excluded. */
static unsigned nb_mpr_riu_dq_empty_mask(void)
{
    unsigned words[CSR_DDRPHY_FIFO_EMPTY_SIZE];
    unsigned mask=0;
    for (unsigned word=0; word<CSR_DDRPHY_FIFO_EMPTY_SIZE; ++word)
        words[word]=(unsigned)csr_read_simple(
            CSR_DDRPHY_FIFO_EMPTY_ADDR + 4u*word);
    for (unsigned dq=0; dq<32u; ++dq) {
        const unsigned tap=usnative_dq_taps[dq];
        unsigned word_index=tap/32u;
#ifdef CONFIG_CSR_ORDERING_BIG
        /* Wide CSR words are emitted most-significant subregister first. */
        word_index=CSR_DDRPHY_FIFO_EMPTY_SIZE-1u-word_index;
#endif
        if (words[word_index] & (1u << (tap%32u))) mask |= 1u << dq;
    }
    return mask;
}

/* Start a fresh diagnostic epoch with mode disabled. A new PHY may expose a
 * guarded diagnostic flush pulse; count only accepted pulse requests, and
 * always verify the actual all-DQ FIFO_EMPTY snapshot afterward. Older images
 * retain bounded observation-only behavior. */
static int nb_mpr_riu_prepare_dq_epoch(unsigned *last_mask, unsigned *stable)
{
    unsigned previous=0, run=0, flush_pulses=0;
    ddrphy_scheduled_fifo_mode_write(0);
    cdelay(100);
    if (ddrphy_scheduled_fifo_mode_read()!=0u) {
        *last_mask=nb_mpr_riu_dq_empty_mask();
        *stable=0;
        return 0;
    }
    for (unsigned poll=0; poll<100u; ++poll) {
        const unsigned mask=nb_mpr_riu_dq_empty_mask();
        if (mask==0xffffffffu && mask==previous) ++run;
        else run=(mask==0xffffffffu) ? 1u : 0u;
        previous=mask;
        *last_mask=mask;
        *stable=run;
        if (run>=3u) return 1;
#ifdef NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
        if (mask!=0xffffffffu) {
            const unsigned before=ddrphy_fifo_epoch_flush_count_read();
            ddrphy_fifo_epoch_flush_write(1u);
            cdelay(100); /* includes the hardware two-system-edge cooldown */
            const unsigned after=ddrphy_fifo_epoch_flush_count_read();
            const unsigned accepted=after-before;
            if (accepted!=1u) {
                printf("DDR4 MPR FIFO epoch flush rejected: nonempty=%08x accepted_delta=%u poll=%u; epoch not qualified\n",
                    mask,accepted,poll);
                return 0;
            }
            ++flush_pulses;
        }
#endif
        cdelay(100);
    }
    printf("DDR4 MPR FIFO epoch flush timeout: nonempty=%08x stable=%u accepted_pulses=%u; epoch not qualified\n",
        *last_mask,*stable,flush_pulses);
    return 0;
}

#ifdef NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
static int nb_mpr_riu_prepare_candidate_epoch(const char *stage,
    unsigned delay, unsigned width, unsigned coarse)
{
    unsigned mask=0,stable=0;
    const int ready=nb_mpr_riu_prepare_dq_epoch(&mask,&stable);
    printf("DDR4 MPR RIU candidate epoch: stage=%s delay=%u width=%u coarse=%u dq_empty=%08x stable=%u status=%s\n",
        stage,delay,width,coarse,mask,stable,ready ? "qualified" : "rejected");
    return ready;
}
#endif
#endif

/* 88 bytes/row, 1408 bytes total: includes all four raw DFI words. */
struct nb_mpr_riu_gate_sample {
    unsigned gt_status_mask;
    unsigned edge_mask;
    unsigned pop_mask;
    unsigned exact_mask;
    unsigned fresh_exact_mask;
    unsigned extra_pop_mask;
    unsigned edge_delta[4];
    unsigned fifo_delta[4];
    uint64_t words[4];
};

/* Capture one MPR location per armed raw RX trace. This runs only after the
 * measured identical-read and distinct-pattern cohorts, so it cannot add
 * UART traffic between their READ commands. Keep the existing trace index
 * selection intact for later diagnostics. */
#if defined(CSR_DDRPHY_RX_TRACE_WORD0_ADDR) && \
    defined(CSR_DDRPHY_RX_TRACE_DONE_ADDR) && \
    defined(CSR_DDRPHY_RX_TRACE_INDEX_ADDR) && \
    defined(CSR_DDRPHY_RX_TRACE_WIDTH_ADDR) && \
    defined(CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR)
static void nb_mpr_riu_location_trace(void)
{
    const unsigned saved_index=ddrphy_rx_trace_index_read();
    const unsigned trace_words=ddrphy_rx_trace_width_read()/32u;

    if (!trace_words || trace_words>64u) {
        printf("DDR4 MPR location trace unavailable: trace words=%u\n",trace_words);
        return;
    }

    for (unsigned location=0; location<3u; ++location) {
        uint64_t words[4];
        ddrphy_rx_trace_arm_write(1);
        cdelay(100);
        nb_mpr_lane_location_read(words,location);
        cdelay(100);
        for (unsigned poll=0; poll<1000u && !ddrphy_rx_trace_done_read(); ++poll)
            cdelay(1);

        printf("DDR4 MPR location read: location=%u words=%08x%08x,%08x%08x,%08x%08x,%08x%08x\n",
            location,
            (unsigned)(words[0]>>32),(unsigned)words[0],
            (unsigned)(words[1]>>32),(unsigned)words[1],
            (unsigned)(words[2]>>32),(unsigned)words[2],
            (unsigned)(words[3]>>32),(unsigned)words[3]);

        if (!ddrphy_rx_trace_done_read()) {
            printf("DDR4 MPR location trace timeout: location=%u\n",location);
            continue;
        }
        for (unsigned sample=0; sample<24u; ++sample) {
            ddrphy_rx_trace_index_write(sample);
            cdelay(100);
            printf("DDR4 MPR location trace: location=%u sample=%u data=",location,sample);
            for (unsigned word=0; word<trace_words; ++word)
                printf("%s%08x",word ? "," : "",
                    (unsigned)csr_read_simple(CSR_DDRPHY_RX_TRACE_WORD0_ADDR+4u*word));
            printf("\n");
        }
    }
    ddrphy_rx_trace_index_write(saved_index);
    cdelay(100);
}
#endif

static void nb_mpr_riu_gate_set_lane(unsigned lane, unsigned delay,
                                     unsigned width)
{
    switch (lane) {
    case 0:
        ddrphy_gate_delay0_write(delay); ddrphy_gate_width0_write(width); break;
    case 1:
        ddrphy_gate_delay1_write(delay); ddrphy_gate_width1_write(width); break;
    case 2:
        ddrphy_gate_delay2_write(delay); ddrphy_gate_width2_write(width); break;
    default:
        ddrphy_gate_delay3_write(delay); ddrphy_gate_width3_write(width); break;
    }
}

static unsigned nb_mpr_riu_gate_fifo_reads(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_fifo_reads0_read();
    case 1: return ddrphy_fifo_reads1_read();
    case 2: return ddrphy_fifo_reads2_read();
    default: return ddrphy_fifo_reads3_read();
    }
}

/* RMW each control so other CTRL0 configuration survives. GT_STATUS is RO;
 * write it as zero. Always deassert CLR_GATE before returning. */
static int nb_mpr_riu_gate_clear(unsigned control)
{
    unsigned value;
    if (!native_riu_access(control, NB_MPR_RIU_CTRL0, 0, 0, &value)) return 0;
    value &= NB_MPR_RIU_CTRL0_WRITABLE;
    if (!native_riu_access(control, NB_MPR_RIU_CTRL0,
            value | NB_MPR_RIU_CLR_GATE, 1, &value)) return 0;
    if (!native_riu_access(control, NB_MPR_RIU_CTRL0, 0, 0, &value)) return 0;
    value &= NB_MPR_RIU_CTRL0_WRITABLE & ~NB_MPR_RIU_CLR_GATE;
    if (!native_riu_access(control, NB_MPR_RIU_CTRL0, value, 1, &value)) return 0;
    return 1;
}

/* The reset-on-each-read cohort checks every RIU transaction and verifies
 * that the writable CTRL0 state is restored with CLR_GATE deasserted. */
static int nb_mpr_riu_gate_clear_checked(unsigned control)
{
    unsigned value, actual;
    if (!native_riu_access(control,NB_MPR_RIU_CTRL0,0,0,&value)) return 0;
    const unsigned preserved=value & NB_MPR_RIU_CTRL0_WRITABLE & ~NB_MPR_RIU_CLR_GATE;
    if (!native_riu_access(control,NB_MPR_RIU_CTRL0,
            preserved | NB_MPR_RIU_CLR_GATE,1,&actual)) return 0;
    if (!native_riu_access(control,NB_MPR_RIU_CTRL0,0,0,&value) ||
        (value & NB_MPR_RIU_CTRL0_WRITABLE & ~NB_MPR_RIU_CLR_GATE)!=preserved ||
        !(value & NB_MPR_RIU_CLR_GATE))
        return 0;
    if (!native_riu_access(control,NB_MPR_RIU_CTRL0,preserved,1,&actual)) return 0;
    if (!native_riu_access(control,NB_MPR_RIU_CTRL0,0,0,&value) ||
        (value & NB_MPR_RIU_CTRL0_WRITABLE)!=preserved)
        return 0;
    return 1;
}

struct nb_mpr_riu_gate_clear_context {
    unsigned attempts, successful, failed_read, failed_control;
};

static int nb_mpr_riu_gate_prepare_pattern_read(void *opaque,
    unsigned read, unsigned location)
{
    struct nb_mpr_riu_gate_clear_context *context=opaque;
    (void)location;
    for (unsigned index=0; index<USNATIVE_ARRAY_COUNT(usnative_data_controls); ++index) {
        const unsigned control=usnative_data_controls[index];
        context->attempts++;
        if (!nb_mpr_riu_gate_clear_checked(control)) {
            context->failed_read=read;
            context->failed_control=control;
            return 0;
        }
        context->successful++;
    }
    return 1;
}

static int nb_mpr_riu_gate_read_status(unsigned *mask)
{
    unsigned value;
    *mask = 0;
    for (unsigned i=0; i<USNATIVE_ARRAY_COUNT(usnative_data_controls); ++i) {
        if (!native_riu_access(usnative_data_controls[i], NB_MPR_RIU_CTRL0,
                               0, 0, &value)) return 0;
        *mask |= ((value & NB_MPR_RIU_GT_STATUS) != 0u) << i;
    }
    return 1;
}

#if defined(NB_MPR_RIU_HAS_LANE_PHASE_CSRS) && \
    defined(CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN)
static unsigned nb_mpr_riu_fifo_count(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_fifo_reads0_read();
    case 1: return ddrphy_fifo_reads1_read();
    case 2: return ddrphy_fifo_reads2_read();
    default: return ddrphy_fifo_reads3_read();
    }
}

static unsigned nb_mpr_riu_edge_count(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_dqs_wrclk_edges0_read();
    case 1: return ddrphy_dqs_wrclk_edges1_read();
    case 2: return ddrphy_dqs_wrclk_edges2_read();
    default: return ddrphy_dqs_wrclk_edges3_read();
    }
}

/* Buffered location reads let the host score every UI tag while preserving
 * request cadence. Return the number of joint all-lane fresh reads and the
 * target lane's fresh-read count. */
static unsigned nb_mpr_riu_phase_pattern_probe(unsigned target_lane,
    unsigned lower, unsigned upper, unsigned *target_fresh,
    unsigned *joint_fresh, unsigned *all_exact_reads)
{
    static const unsigned patterns[3]={0x55u,0x33u,0x0fu};
    static const unsigned locations[12]={0,1,2,0,2,1,0,1,2,1,0,2};
    struct nb_mpr_riu_phase_sample {
        unsigned location, edge_mask, pop_mask, pattern_mask, fresh_mask;
        unsigned edges[4], pops[4];
        uint64_t words[4];
    } samples[12];
    *target_fresh=0;
    *joint_fresh=0;
    *all_exact_reads=0;
    for (unsigned read=0; read<12u; ++read) {
        struct nb_mpr_riu_phase_sample *sample=&samples[read];
        sample->location=locations[read];
        sample->edge_mask=sample->pop_mask=sample->pattern_mask=sample->fresh_mask=0;
        for (unsigned lane=0; lane<4u; ++lane) {
            sample->edges[lane]=nb_mpr_riu_edge_count(lane);
            sample->pops[lane]=nb_mpr_riu_fifo_count(lane);
        }
        nb_mpr_lane_location_read(sample->words,sample->location);
        cdelay(100);
        for (unsigned lane=0; lane<4u; ++lane) {
            sample->edges[lane]=nb_mpr_riu_edge_count(lane)-sample->edges[lane];
            sample->pops[lane]=nb_mpr_riu_fifo_count(lane)-sample->pops[lane];
            const unsigned matched=nb_mpr_pattern_lane_matches(sample->words,
                lane,patterns[sample->location]);
            sample->edge_mask |= (sample->edges[lane]==1u)<<lane;
            sample->pop_mask |= (sample->pops[lane]==1u)<<lane;
            sample->pattern_mask |= matched<<lane;
            sample->fresh_mask |= (matched && sample->edges[lane]==1u &&
                sample->pops[lane]==1u)<<lane;
        }
        *target_fresh += (sample->fresh_mask>>target_lane)&1u;
        *joint_fresh += sample->fresh_mask==0xfu;
        *all_exact_reads += sample->edge_mask==0xfu && sample->pop_mask==0xfu;
    }
    /* Interpret the same buffered words for UI rotation. Keep this separate
     * from freshness: all 12 edge/pop deltas must be exactly one, and every
     * DQ must agree on one unique rotation before the diagnostic reports it. */
    unsigned coherent_lane_mask=0;
    for (unsigned lane=0; lane<4u; ++lane) {
        uint8_t dq_masks[8]={0,0,0,0,0,0,0,0};
        unsigned lane_health=1;
        for (unsigned dq_in_lane=0; dq_in_lane<8u; ++dq_in_lane) {
            uint8_t captured[12],locations[12],rotation_mask=0;
            const unsigned dq=8u*lane+dq_in_lane;
            for (unsigned read=0; read<12u; ++read) {
                const struct nb_mpr_riu_phase_sample *sample=&samples[read];
                unsigned byte=0;
                locations[read]=(uint8_t)sample->location;
                if (sample->edges[lane]!=1u || sample->pops[lane]!=1u)
                    lane_health=0;
                for (unsigned ui=0; ui<8u; ++ui) {
                    const unsigned phase=ui/2u;
                    const unsigned bit=(unsigned)(sample->words[phase] >>
                        (dq+((ui&1u) ? 32u : 0u)))&1u;
                    byte|=bit<<ui;
                }
                captured[read]=(uint8_t)byte;
            }
            if (lane_health)
                (void)usnative_mpr_rotation_cohort_mask(captured,locations,
                    &rotation_mask);
            dq_masks[dq_in_lane]=rotation_mask;
        }
        uint8_t lane_rotation=0;
        const unsigned unique=lane_health &&
            usnative_mpr_rotation_lane_mask(dq_masks,&lane_rotation);
        coherent_lane_mask |= unique<<lane;
        unsigned observed_rotation=0;
        while (observed_rotation<8u && !(lane_rotation&(1u<<observed_rotation)))
            ++observed_rotation;
        printf("DDR4 MPR UI rotation: lane=%u observed_right=%u mask=%02x correction_right=%u unique=%u fresh_health=%u diagnostic_only=1\n",
            lane,observed_rotation,lane_rotation,
            unique ? ((8u-observed_rotation)&7u) : 0u,unique,lane_health);
    }
    for (unsigned read=0; read<12u; ++read) {
        const struct nb_mpr_riu_phase_sample *s=&samples[read];
        printf("DDR4 MPR pattern row: delay=1 width=1 coarse=4 read=%u location=%u pattern=%02x edge=%x pop=%x match=%x fresh=%x edge_delta=%u,%u,%u,%u fifo_delta=%u,%u,%u,%u words=%08x%08x,%08x%08x,%08x%08x,%08x%08x\n",
            read,s->location,patterns[s->location],s->edge_mask,s->pop_mask,
            s->pattern_mask,s->fresh_mask,
            s->edges[0],s->edges[1],s->edges[2],s->edges[3],
            s->pops[0],s->pops[1],s->pops[2],s->pops[3],
            (unsigned)(s->words[0]>>32),(unsigned)s->words[0],
            (unsigned)(s->words[1]>>32),(unsigned)s->words[1],
            (unsigned)(s->words[2]>>32),(unsigned)s->words[2],
            (unsigned)(s->words[3]>>32),(unsigned)s->words[3]);
    }
    (void)lower;
    (void)upper;
    return coherent_lane_mask;
}

/* The fallback varies only lane 3's normal per-byte RX DQ delay. Snapshot and
 * restore its eight logical DQ taps individually because an earlier per-bit
 * deskew may have left them at different values. */
static int nb_mpr_riu_read_dq_lane_rx_taps(unsigned lane, unsigned taps[8])
{
    if (lane>=4u) return 0;
    for (unsigned bit=0; bit<8u; ++bit) {
        const unsigned dq=lane*8u+bit;
        ddrphy_tap_select_write(usnative_dq_taps[dq]);
        cdelay(100);
        if (!native_tap_wait() || !ddrphy_tap_allowed_read()) return 0;
        taps[bit]=ddrphy_tap_rx_count_read();
    }
    return 1;
}

static int nb_mpr_riu_restore_one_dq_rx_tap(unsigned dq, unsigned tap,
    void *context)
{
    (void)context;
    if (dq>=32u || tap>=SDRAM_PHY_DELAYS) return 0;
    ddrphy_tap_select_write(usnative_dq_taps[dq]);
    cdelay(100);
    if (!native_tap_wait() || !ddrphy_tap_allowed_read()) return 0;
    ddrphy_tap_rx_rst_write(1u);
    cdelay(100);
    for (unsigned i=0; i<tap; ++i) {
        ddrphy_tap_rx_inc_write(1u);
        cdelay(100);
    }
    return native_tap_wait() && ddrphy_tap_allowed_read() &&
        ddrphy_tap_rx_count_read()==tap;
}

static int nb_mpr_riu_readback_one_dq_rx_tap(unsigned dq, unsigned *tap,
    void *context)
{
    (void)context;
    if (dq>=32u || !tap) return 0;
    ddrphy_tap_select_write(usnative_dq_taps[dq]);
    cdelay(100);
    if (!native_tap_wait() || !ddrphy_tap_allowed_read()) return 0;
    *tap=ddrphy_tap_rx_count_read();
    return 1;
}

static int nb_mpr_riu_restore_dq_lane_rx_taps(unsigned lane,
    const unsigned taps[8])
{
    return usnative_mpr_restore_lane_taps(lane,taps,8u,
        nb_mpr_riu_restore_one_dq_rx_tap,
        nb_mpr_riu_readback_one_dq_rx_tap,0);
}

static int nb_mpr_riu_set_lane_rx_tap(unsigned lane, unsigned tap)
{
    if (lane>=4u || tap>96u || (tap&7u)) return 0;
    /* nb_delay is the existing lane-wide RXDQ reset/CE API; only the
     * selected byte lane is enabled. */
    if (!nb_delay(lane,tap,0)) return 0;
    for (unsigned bit=0; bit<8u; ++bit) {
        const unsigned dq=lane*8u+bit;
        ddrphy_tap_select_write(usnative_dq_taps[dq]);
        cdelay(100);
        if (!native_tap_wait() || !ddrphy_tap_allowed_read() ||
            ddrphy_tap_rx_count_read()!=tap) return 0;
    }
    return 1;
}

static unsigned nb_mpr_riu_phase_distance(unsigned lower, unsigned upper,
    unsigned saved_lower, unsigned saved_upper)
{
    unsigned dl=lower>saved_lower ? lower-saved_lower : saved_lower-lower;
    unsigned du=upper>saved_upper ? upper-saved_upper : saved_upper-upper;
    if (dl>2u) dl=4u-dl;
    if (du>2u) du=4u-du;
    return dl+du;
}

/* Sweep the independent lower/upper mask phases for one byte at a time while
 * holding gate=1, width=1, coarse=4 and the default (mode 0) RX return path.
 * Results are diagnostic only: any chosen vector is applied temporarily for
 * a joint confirmation, then every register is restored and checked. */
static int nb_mpr_riu_lane_phase_cohort(void)
{
    unsigned saved_lower[4],saved_upper[4],saved_delay[4],saved_width[4];
    unsigned saved_lane3_rx[8],saved_lane3_rx_count=0;
    const unsigned saved_dly_sel=ddrphy_dly_sel_read();
    const unsigned saved_tap_select=ddrphy_tap_select_read();
    unsigned lane3_rx_tap_changed=0;
    unsigned saved_rl[8],saved_mode=0,saved_fifo_delay=0;
    const unsigned saved_global=ddrphy_gate_delay_read();
    const unsigned saved_override=ddrphy_gate_override_read();
    const unsigned saved_vtc=ddrphy_en_vtc_read();
    /* This diagnostic writes experimental gate/phase and MIG-reserved RL
     * controls. Its normal caller is nb_boot() with VTC and scheduled return
     * disabled. Refuse unsupported direct entry before any RIU/CSR mutation
     * or MPR command; restoring a mode bit cannot restore an in-flight read. */
    const unsigned entry_mode=ddrphy_scheduled_fifo_mode_read();
    if (saved_vtc || entry_mode) {
        printf("DDR4 MPR lane-phase cohort unsupported entry: EN_VTC=%u scheduled_mode=%u; no changes made\n",
            saved_vtc,entry_mode);
        return 0;
    }
    unsigned best_lower[4]={0,0,0,0},best_upper[4]={0,0,0,0};
    unsigned best_distance[4]={~0u,~0u,~0u,~0u};
    unsigned have_candidate[4]={0,0,0,0};
    unsigned experiment_error=0,restore_error=0,rl_saved=0;
    unsigned lane3_tap_candidate=0;

    for (unsigned lane=0; lane<4u; ++lane) {
        saved_lower[lane]=nb_mpr_riu_phase_read(lane,0);
        saved_upper[lane]=nb_mpr_riu_phase_read(lane,1);
        saved_delay[lane]=lane==0 ? ddrphy_gate_delay0_read() :
            lane==1 ? ddrphy_gate_delay1_read() :
            lane==2 ? ddrphy_gate_delay2_read() : ddrphy_gate_delay3_read();
        saved_width[lane]=lane==0 ? ddrphy_gate_width0_read() :
            lane==1 ? ddrphy_gate_width1_read() :
            lane==2 ? ddrphy_gate_width2_read() : ddrphy_gate_width3_read();
    }
    if (!nb_mpr_riu_read_dq_lane_rx_taps(3u,saved_lane3_rx)) {
        experiment_error=1;
        goto restore;
    }
    saved_lane3_rx_count=8u;
    saved_mode=ddrphy_scheduled_fifo_mode_read();
    saved_fifo_delay=ddrphy_scheduled_fifo_delay_read();
    for (unsigned i=0; i<8u; ++i) {
        unsigned actual;
        if (!native_riu_access(usnative_data_controls[i],NB_MPR_RIU_RL_DLY_RNK0,
                               0,0,&actual)) {
            experiment_error=1;
            goto restore;
        }
        saved_rl[i]=actual&NB_MPR_RIU_RL_WRITABLE;
        rl_saved++;
    }

    ddrphy_scheduled_fifo_mode_write(0);
    cdelay(100);
    if (ddrphy_scheduled_fifo_mode_read()!=0u) {
        experiment_error=1;
        goto restore;
    }
    ddrphy_gate_override_write(USNATIVE_LANE_MASK);
    ddrphy_gate_delay_write(1u);
    for (unsigned lane=0; lane<4u; ++lane) nb_mpr_riu_gate_set_lane(lane,1u,1u);
    cdelay(100);
    for (unsigned i=0; i<8u; ++i) {
        const unsigned candidate=(saved_rl[i]&0x01ffu)|(4u<<9);
        unsigned actual;
        if (!native_riu_access(usnative_data_controls[i],NB_MPR_RIU_RL_DLY_RNK0,
                               candidate,1,&actual) ||
            !native_riu_access(usnative_data_controls[i],NB_MPR_RIU_RL_DLY_RNK0,
                               0,0,&actual) ||
            (actual&NB_MPR_RIU_RL_WRITABLE)!=candidate) {
            experiment_error=1;
            goto restore;
        }
    }

    printf("DDR4 MPR lane-phase cohort: gate_delay=1 gate_width=1 coarse=4 phase_range=0..3 mode=0; each lane/phase cohort is isolated\n");
    for (unsigned lane=0; lane<4u && !experiment_error; ++lane) {
        for (unsigned lower=0; lower<4u && !experiment_error; ++lower) {
            for (unsigned upper=0; upper<4u && !experiment_error; ++upper) {
                for (unsigned restore_lane=0; restore_lane<4u; ++restore_lane) {
                    nb_mpr_riu_phase_write(restore_lane,0,saved_lower[restore_lane]);
                    nb_mpr_riu_phase_write(restore_lane,1,saved_upper[restore_lane]);
                }
                nb_mpr_riu_phase_write(lane,0,lower);
                nb_mpr_riu_phase_write(lane,1,upper);
                cdelay(100);
                if (nb_mpr_riu_phase_read(lane,0)!=lower ||
                    nb_mpr_riu_phase_read(lane,1)!=upper) {
                    experiment_error=1;
                    break;
                }
#ifdef NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
                unsigned warmup_empty=0,warmup_stable=0;
                const int warmup_epoch_ok=nb_mpr_riu_prepare_dq_epoch(
                    &warmup_empty,&warmup_stable);
                printf("DDR4 MPR lane-phase warmup epoch: lane=%u phase=%u upper=%u gate_delay=1 gate_width=1 coarse=4 dq_empty=%08x stable=%u status=%s\n",
                    lane,lower,upper,warmup_empty,warmup_stable,
                    warmup_epoch_ok ? "qualified" : "rejected");
                if (!warmup_epoch_ok) {
                    experiment_error=1;
                    break;
                }
#endif
                for (unsigned warmup=0; warmup<4u; ++warmup) {
                    uint64_t words[4];
                    nb_mpr_lane_sweep_read(words);
                    cdelay(100);
                }
                unsigned empty_mask=0,stable=0;
                const int epoch_ok=nb_mpr_riu_prepare_dq_epoch(&empty_mask,&stable);
                unsigned target_fresh=0,joint_fresh=0,all_exact_reads=0,coherent_lanes=0;
                printf("DDR4 MPR lane-phase begin: lane=%u phase=%u upper=%u base_phase=%u base_upper=%u gate_delay=1 gate_width=1 coarse=4 epoch_empty=%08x epoch_stable=%u\n",
                    lane,lower,upper,saved_lower[lane],saved_upper[lane],empty_mask,stable);
                if (epoch_ok) coherent_lanes=nb_mpr_riu_phase_pattern_probe(lane,lower,upper,
                    &target_fresh,&joint_fresh,&all_exact_reads);
                printf("DDR4 MPR lane-phase summary: lane=%u phase=%u upper=%u reads=%u target_fresh=%u joint_fresh=%u exact_edgepop_reads=%u coherent_lanes=%x epoch_empty=%08x epoch_stable=%u status=%s\n",
                    lane,lower,upper,epoch_ok ? 12u : 0u,target_fresh,joint_fresh,
                    all_exact_reads,coherent_lanes,empty_mask,stable,
                    epoch_ok ? "complete" : "epoch_error");
                printf("DDR4 MPR lane-phase end: lane=%u phase=%u upper=%u\n",
                    lane,lower,upper);
                if (!epoch_ok) {
                    experiment_error=1;
                    break;
                }
                const unsigned distance=nb_mpr_riu_phase_distance(lower,upper,
                    saved_lower[lane],saved_upper[lane]);
                if (target_fresh==12u && (coherent_lanes&(1u<<lane)) &&
                    (!have_candidate[lane] ||
                        distance<best_distance[lane])) {
                    have_candidate[lane]=1;
                    best_distance[lane]=distance;
                    best_lower[lane]=lower;
                    best_upper[lane]=upper;
                }
            }
        }
    }

    /* If the exhaustive 64-point phase cohort found no lane-3 candidate,
     * still perform the bounded RXDQ lane-3 sweep. Use each passing phase
     * candidate where one exists; retain the saved phase for every other
     * lane without a candidate. Lane 3 phases stay at zero. The sweep is
     * diagnostic only and cannot qualify a joint vector while any other
     * lane lacks a coherent candidate. */
    if (!experiment_error && !have_candidate[3]) {
            unsigned pass[USNATIVE_MPR_LANE3_TAP_POINTS]={0};
            unsigned original_average=0;
            unsigned other_lane_qualified_mask=0,retained_phase_mask=0;
            unsigned test_lower[3],test_upper[3];
            for (unsigned i=0; i<8u; ++i) original_average+=saved_lane3_rx[i];
            original_average=(original_average+4u)/8u;
            for (unsigned lane=0; lane<3u; ++lane) {
                if (have_candidate[lane]) {
                    other_lane_qualified_mask|=1u<<lane;
                    test_lower[lane]=best_lower[lane];
                    test_upper[lane]=best_upper[lane];
                } else {
                    retained_phase_mask|=1u<<lane;
                    test_lower[lane]=saved_lower[lane];
                    test_upper[lane]=saved_upper[lane];
                }
                nb_mpr_riu_phase_write(lane,0,test_lower[lane]);
                nb_mpr_riu_phase_write(lane,1,test_upper[lane]);
            }
            nb_mpr_riu_phase_write(3u,0,0u);
            nb_mpr_riu_phase_write(3u,1,0u);
            cdelay(100);
            for (unsigned lane=0; lane<4u; ++lane) {
                const unsigned expected_lower=lane==3u ? 0u : test_lower[lane];
                const unsigned expected_upper=lane==3u ? 0u : test_upper[lane];
                if (nb_mpr_riu_phase_read(lane,0)!=expected_lower ||
                    nb_mpr_riu_phase_read(lane,1)!=expected_upper) experiment_error=1;
            }
            if (experiment_error) goto restore;
            printf("DDR4 MPR lane3 RX-tap sweep begin: taps=0..96 step=8 gate_delay=1 gate_width=1 coarse=4 phase_vector=%u,%u,%u,0/%u,%u,%u,0 other_lane_vector=measured_or_saved other_lane_qualified_mask=%x retained_phase_mask=%x diagnostic_only=1\n",
                test_lower[0],test_lower[1],test_lower[2],test_upper[0],test_upper[1],test_upper[2],
                other_lane_qualified_mask,retained_phase_mask);
            for (unsigned index=0; index<USNATIVE_MPR_LANE3_TAP_POINTS && !experiment_error; ++index) {
                const unsigned tap=USNATIVE_MPR_LANE3_TAP_STEP*index;
                printf("DDR4 MPR lane3 RX-tap point begin: tap=%u diagnostic_only=1\n",tap);
                lane3_rx_tap_changed=1;
                if (!nb_mpr_riu_set_lane_rx_tap(3u,tap)) {
                    printf("DDR4 MPR lane3 RX-tap point end: tap=%u status=tap_error\n",tap);
                    experiment_error=1;
                    break;
                }
                unsigned warmup_empty=0,warmup_stable=0;
                const int warmup_epoch_ok=nb_mpr_riu_prepare_dq_epoch(
                    &warmup_empty,&warmup_stable);
                printf("DDR4 MPR lane3 RX-tap epoch: tap=%u stage=warmup dq_empty=%08x stable=%u status=%s\n",
                    tap,warmup_empty,warmup_stable,
                    warmup_epoch_ok ? "qualified" : "rejected");
                if (!warmup_epoch_ok) {
                    printf("DDR4 MPR lane3 RX-tap point end: tap=%u status=warmup_epoch_error\n",tap);
                    experiment_error=1;
                    break;
                }
                for (unsigned warmup=0; warmup<4u; ++warmup) {
                    uint64_t words[4];
                    nb_mpr_lane_sweep_read(words);
                    cdelay(100);
                }
                unsigned empty_mask=0,stable=0;
                const int epoch_ok=nb_mpr_riu_prepare_dq_epoch(&empty_mask,&stable);
                unsigned target_fresh=0,joint_fresh=0,all_exact_reads=0,coherent_lanes=0;
                printf("DDR4 MPR lane3 RX-tap epoch: tap=%u stage=measured dq_empty=%08x stable=%u status=%s\n",
                    tap,empty_mask,stable,epoch_ok ? "qualified" : "rejected");
                if (epoch_ok) coherent_lanes=nb_mpr_riu_phase_pattern_probe(3u,0u,0u,
                    &target_fresh,&joint_fresh,&all_exact_reads);
                pass[index]=usnative_mpr_lane3_tap_candidate(epoch_ok,
                    target_fresh,all_exact_reads,coherent_lanes);
                printf("DDR4 MPR lane3 RX-tap summary: tap=%u reads=%u target_fresh=%u joint_fresh=%u exact_edgepop_reads=%u coherent_lanes=%x epoch_empty=%08x epoch_stable=%u status=%s candidate=%u diagnostic_only=1\n",
                    tap,epoch_ok ? 12u : 0u,target_fresh,joint_fresh,
                    all_exact_reads,coherent_lanes,empty_mask,stable,
                    epoch_ok ? "complete" : "epoch_error",pass[index]);
                printf("DDR4 MPR lane3 RX-tap point end: tap=%u status=%s\n",
                    tap,epoch_ok ? "complete" : "epoch_error");
                if (!epoch_ok) experiment_error=1;
            }
            if (!experiment_error) {
                unsigned chosen=0,first=0,last=0,best_run=0;
                if (usnative_mpr_lane3_choose_tap(pass,original_average,
                        &chosen,&first,&last,&best_run)) {
                    lane3_rx_tap_changed=1;
                    if (!nb_mpr_riu_set_lane_rx_tap(3u,chosen)) {
                        printf("DDR4 MPR lane3 RX-tap selection failed: tap=%u\n",chosen);
                        experiment_error=1;
                    } else {
                        best_lower[3]=0u; best_upper[3]=0u;
                        have_candidate[3]=1u;
                        lane3_tap_candidate=1u;
                        printf("DDR4 MPR lane3 RX-tap selected: range=%u..%u samples=%u tap=%u exact_edgepop=12/12 diagnostic_only=1\n",
                            first,last,best_run,chosen);
                    }
                } else {
                    printf("DDR4 MPR lane3 RX-tap sweep result: no tap passed target_fresh=12, exact_edgepop_reads=12, and coherent lane-3 rotation; joint confirmation skipped\n");
                }
            }
            printf("DDR4 MPR lane3 RX-tap sweep end: status=%s diagnostic_only=1\n",
                experiment_error ? "aborted" : lane3_tap_candidate ? "complete" : "no_candidate");
    }

    if (!experiment_error && have_candidate[0] && have_candidate[1] &&
            have_candidate[2] && have_candidate[3]) {
        for (unsigned lane=0; lane<4u; ++lane) {
            nb_mpr_riu_phase_write(lane,0,best_lower[lane]);
            nb_mpr_riu_phase_write(lane,1,best_upper[lane]);
        }
        cdelay(100);
#ifdef NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
        unsigned warmup_empty=0,warmup_stable=0;
        if (!nb_mpr_riu_prepare_dq_epoch(&warmup_empty,&warmup_stable)) {
            printf("DDR4 MPR lane-phase joint warmup epoch ERROR: gate_delay=1 gate_width=1 coarse=4 dq_empty=%08x stable=%u\n",
                warmup_empty,warmup_stable);
            experiment_error=1;
        }
#endif
        for (unsigned warmup=0; warmup<4u; ++warmup) {
            if (experiment_error) break;
            uint64_t words[4];
            nb_mpr_lane_sweep_read(words);
            cdelay(100);
        }
        unsigned empty_mask=0,stable=0,target_fresh=0,joint_fresh=0,all_exact_reads=0,coherent_lanes=0;
        const int epoch_ok=nb_mpr_riu_prepare_dq_epoch(&empty_mask,&stable);
        printf("DDR4 MPR lane-phase joint begin: gate_delay=1 gate_width=1 coarse=4 epoch_empty=%08x epoch_stable=%u vector=%u,%u,%u,%u/%u,%u,%u,%u\n",
            empty_mask,stable,best_lower[0],best_lower[1],best_lower[2],best_lower[3],
            best_upper[0],best_upper[1],best_upper[2],best_upper[3]);
        if (epoch_ok) coherent_lanes=nb_mpr_riu_phase_pattern_probe(0,0,0,&target_fresh,
            &joint_fresh,&all_exact_reads);
        printf("DDR4 MPR lane-phase joint summary: reads=%u target_fresh=%u joint_fresh=%u exact_edgepop_reads=%u coherent_lanes=%x epoch_empty=%08x epoch_stable=%u status=%s diagnostic_only=1\n",
            epoch_ok ? 12u : 0u,target_fresh,joint_fresh,all_exact_reads,coherent_lanes,empty_mask,stable,
            epoch_ok ? "complete" : "epoch_error");
        if (!epoch_ok) experiment_error=1;
            printf("DDR4 MPR lane-phase joint verdict: status=%s target_fresh=%u joint_fresh=%u exact_edgepop_reads=%u coherent_lanes=%x required=12 diagnostic_only=1\n",
            usnative_mpr_lane3_joint_confirmed(epoch_ok,target_fresh,
                joint_fresh,all_exact_reads,coherent_lanes) ? "confirmed" : "not_confirmed",
            target_fresh,joint_fresh,all_exact_reads,coherent_lanes);
        if (lane3_tap_candidate)
            printf("DDR4 MPR lane3 RX-tap joint verdict: status=%s target_fresh=%u joint_fresh=%u exact_edgepop_reads=%u coherent_lanes=%x required=12 diagnostic_only=1\n",
                usnative_mpr_lane3_joint_confirmed(epoch_ok,target_fresh,
                    joint_fresh,all_exact_reads,coherent_lanes) ? "confirmed" : "not_confirmed",
                target_fresh,joint_fresh,all_exact_reads,coherent_lanes);
    } else if (!experiment_error) {
        printf("DDR4 MPR lane-phase joint skipped: per-lane 12/12 candidate missing=%u,%u,%u,%u\n",
            !have_candidate[0],!have_candidate[1],!have_candidate[2],!have_candidate[3]);
    }

restore:
    if (lane3_rx_tap_changed && saved_lane3_rx_count==8u &&
        !nb_mpr_riu_restore_dq_lane_rx_taps(3u,saved_lane3_rx))
        restore_error=1;
    ddrphy_tap_select_write(saved_tap_select);
    ddrphy_dly_sel_write(saved_dly_sel);
    cdelay(100);
    if (ddrphy_tap_select_read()!=saved_tap_select ||
        ddrphy_dly_sel_read()!=saved_dly_sel) restore_error=1;
    ddrphy_scheduled_fifo_mode_write(0);
    cdelay(100);
    if (ddrphy_scheduled_fifo_mode_read()!=0u) restore_error=1;
    ddrphy_scheduled_fifo_delay_write(saved_fifo_delay);
    cdelay(100);
    if (ddrphy_scheduled_fifo_delay_read()!=saved_fifo_delay) restore_error=1;
    ddrphy_gate_delay_write(saved_global);
    for (unsigned lane=0; lane<4u; ++lane) {
        nb_mpr_riu_gate_set_lane(lane,saved_delay[lane],saved_width[lane]);
        nb_mpr_riu_phase_write(lane,0,saved_lower[lane]);
        nb_mpr_riu_phase_write(lane,1,saved_upper[lane]);
    }
    ddrphy_gate_override_write(saved_override);
    cdelay(100);
    if (ddrphy_gate_delay_read()!=saved_global ||
        ddrphy_gate_override_read()!=saved_override ||
        ddrphy_scheduled_fifo_mode_read()!=0u ||
        ddrphy_scheduled_fifo_delay_read()!=saved_fifo_delay ||
        ddrphy_en_vtc_read()!=saved_vtc) restore_error=1;
    for (unsigned lane=0; lane<4u; ++lane) {
        if (nb_mpr_riu_phase_read(lane,0)!=saved_lower[lane] ||
            nb_mpr_riu_phase_read(lane,1)!=saved_upper[lane]) restore_error=1;
    }
    for (unsigned lane=0; lane<4u; ++lane) {
        const unsigned delay=lane==0 ? ddrphy_gate_delay0_read() :
            lane==1 ? ddrphy_gate_delay1_read() :
            lane==2 ? ddrphy_gate_delay2_read() : ddrphy_gate_delay3_read();
        const unsigned width=lane==0 ? ddrphy_gate_width0_read() :
            lane==1 ? ddrphy_gate_width1_read() :
            lane==2 ? ddrphy_gate_width2_read() : ddrphy_gate_width3_read();
        if (delay!=saved_delay[lane] || width!=saved_width[lane]) restore_error=1;
    }
    for (unsigned i=0; i<rl_saved; ++i) {
        unsigned actual;
        if (!native_riu_access(usnative_data_controls[i],NB_MPR_RIU_RL_DLY_RNK0,
                               saved_rl[i],1,&actual) ||
            !native_riu_access(usnative_data_controls[i],NB_MPR_RIU_RL_DLY_RNK0,
                               0,0,&actual) ||
            (actual&NB_MPR_RIU_RL_WRITABLE)!=saved_rl[i]) restore_error=1;
    }
    ddrphy_scheduled_fifo_mode_write(saved_mode);
    cdelay(100);
    if (ddrphy_scheduled_fifo_mode_read()!=saved_mode) restore_error=1;
    printf("DDR4 MPR lane-phase cohort restored: experiment=%s restore=%s; phase/gate/RL/scheduler state verified\n",
        experiment_error ? "ERROR" : "complete",
        restore_error ? "FAIL" : "VERIFIED");
    return restore_error ? 0 : 1;
}
#endif

static int nb_mpr_riu_gate_cohort(void)
{
    static const unsigned warmup_reads = 4u;
    static const unsigned measured_reads = 16u;
    struct nb_mpr_riu_gate_sample samples[16];
    unsigned saved_ctrl0[8], saved_rl[8], current_rl[8];
    unsigned saved_delay[4], saved_width[4];
    const unsigned saved_override = ddrphy_gate_override_read();
    unsigned controls_saved = 0;
    unsigned software_gate_modified = 0;
    unsigned experiment_error = 0, restore_error = 0;
    const unsigned saved_global_delay = ddrphy_gate_delay_read();
#ifdef CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN
    const unsigned saved_scheduled_mode=ddrphy_scheduled_fifo_mode_read();
    const unsigned saved_scheduled_delay=ddrphy_scheduled_fifo_delay_read();
    unsigned scheduled_controls_modified=0;
#endif

    /* Validate the generated physical-controller list rather than assuming
     * controller numbers or their ordering in the native wrapper. */
    if (USNATIVE_ARRAY_COUNT(usnative_data_controls) != 8u ||
        !usnative_mapping_validate()) {
        printf("DDR4 MPR RIU gate cohort: invalid generated control mapping\n");
        return 1;
    }

    saved_delay[0]=ddrphy_gate_delay0_read();
    saved_delay[1]=ddrphy_gate_delay1_read();
    saved_delay[2]=ddrphy_gate_delay2_read();
    saved_delay[3]=ddrphy_gate_delay3_read();
    saved_width[0]=ddrphy_gate_width0_read();
    saved_width[1]=ddrphy_gate_width1_read();
    saved_width[2]=ddrphy_gate_width2_read();
    saved_width[3]=ddrphy_gate_width3_read();

    for (unsigned i=0; i<8u; ++i) {
        unsigned value;
        const unsigned control=usnative_data_controls[i];
        if (!native_riu_access(control, NB_MPR_RIU_CTRL0, 0, 0, &value)) {
            experiment_error=1; goto no_changes;
        }
        saved_ctrl0[i]=value & NB_MPR_RIU_CTRL0_WRITABLE;
        if (!native_riu_access(control, NB_MPR_RIU_RL_DLY_RNK0,
                               0, 0, &value)) {
            experiment_error=1; goto no_changes;
        }
        saved_rl[i]=current_rl[i]=value & NB_MPR_RIU_RL_WRITABLE;
        controls_saved++;
    }

#ifdef CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN
    /* The broad RIU grid remains on the original fixed-valid path. The
     * scheduled-return cohort below is an additional, separately labelled
     * experiment and always restores these two software controls. */
    scheduled_controls_modified=1;
    ddrphy_scheduled_fifo_mode_write(0);
    cdelay(100);
    if (ddrphy_scheduled_fifo_mode_read()!=0u) experiment_error=1;
#endif

    printf("DDR4 MPR RIU gate cohort: gate_delay=0..4 width=1..2 coarse=0..15; four warmups then 16 measured MPR reads/config; controls:");
    for (unsigned i=0; i<8u; ++i) printf(" %u", usnative_data_controls[i]);
    printf("\n");
    for (unsigned i=0; i<8u; ++i)
        printf("DDR4 MPR RIU gate saved: control=%u CTRL0=%04x RL0=%04x\n",
            usnative_data_controls[i],saved_ctrl0[i],saved_rl[i]);

    ddrphy_gate_override_write(0xfu);
    software_gate_modified=1;
    cdelay(100);

    for (unsigned delay=0; delay<=4u && !experiment_error; ++delay) {
        for (unsigned width=1; width<=2u && !experiment_error; ++width) {
            for (unsigned coarse=0; coarse<=15u && !experiment_error; ++coarse) {
                for (unsigned lane=0; lane<4u; ++lane) {
                    nb_mpr_riu_gate_set_lane(lane, delay, width);
                }
                cdelay(100);

#ifdef NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
                if (!nb_mpr_riu_prepare_candidate_epoch("fixed_before_warmup",
                        delay,width,coarse)) {
                    experiment_error=1;
                    break;
                }
#endif

                for (unsigned i=0; i<8u; ++i) {
                    const unsigned control=usnative_data_controls[i];
                    const unsigned candidate=(current_rl[i] & 0x01ffu) | (coarse << 9);
                    unsigned actual;
                    if (!native_riu_access(control, NB_MPR_RIU_RL_DLY_RNK0,
                                           candidate, 1, &actual) ||
                        !native_riu_access(control, NB_MPR_RIU_RL_DLY_RNK0,
                                           0, 0, &actual) ||
                        (actual & NB_MPR_RIU_RL_WRITABLE) != candidate) {
                        experiment_error=1; break;
                    }
                    current_rl[i]=candidate;
                }
                if (experiment_error) break;

                for (unsigned warmup=0; warmup<warmup_reads; ++warmup) {
                    uint64_t words[4];
                    nb_mpr_lane_sweep_read(words);
                    cdelay(100);
                }

#ifdef NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
                if (!nb_mpr_riu_prepare_candidate_epoch("fixed_before_measured",
                        delay,width,coarse)) {
                    experiment_error=1;
                    break;
                }
#endif

                for (unsigned read=0; read<measured_reads; ++read) {
                    unsigned edge_before[4], fifo_before[4];
                    uint64_t words[4];
                    struct nb_mpr_riu_gate_sample *sample=&samples[read];
                    for (unsigned i=0; i<8u; ++i) {
                        if (!nb_mpr_riu_gate_clear(usnative_data_controls[i])) {
                            experiment_error=1; break;
                        }
                    }
                    if (experiment_error) break;
                    for (unsigned lane=0; lane<4u; ++lane) {
                        edge_before[lane]=nb_mpr_dqs_wrclk_edges(lane);
                        fifo_before[lane]=nb_mpr_riu_gate_fifo_reads(lane);
                    }
                    nb_mpr_lane_sweep_read(words);
                    cdelay(100);

                    sample->gt_status_mask=0;
                    sample->edge_mask=sample->pop_mask=sample->exact_mask=0;
                    sample->fresh_exact_mask=sample->extra_pop_mask=0;
                    for (unsigned phase=0; phase<4u; ++phase)
                        sample->words[phase]=words[phase];
                    if (!nb_mpr_riu_gate_read_status(&sample->gt_status_mask)) {
                        experiment_error=1; break;
                    }
                    for (unsigned lane=0; lane<4u; ++lane) {
                        sample->edge_delta[lane]=
                            nb_mpr_dqs_wrclk_edges(lane)-edge_before[lane];
                        sample->fifo_delta[lane]=
                            nb_mpr_riu_gate_fifo_reads(lane)-fifo_before[lane];
                        const unsigned exact=(unsigned)nb_mpr_lane_word_matches(words,lane);
                        sample->edge_mask |= (sample->edge_delta[lane]==1u) << lane;
                        sample->pop_mask |= (sample->fifo_delta[lane]==1u) << lane;
                        sample->exact_mask |= exact << lane;
                        sample->fresh_exact_mask |= (exact &&
                            sample->edge_delta[lane]==1u &&
                            sample->fifo_delta[lane]==1u) << lane;
                        sample->extra_pop_mask |=
                            (sample->fifo_delta[lane]>1u) << lane;
                    }
                }
                if (experiment_error) break;

                /* One bounded 16-row buffer is reported only after this
                 * identical-read cohort, with no UART traffic between READs. */
                for (unsigned read=0; read<measured_reads; ++read) {
                    const struct nb_mpr_riu_gate_sample *s=&samples[read];
                    printf("DDR4 MPR RIU gate row: delay=%u width=%u coarse=%u read=%u GT=%02x edge=%x pop=%x exact=%x fresh=%x extra=%x edge_delta=%u,%u,%u,%u fifo_delta=%u,%u,%u,%u words=%08x%08x,%08x%08x,%08x%08x,%08x%08x\n",
                        delay,width,coarse,read,s->gt_status_mask,s->edge_mask,
                        s->pop_mask,s->exact_mask,s->fresh_exact_mask,
                        s->extra_pop_mask,s->edge_delta[0],s->edge_delta[1],
                        s->edge_delta[2],s->edge_delta[3],s->fifo_delta[0],
                        s->fifo_delta[1],s->fifo_delta[2],s->fifo_delta[3],
                        (unsigned)(s->words[0]>>32),(unsigned)s->words[0],
                        (unsigned)(s->words[1]>>32),(unsigned)s->words[1],
                        (unsigned)(s->words[2]>>32),(unsigned)s->words[2],
                        (unsigned)(s->words[3]>>32),(unsigned)s->words[3]);
                }
                /* Distinct MPR locations tag freshness after the repeating
                 * MPR0 cohort, while keeping UART out of those 12 reads. */
#ifdef NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
                if (!nb_mpr_riu_prepare_candidate_epoch("pattern_before_measured",
                        delay,width,coarse)) {
                    experiment_error=1;
                    break;
                }
#endif
                nb_mpr_pattern_probe(delay,width,coarse);
#if defined(CSR_DDRPHY_RX_TRACE_WORD0_ADDR) && \
    defined(CSR_DDRPHY_RX_TRACE_DONE_ADDR) && \
    defined(CSR_DDRPHY_RX_TRACE_INDEX_ADDR) && \
    defined(CSR_DDRPHY_RX_TRACE_WIDTH_ADDR) && \
    defined(CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR)
                if (delay==1u && width==1u && coarse==4u)
                    nb_mpr_riu_location_trace();
#endif
                printf("DDR4 MPR RIU gate cohort end: delay=%u width=%u coarse=%u; GT bit i maps controls in printed mapping order\n",
                    delay,width,coarse);
            }
        }
    }

#ifdef CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN
    /* Focused scheduled-return cohort at the hardware MPR0 candidates. Keep
     * the eight per-control coarse settings matched and use the existing
     * twelve-read distinct-location probe for each setting. The leading
     * marker distinguishes scheduler delay from the row's gate-delay field. */
    if (!experiment_error) {
        static const unsigned coarse_candidates[3]={3u,4u,5u};
        for (unsigned pop_delay=4u; pop_delay<=16u && !experiment_error; ++pop_delay) {
            ddrphy_scheduled_fifo_delay_write(pop_delay);
            if (ddrphy_scheduled_fifo_delay_read()!=pop_delay) {
                experiment_error=1;
                break;
            }
            for (unsigned candidate=0; candidate<3u && !experiment_error; ++candidate) {
                const unsigned coarse=coarse_candidates[candidate];
                unsigned epoch_empty=0, epoch_stable=0;
                if (!nb_mpr_riu_prepare_dq_epoch(&epoch_empty,&epoch_stable)) {
                    printf("DDR4 MPR scheduled-return epoch ERROR: coarse=%u fifo_delay=%u dq_empty=%08x stable=%u required=3\n",
                        coarse,pop_delay,epoch_empty,epoch_stable);
                    experiment_error=1;
                    break;
                }
                for (unsigned lane=0; lane<4u; ++lane)
                    nb_mpr_riu_gate_set_lane(lane,1u,1u);
                cdelay(100);
                for (unsigned i=0; i<8u; ++i) {
                    const unsigned control=usnative_data_controls[i];
                    const unsigned candidate_rl=(saved_rl[i]&0x01ffu) | (coarse<<9);
                    unsigned actual;
                    if (!native_riu_access(control,NB_MPR_RIU_RL_DLY_RNK0,
                                           candidate_rl,1,&actual) ||
                        !native_riu_access(control,NB_MPR_RIU_RL_DLY_RNK0,
                                           0,0,&actual) ||
                        (actual&NB_MPR_RIU_RL_WRITABLE)!=candidate_rl) {
                        experiment_error=1;
                        break;
                    }
                    current_rl[i]=candidate_rl;
                }
                if (experiment_error) break;
                ddrphy_scheduled_fifo_mode_write(1);
                cdelay(100);
                if (ddrphy_scheduled_fifo_mode_read()!=1u) {
                    experiment_error=1;
                    break;
                }
                /* Let the scheduled pop path settle after a candidate change
                 * without serial output inside the measured pattern cohort. */
                const unsigned warmup_underflows_before=ddrphy_scheduled_fifo_underflows_read();
                const unsigned warmup_overlaps_before=ddrphy_scheduled_fifo_overlaps_read();
                const unsigned warmup_missing_before=
                    ddrphy_scheduled_fifo_missing_last_read() & 0xfu;
                for (unsigned warmup=0; warmup<4u; ++warmup) {
                    uint64_t words[4];
                    nb_mpr_lane_sweep_read(words);
                    cdelay(100);
                }
                const unsigned warmup_underflow_delta=
                    ddrphy_scheduled_fifo_underflows_read()-warmup_underflows_before;
                const unsigned warmup_overlap_delta=
                    ddrphy_scheduled_fifo_overlaps_read()-warmup_overlaps_before;
                const unsigned sticky_missing=ddrphy_scheduled_fifo_missing_last_read() & 0xfu;
                const unsigned warmup_missing=warmup_underflow_delta ?
                    (sticky_missing & ~warmup_missing_before) : 0u;

                /* A warmup can leave a late return behind after a missed
                 * request. Drain and prove a new all-DQ-empty epoch before
                 * taking measured counter baselines. */
                unsigned measured_epoch_empty=0, measured_epoch_stable=0;
                if (!nb_mpr_riu_prepare_dq_epoch(&measured_epoch_empty,
                                                 &measured_epoch_stable)) {
                    printf("DDR4 MPR scheduled-return warmup epoch ERROR: coarse=%u fifo_delay=%u warmup_underflow_delta=%u warmup_overlap_delta=%u dq_empty=%08x stable=%u sticky_missing=%x\n",
                        coarse,pop_delay,warmup_underflow_delta,warmup_overlap_delta,
                        measured_epoch_empty,measured_epoch_stable,sticky_missing);
                    experiment_error=1;
                    break;
                }
                ddrphy_scheduled_fifo_mode_write(1);
                cdelay(100);
                if (ddrphy_scheduled_fifo_mode_read()!=1u) {
                    experiment_error=1;
                    break;
                }
                const unsigned underflows_before=ddrphy_scheduled_fifo_underflows_read();
                const unsigned overlaps_before=ddrphy_scheduled_fifo_overlaps_read();
                const unsigned measured_missing_before=
                    ddrphy_scheduled_fifo_missing_last_read() & 0xfu;
                printf("DDR4 MPR scheduled-return begin: fifo_mode=1 gate_delay=1 gate_width=1 coarse=%u fifo_delay=%u epoch_empty=%08x epoch_stable=%u warmup_underflow_delta=%u warmup_overlap_delta=%u warmup_missing_new=%x sticky_missing_after_warmup=%x\n",
                    coarse,pop_delay,measured_epoch_empty,measured_epoch_stable,
                    warmup_underflow_delta,warmup_overlap_delta,warmup_missing,sticky_missing);
                nb_mpr_pattern_probe(1u,1u,coarse);
                const unsigned measured_underflow_delta=
                    ddrphy_scheduled_fifo_underflows_read()-underflows_before;
                const unsigned measured_overlap_delta=
                    ddrphy_scheduled_fifo_overlaps_read()-overlaps_before;
                const unsigned measured_sticky_missing=
                    ddrphy_scheduled_fifo_missing_last_read() & 0xfu;
                const unsigned measured_missing=measured_underflow_delta ?
                    (measured_sticky_missing & ~measured_missing_before) : 0u;
                printf("DDR4 MPR scheduled-return summary: coarse=%u fifo_delay=%u underflow_delta=%u overlap_delta=%u missing_last=%x measured_missing=%x measured_missing_before=%x warmup_underflow_delta=%u warmup_overlap_delta=%u warmup_missing=%x sticky_missing_latched=%x epoch_empty=%08x epoch_stable=%u\n",
                    coarse,pop_delay,measured_underflow_delta,
                    measured_overlap_delta,measured_sticky_missing,
                    measured_missing,measured_missing_before,
                    warmup_underflow_delta,warmup_overlap_delta,
                    warmup_missing,measured_sticky_missing,
                    measured_epoch_empty,measured_epoch_stable);
            }
        }
        ddrphy_scheduled_fifo_mode_write(0);
        cdelay(100);
        if (ddrphy_scheduled_fifo_mode_read()!=0u) experiment_error=1;
    }
#endif

    /* Contrast the unreset changing-pattern rows with a fresh CLR_GATE pulse
     * on all mapped data controls immediately before every location READ.
     * The prepared probe buffers all twelve reads and prints only afterwards. */
    if (!experiment_error) {
#ifdef CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN
        ddrphy_scheduled_fifo_mode_write(0);
        cdelay(100);
        if (ddrphy_scheduled_fifo_mode_read()!=0u) experiment_error=1;
#endif
        static const unsigned coarse_candidates[3]={3u,4u,5u};
        for (unsigned candidate=0; candidate<3u && !experiment_error; ++candidate) {
            const unsigned coarse=coarse_candidates[candidate];
            for (unsigned lane=0; lane<4u; ++lane)
                nb_mpr_riu_gate_set_lane(lane,1u,1u);
            cdelay(100);
            for (unsigned index=0; index<8u; ++index) {
                const unsigned control=usnative_data_controls[index];
                const unsigned candidate_rl=(saved_rl[index]&0x01ffu) | (coarse<<9);
                unsigned actual;
                if (!native_riu_access(control,NB_MPR_RIU_RL_DLY_RNK0,
                                       candidate_rl,1,&actual) ||
                    !native_riu_access(control,NB_MPR_RIU_RL_DLY_RNK0,
                                       0,0,&actual) ||
                    (actual&NB_MPR_RIU_RL_WRITABLE)!=candidate_rl) {
                    experiment_error=1;
                    break;
                }
                current_rl[index]=candidate_rl;
            }
            if (experiment_error) break;
#ifdef NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
            if (!nb_mpr_riu_prepare_candidate_epoch("gateclear_before_warmup",
                    1u,1u,coarse)) {
                experiment_error=1;
                break;
            }
#endif
            for (unsigned warmup=0; warmup<4u; ++warmup) {
                uint64_t words[4];
                nb_mpr_lane_sweep_read(words);
                cdelay(100);
            }
#ifdef NB_MPR_RIU_HAS_FIFO_EPOCH_FLUSH
            if (!nb_mpr_riu_prepare_candidate_epoch("gateclear_before_measured",
                    1u,1u,coarse)) {
                experiment_error=1;
                break;
            }
#endif
            struct nb_mpr_riu_gate_clear_context context={
                0u,0u,0xffffffffu,0xffffffffu};
            unsigned completed_reads=0;
            printf("DDR4 MPR gate-clear pattern begin: gate_delay=1 gate_width=1 coarse=%u clear_each_read=1\n",
                coarse);
            const int complete=nb_mpr_pattern_probe_prepared(1u,1u,coarse,
                nb_mpr_riu_gate_prepare_pattern_read,&context,&completed_reads);
            printf("DDR4 MPR gate-clear pattern summary: coarse=%u reads=%u clear_attempts=%u clear_success=%u status=%s failed_read=%u failed_control=%u\n",
                coarse,completed_reads,context.attempts,context.successful,
                complete ? "complete" : "clear_error",
                context.failed_read,context.failed_control);
            if (!complete) experiment_error=1;
        }
    }

#if defined(NB_MPR_RIU_HAS_LANE_PHASE_CSRS) && \
    defined(CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN)
    if (!experiment_error && !nb_mpr_riu_lane_phase_cohort())
        restore_error=1;
#endif

    /* Restore lane scheduler first; preserve global register as well as the
     * per-lane override values, since the override selects which is active. */
    if (software_gate_modified) {
        ddrphy_gate_delay_write(saved_global_delay);
        for (unsigned lane=0; lane<4u; ++lane)
            nb_mpr_riu_gate_set_lane(lane,saved_delay[lane],saved_width[lane]);
        cdelay(100);
        ddrphy_gate_override_write(saved_override);
        cdelay(100);
        if (ddrphy_gate_delay_read() != saved_global_delay ||
            ddrphy_gate_override_read() != saved_override ||
            ddrphy_gate_delay0_read() != saved_delay[0] ||
            ddrphy_gate_delay1_read() != saved_delay[1] ||
            ddrphy_gate_delay2_read() != saved_delay[2] ||
            ddrphy_gate_delay3_read() != saved_delay[3] ||
            ddrphy_gate_width0_read() != saved_width[0] ||
            ddrphy_gate_width1_read() != saved_width[1] ||
            ddrphy_gate_width2_read() != saved_width[2] ||
            ddrphy_gate_width3_read() != saved_width[3])
            restore_error=1;
    }
#ifdef CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN
    if (scheduled_controls_modified) {
        /* Keep the scheduler quiescent while restoring its delay; only then
         * put back the caller's original mode and verify both CSRs. */
        ddrphy_scheduled_fifo_mode_write(0);
        cdelay(100);
        ddrphy_scheduled_fifo_delay_write(saved_scheduled_delay);
        cdelay(100);
        if (ddrphy_scheduled_fifo_delay_read()!=saved_scheduled_delay)
            restore_error=1;
        ddrphy_scheduled_fifo_mode_write(saved_scheduled_mode);
        cdelay(100);
        if (ddrphy_scheduled_fifo_mode_read()!=saved_scheduled_mode)
            restore_error=1;
    }
#endif
    if (controls_saved == 8u) {
        for (unsigned i=0; i<8u; ++i) {
            unsigned actual;
            if (!native_riu_access(usnative_data_controls[i],
                    NB_MPR_RIU_RL_DLY_RNK0, saved_rl[i], 1, &actual) ||
                !native_riu_access(usnative_data_controls[i],
                    NB_MPR_RIU_RL_DLY_RNK0, 0, 0, &actual) ||
                (actual & NB_MPR_RIU_RL_WRITABLE) != saved_rl[i]) {
                restore_error=1;
            }
            if (!native_riu_access(usnative_data_controls[i], NB_MPR_RIU_CTRL0,
                    saved_ctrl0[i], 1, &actual) ||
                !native_riu_access(usnative_data_controls[i], NB_MPR_RIU_CTRL0,
                    0, 0, &actual) ||
                (actual & NB_MPR_RIU_CTRL0_WRITABLE) != saved_ctrl0[i]) {
                restore_error=1;
            }
        }
    }

    if (restore_error) {
        /* Restoration could not be proven; stop issuing DRAM commands. */
        ddrphy_rst_write(1);
        sdram_dfii_control_write(0);
        printf("DDR4 MPR RIU gate cohort RESTORE FAILURE; PHY reset asserted and DFII controls quiesced\n");
        return 0;
    }
#ifdef CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN
    printf("DDR4 MPR RIU gate cohort %s; restored RL_DLY_RNK, CTRL0, gate delays/widths, override=%x and scheduled mode=%u delay=%u\n",
        experiment_error ? "aborted with experiment error" : "complete",
        saved_override,saved_scheduled_mode,saved_scheduled_delay);
#else
    printf("DDR4 MPR RIU gate cohort %s; restored RL_DLY_RNK, CTRL0, gate delays/widths and override=%x\n",
        experiment_error ? "aborted with experiment error" : "complete",
        saved_override);
#endif
    return 1;

no_changes:
    /* The incomplete save pass is read-only, so no restoration is required. */
    printf("DDR4 MPR RIU gate cohort aborted before changes: RIU save error\n");
    return 1;
}

#endif /* CONFIG_SDRAM_USNATIVE_MPR_RIU_GATE_COHORT */
#endif /* USNATIVE_NATIVE_MPR_RIU_GATE_COHORT_H */
