/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Diagnostic-only DDR4 MPR read for AES-KU40 native x32 DDR4-1600.
 *
 * Caller contract: DFII software owns the interface, init_sequence() has
 * completed, the configured 1x-refresh MR3 value is the known EDY4016A
 * default (zero), and the PHY is at the bootstrap CK/RX/TX/gate settings.
 * The routine only reads MPR0; it never writes the array or MPR contents.
 */
#ifndef USNATIVE_NATIVE_MPR_PROBE_H
#define USNATIVE_NATIVE_MPR_PROBE_H

#if defined(CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC)

#if !defined(SDRAM_PHY_DDR4) || SDRAM_PHY_DATABITS != 32 || \
    SDRAM_PHY_DFI_DATABITS != 64 || SDRAM_PHY_PHASES != 4 || \
    CONFIG_CLOCK_FREQUENCY != 200000000 || SDRAM_PHY_CL != 11 || \
    SDRAM_PHY_CWL != 9
#error "DDR4 MPR diagnostic is restricted to AES-KU40 native x32 DDR4-1600 1:4"
#endif

#if defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TRACE) && \
    !defined(CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC)
#error "The MPR RX trace requires the DDR4 MPR diagnostic"
#endif

#if defined(CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP) && \
    !defined(CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC)
#error "The MPR lane sweep requires the DFI diagnostic"
#endif
#if defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TAP_SCAN) && \
    (!defined(CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP) || \
     !defined(CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC))
#error "The MPR RX tap scan requires the MPR lane sweep and DFI diagnostic"
#endif
#if defined(CONFIG_SDRAM_USNATIVE_MPR_2D_SCAN) && \
    (!defined(CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP) || \
     !defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TAP_SCAN) || \
     !defined(CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC))
#error "The MPR 2D scan requires the MPR lane sweep, RX scan entry point and DFI diagnostic"
#endif
#if defined(CONFIG_SDRAM_USNATIVE_MPR_CADENCE_PERTURB) && \
    (!defined(CONFIG_SDRAM_USNATIVE_MPR_2D_SCAN) || \
     !defined(CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR))
#error "The MPR cadence perturbation requires the MPR 2D scan and DQS write-clock monitor"
#endif

#define NB_MPR_NORMAL_MR3 0x0000u /* EDY4016A default 1x fine-refresh mode */
#define NB_MPR_ENABLE_MR3 0x0004u /* MR3 A2=1; page 0, serial read format */
#define NB_MPR_READ_COUNT 9u
#define NB_MPR_CHECK_FROM 5u
#define NB_MPR_GATE_FIRST 0u
#define NB_MPR_GATE_LAST 5u
#ifndef CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET
#define CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET 0
#endif
#if CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET < -2 || \
    CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET > 2
#error "MPR lane-0 gate offset must be in [-2, 2]"
#endif

#if defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TRACE)
#define NB_MPR_TRACE_LANE0_OFFSET CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET
#else
#define NB_MPR_TRACE_LANE0_OFFSET 0
#endif

static int nb_mpr_word_matches(uint64_t word)
{
    /* MPR page 0, location 0 is the repeated 01010101 pattern. The native
     * 1:4 receiver places the two UIs for each DQ bit at positions bit and
     * bit+32 in a 64-bit DFI word (ddrphy.py RX mapping). Adjacent MPR UIs
     * are complements, independent of which UI starts the captured pair. */
    return ((uint32_t)word ^ (uint32_t)(word >> 32)) == 0xffffffffu;
}

/* Check the complete four-phase MPR0 BL8 signature for one physical byte.
 * Each DQ's low-half sample must keep the same polarity across phases, while
 * its paired high-half sample is the opposite UI. Either initial polarity is
 * valid because the capture may begin on either edge of the alternating
 * pattern. */
static int nb_mpr_lane_word_matches(const uint64_t words[4], unsigned lane)
{
    if (lane >= 4) return 0;
    for (unsigned dq=lane*8; dq<lane*8+8; ++dq) {
        const unsigned first=(unsigned)((words[0] >> dq) & 1u);
        for (unsigned phase=0; phase<4; ++phase) {
            const unsigned low=(unsigned)((words[phase] >> dq) & 1u);
            const unsigned high=(unsigned)((words[phase] >> (dq+32)) & 1u);
            if (low != first || high == first) return 0;
        }
    }
    return 1;
}

/* Pack one byte's low/high DDR samples from all four DFI phases. */
static uint64_t nb_mpr_lane_samples(const uint64_t words[4], unsigned lane)
{
    uint64_t packed=0;
    for (unsigned phase=0; phase<4; ++phase) {
        packed |= ((words[phase] >> (8*lane)) & 0xffu) << (16*phase);
        packed |= ((words[phase] >> (32+8*lane)) & 0xffu) << (16*phase+8);
    }
    return packed;
}

#if defined(CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP) || \
    defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TAP_SCAN)
static unsigned nb_mpr_lane_fifo_reads(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_fifo_reads0_read();
    case 1: return ddrphy_fifo_reads1_read();
    case 2: return ddrphy_fifo_reads2_read();
    default: return ddrphy_fifo_reads3_read();
    }
}
#endif

#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
#ifndef CSR_DDRPHY_DQS_WRCLK_EDGES0_ADDR
#error "The DQS write-clock monitor requires its per-byte read-only CSRs"
#endif
static unsigned nb_mpr_dqs_wrclk_edges(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_dqs_wrclk_edges0_read();
    case 1: return ddrphy_dqs_wrclk_edges1_read();
    case 2: return ddrphy_dqs_wrclk_edges2_read();
    default: return ddrphy_dqs_wrclk_edges3_read();
    }
}
#endif

static void nb_mpr_mrs(unsigned mr3)
{
    sdram_dfii_pi0_address_write(mr3);
    sdram_dfii_pi0_baddress_write(3); /* Mode Register 3 */
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
    cdelay(100); /* >= tMRD/tMOD at the 200 MHz controller clock */
}

#if defined(CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP)
#define NB_MPR_LANE_GATE_FIRST 1u
#define NB_MPR_LANE_GATE_LAST 3u

static void nb_mpr_lane_location_read(uint64_t words[4], unsigned location)
{
    sdram_dfii_pi0_address_write(0);
    /* DDR4 MPR page locations are selected by BA[1:0], not address bits. */
    sdram_dfii_pi0_baddress_write(location & 3u);
    command_p0(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
    cdelay(100);
    words[0]=sdram_dfii_pi0_rddata_read();
    words[1]=sdram_dfii_pi1_rddata_read();
    words[2]=sdram_dfii_pi2_rddata_read();
    words[3]=sdram_dfii_pi3_rddata_read();
}

static void nb_mpr_lane_sweep_read(uint64_t words[4])
{
    nb_mpr_lane_location_read(words, 0);
}

static unsigned nb_mpr_valid_count(void)
{
    return native_dfi_activity_rddata_valid_cycles_read();
}

#if defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TAP_SCAN)
static void nb_mpr_lane_gate_width(unsigned lane, unsigned gate, unsigned width)
{
    switch (lane) {
    case 0: ddrphy_gate_delay0_write(gate); ddrphy_gate_width0_write(width); break;
    case 1: ddrphy_gate_delay1_write(gate); ddrphy_gate_width1_write(width); break;
    case 2: ddrphy_gate_delay2_write(gate); ddrphy_gate_width2_write(width); break;
    default: ddrphy_gate_delay3_write(gate); ddrphy_gate_width3_write(width); break;
    }
}

#if defined(CSR_DDRPHY_TAP_RX_RST_ADDR) && defined(CSR_DDRPHY_TAP_RX_INC_ADDR)
/* This diagnostic changes only the selected DQS slice, not the lane's DQ. */
static int nb_mpr_strobe_rx_tap(unsigned lane, unsigned tap)
{
    if (lane>=4 || tap>=SDRAM_PHY_DELAYS) return 0;
    ddrphy_tap_select_write(usnative_dqs_taps[lane]);
    cdelay(100);
    if (!native_tap_wait()) return 0;
    ddrphy_tap_rx_rst_write(1);
    cdelay(100);
    for (unsigned i=0; i<tap; ++i) {
        ddrphy_tap_rx_inc_write(1);
        cdelay(100);
    }
    return native_tap_wait() && ddrphy_tap_rx_count_read()==tap;
}
#endif

/* Try a single MPR0 BL8 read and report the three independent checks for a
 * selected byte. Counter deltas are measured around the read itself. */
static unsigned nb_mpr_2d_candidate(unsigned lane, unsigned *fifo_ok,
    unsigned *valid_ok, unsigned *exact_ok, uint64_t *lane_word)
{
    uint64_t words[4];
    unsigned before=nb_mpr_lane_fifo_reads(lane);
    unsigned valid_before=nb_mpr_valid_count();
    nb_mpr_lane_sweep_read(words);
    if (lane_word) *lane_word=nb_mpr_lane_samples(words,lane);
    *fifo_ok=(nb_mpr_lane_fifo_reads(lane)-before)==1u;
    *valid_ok=(nb_mpr_valid_count()-valid_before)==1u;
    *exact_ok=(unsigned)nb_mpr_lane_word_matches(words,lane);
    return *fifo_ok && *valid_ok && *exact_ok;
}

#if defined(CSR_DDRPHY_RX_TRACE_WORD0_ADDR) && \
    defined(CSR_DDRPHY_RX_TRACE_DONE_ADDR) && \
    defined(CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR)
/* Dump a stopped trace while no DDR command is in flight. The generated
 * RX trace CSR words are contiguous 32-bit slots; the host decoder checks
 * their layout against the generated core and PHY metadata. */
static void nb_mpr_print_rx_trace(unsigned kind, unsigned read)
{
    unsigned words=ddrphy_rx_trace_width_read()/32;
    if (!ddrphy_rx_trace_done_read() || !words || words>64) {
        printf("DDR4 MPR paired trace unavailable: kind=%u read=%u done=%u words=%u\n",
            kind,read,ddrphy_rx_trace_done_read(),words);
        return;
    }
    for (unsigned sample=0; sample<64; ++sample) {
        ddrphy_rx_trace_index_write(sample);
        cdelay(100);
        printf("DDR4 MPR paired trace: kind=%u read=%u sample=%u data=",
            kind,read,sample);
        for (unsigned word=0; word<words; ++word)
            printf("%s%08x",word ? "," : "",
                (unsigned)csr_read_simple(CSR_DDRPHY_RX_TRACE_WORD0_ADDR+4*word));
        printf("\n");
    }
}

/* Read settled primitive RX counts immediately after the selected read.
 * Selector writes only change the status mux; no delay reset/increment is
 * issued. Restore the previous selector before the next memory command. */
static void nb_mpr_print_rx_taps(unsigned kind, unsigned read)
{
    const unsigned saved=ddrphy_tap_select_read();
    const unsigned lanes[2]={0,2};
    for (unsigned item=0; item<2; ++item) {
        const unsigned lane=lanes[item];
        unsigned selectors[10];
        selectors[0]=usnative_dqs_taps[lane];
        selectors[1]=usnative_dm_taps[lane];
        for (unsigned dq=0; dq<8; ++dq)
            selectors[2+dq]=usnative_dq_taps[8*lane+dq];
        printf("DDR4 MPR paired RX taps: kind=%u read=%u lane=%u values=",
            kind,read,lane);
        for (unsigned index=0; index<10; ++index) {
            ddrphy_tap_select_write(selectors[index]);
            if (!native_tap_wait()) {
                printf("%stimeout",index ? "," : "");
                break;
            }
            printf("%s%u",index ? "," : "",ddrphy_tap_rx_count_read());
        }
        printf("\n");
    }
    ddrphy_tap_select_write(saved);
    if (!native_tap_wait())
        printf("DDR4 MPR paired RX taps restore timeout: kind=%u read=%u\n",kind,read);
}

/* Compare one missing and one captured MPR return at the same boot gate,
 * before software edits delay taps, gate phase, or VTC. */
static void nb_mpr_paired_rx_trace(void)
{
    unsigned found_missing=0,found_captured=0;
    for (unsigned read=0; read<16 && !(found_missing && found_captured); ++read) {
        uint64_t words[4];
        ddrphy_debug_clear_write(1);
        ddrphy_rx_trace_arm_write(1);
        cdelay(100);
        unsigned edge_before=nb_mpr_dqs_wrclk_edges(0);
        unsigned fifo_before=nb_mpr_lane_fifo_reads(0);
        unsigned valid_before=nb_mpr_valid_count();
        nb_mpr_lane_sweep_read(words);
        cdelay(100);
        unsigned edge_delta=nb_mpr_dqs_wrclk_edges(0)-edge_before;
        unsigned fifo_delta=nb_mpr_lane_fifo_reads(0)-fifo_before;
        unsigned valid_delta=nb_mpr_valid_count()-valid_before;
        unsigned kind=0;
        if (!edge_delta && !fifo_delta && !found_missing) {
            kind=1; found_missing=1;
        } else if (edge_delta==1u && fifo_delta==1u && !found_captured) {
            kind=2; found_captured=1;
        }
        printf("DDR4 MPR paired read: index=%u edges=%u fifo=%u valid=%u exact=%u kind=%u\n",
            read,edge_delta,fifo_delta,valid_delta,
            (unsigned)nb_mpr_lane_word_matches(words,0),kind);
        if (kind) {
            nb_mpr_print_rx_taps(kind,read);
            nb_mpr_print_rx_trace(kind,read);
        }
    }
    printf("DDR4 MPR paired trace summary: missing=%u captured=%u\n",
        found_missing,found_captured);
}

/* Keep the MPR request cadence uninterrupted while observing every byte.
 * A one-hot return pattern would otherwise be hidden by lane-by-lane loops. */
static void nb_mpr_all_lane_cadence(void)
{
    struct cadence_sample {
        unsigned edge_mask, pop_mask, exact_mask, valid_delta;
        unsigned edge_delta[4], fifo_delta[4];
    } samples[32];
    const unsigned dly_before=ddrphy_dly_rdy_read();
    const unsigned vtc_before=ddrphy_vtc_rdy_read();
    const unsigned ready_before=ddrphy_ready_read();
    const unsigned debug_before=ddrphy_debug_read();
    const unsigned faults_before=ddrphy_faults_read();
    for (unsigned read=0; read<32; ++read) {
        uint64_t words[4];
        unsigned edge_before[4],fifo_before[4];
        for (unsigned lane=0; lane<4; ++lane) {
            edge_before[lane]=nb_mpr_dqs_wrclk_edges(lane);
            fifo_before[lane]=nb_mpr_lane_fifo_reads(lane);
        }
        unsigned valid_before=nb_mpr_valid_count();
        nb_mpr_lane_sweep_read(words);
        cdelay(100);
        struct cadence_sample *sample=&samples[read];
        sample->edge_mask=sample->pop_mask=sample->exact_mask=0;
        for (unsigned lane=0; lane<4; ++lane) {
            sample->edge_delta[lane]=nb_mpr_dqs_wrclk_edges(lane)-edge_before[lane];
            sample->fifo_delta[lane]=nb_mpr_lane_fifo_reads(lane)-fifo_before[lane];
            sample->edge_mask |= (sample->edge_delta[lane]==1u)<<lane;
            sample->pop_mask |= (sample->fifo_delta[lane]==1u)<<lane;
            sample->exact_mask |= nb_mpr_lane_word_matches(words,lane)<<lane;
        }
        sample->valid_delta=nb_mpr_valid_count()-valid_before;
    }
    printf("DDR4 MPR cadence status: dly=%x,%x vtc=%x,%x ready=%u,%u debug=%08x,%08x faults=%x,%x\n",
        dly_before,ddrphy_dly_rdy_read(),vtc_before,ddrphy_vtc_rdy_read(),
        ready_before,ddrphy_ready_read(),debug_before,ddrphy_debug_read(),
        faults_before,ddrphy_faults_read());
    for (unsigned read=0; read<32; ++read) {
        const struct cadence_sample *sample=&samples[read];
        printf("DDR4 MPR all-lane cadence: read=%u edge=%x pop=%x exact=%x valid=%u edge_delta=%u,%u,%u,%u fifo_delta=%u,%u,%u,%u\n",
            read,sample->edge_mask,sample->pop_mask,sample->exact_mask,
            sample->valid_delta,
            sample->edge_delta[0],sample->edge_delta[1],sample->edge_delta[2],sample->edge_delta[3],
            sample->fifo_delta[0],sample->fifo_delta[1],sample->fifo_delta[2],sample->fifo_delta[3]);
    }
}

#if defined(CONFIG_SDRAM_USNATIVE_MPR_CADENCE_PERTURB)
/* Shift the same READ by controlled CPU-loop delays while keeping MR3, CK,
 * RX/TX taps and gate settings fixed. Buffer all measurements so UART traffic
 * cannot alter the issue cadence. Fresh exact data requires both one accepted
 * FIFO pop and the complete lane MPR signature on that same read. */
static void nb_mpr_cadence_perturb(void)
{
    static const unsigned delays[]={0,1,2,3,4,5,7,11,17};
    struct cadence_sample {
        unsigned edge_mask, pop_mask, exact_mask, fresh_exact_mask, valid_delta;
        unsigned edge_delta[4], fifo_delta[4];
    };
    struct cadence_sample samples[16];

    for (unsigned cohort=0; cohort<sizeof(delays)/sizeof(delays[0]); ++cohort) {
        for (unsigned read=0; read<16; ++read) {
            uint64_t words[4];
            unsigned edge_before[4], fifo_before[4];
            for (unsigned lane=0; lane<4; ++lane) {
                edge_before[lane]=nb_mpr_dqs_wrclk_edges(lane);
                fifo_before[lane]=nb_mpr_lane_fifo_reads(lane);
            }
            unsigned valid_before=nb_mpr_valid_count();
            cdelay(delays[cohort]);
            nb_mpr_lane_sweep_read(words);
            cdelay(100);

            struct cadence_sample *sample=&samples[read];
            sample->edge_mask=sample->pop_mask=sample->exact_mask=0;
            sample->fresh_exact_mask=0;
            for (unsigned lane=0; lane<4; ++lane) {
                sample->edge_delta[lane]=nb_mpr_dqs_wrclk_edges(lane)-edge_before[lane];
                sample->fifo_delta[lane]=nb_mpr_lane_fifo_reads(lane)-fifo_before[lane];
                sample->edge_mask |= (sample->edge_delta[lane]==1u)<<lane;
                sample->pop_mask |= (sample->fifo_delta[lane]==1u)<<lane;
                const unsigned exact=(unsigned)nb_mpr_lane_word_matches(words,lane);
                sample->exact_mask |= exact<<lane;
                sample->fresh_exact_mask |=
                    (exact && sample->fifo_delta[lane]==1u)<<lane;
            }
            sample->valid_delta=nb_mpr_valid_count()-valid_before;
        }
        /* UART output follows this complete 16-read cohort. The next delay
         * cohort starts only after reporting has finished. */
        printf("DDR4 MPR cadence perturb cohort: delay=%u reads=16\n",delays[cohort]);
        for (unsigned read=0; read<16; ++read) {
            const struct cadence_sample *sample=&samples[read];
            printf("DDR4 MPR cadence perturb: delay=%u read=%u edge=%x pop=%x exact=%x fresh_exact=%x valid=%u edge_delta=%u,%u,%u,%u fifo_delta=%u,%u,%u,%u\n",
                delays[cohort],read,sample->edge_mask,sample->pop_mask,
                sample->exact_mask,sample->fresh_exact_mask,sample->valid_delta,
                sample->edge_delta[0],sample->edge_delta[1],sample->edge_delta[2],sample->edge_delta[3],
                sample->fifo_delta[0],sample->fifo_delta[1],sample->fifo_delta[2],sample->fifo_delta[3]);
        }
    }
}
#endif
#endif

#if defined(CONFIG_SDRAM_USNATIVE_MRS_COMMAND_TRACE)
#if !defined(CSR_DDRPHY_RX_TRACE_MRS_MODE_ADDR) || \
    !defined(CSR_DDRPHY_RX_TRACE_WORD0_ADDR)
#error "The MRS command trace requires the extended native RX trace CSRs"
#endif
static void nb_mpr_print_mrs_trace(const char *event)
{
    unsigned words=ddrphy_rx_trace_width_read()/32;
    if (!ddrphy_rx_trace_done_read() || !words || words>64) {
        printf("DDR4 MPR MRS trace unavailable: event=%s done=%u words=%u\n",
            event,ddrphy_rx_trace_done_read(),words);
        return;
    }
    for (unsigned sample=0; sample<64; ++sample) {
        ddrphy_rx_trace_index_write(sample);
        cdelay(100);
        printf("DDR4 MPR MRS trace: event=%s sample=%u data=",event,sample);
        for (unsigned word=0; word<words; ++word)
            printf("%s%08x",word ? "," : "",
                (unsigned)csr_read_simple(CSR_DDRPHY_RX_TRACE_WORD0_ADDR+4*word));
        printf("\n");
    }
}

static void nb_mpr_arm_mrs_trace(void)
{
    ddrphy_rx_trace_mrs_mode_write(1);
    cdelay(100);
    ddrphy_rx_trace_arm_write(1);
    cdelay(100);
}
#endif

#ifdef CONFIG_SDRAM_USNATIVE_MPR_GATE_COHORT
#include "native_mpr_gate_cohort.h"
#endif
#ifdef CONFIG_SDRAM_USNATIVE_MPR_FIFO_COHORT
#include "native_mpr_fifo_cohort.h"
#endif
#ifdef CONFIG_SDRAM_USNATIVE_MPR_RIU_GATE_COHORT
#include "native_mpr_pattern_probe.h"
#include "native_mpr_riu_gate_cohort.h"
#endif

#if defined(CONFIG_SDRAM_USNATIVE_MPR_2D_SCAN)
static int nb_mpr_2d_scan(void)
{
    static const unsigned gate_candidates[]={0,1,2,3,4,5,6,7};
    const unsigned gate_width=1;
    const unsigned lane0_gate_width=1;
#ifdef CSR_DDRPHY_GATE_PHASE0_ADDR
    const unsigned saved_phase0=ddrphy_gate_phase0_read();
#endif
#ifdef CSR_DDRPHY_GATE_PHASE_UPPER0_ADDR
    const unsigned saved_phase_upper0=ddrphy_gate_phase_upper0_read();
#endif
    const unsigned saved_selector=ddrphy_dly_sel_read();
    const unsigned saved_vtc=ddrphy_en_vtc_read();
    const unsigned saved_gate=ddrphy_gate_delay_read();
    const unsigned saved_override=ddrphy_gate_override_read();
    const unsigned saved_lane_gate[4]={ddrphy_gate_delay0_read(),ddrphy_gate_delay1_read(),
        ddrphy_gate_delay2_read(),ddrphy_gate_delay3_read()};
    const unsigned saved_width[4]={ddrphy_gate_width0_read(),ddrphy_gate_width1_read(),
        ddrphy_gate_width2_read(),ddrphy_gate_width3_read()};
    unsigned original_rx[4]={0,0,0,0};
    unsigned selected_gate[4],selected_rx[4];
    unsigned errors=0,restore_ok=1,mpr_enabled=0,rx_saved_mask=0;
#if defined(CSR_DDRPHY_TAP_RX_RST_ADDR) && defined(CSR_DDRPHY_TAP_RX_INC_ADDR)
    unsigned original_dqs_rx=0,dqs_rx_saved=0;
#endif

    for (unsigned lane=0; lane<4; ++lane) {
        selected_gate[lane]=saved_lane_gate[lane];
        selected_rx[lane]=0;
    }
    printf("DDR4 MPR 2D scan: gates=0..7 widths=%u,%u,%u,%u at boot RX; RX=0..128 step=8; reads/candidate=3; repeats=10\n",
        lane0_gate_width,gate_width,gate_width,gate_width);
    printf("DDR4 MPR 2D scope: per-byte FIFO counters only; per-DQ accepted-pop trace is still required before qualification\n");

    /* Save the actual boot RX tap for each byte before changing any delay. */
    for (unsigned lane=0; lane<4; ++lane) {
        ddrphy_dly_sel_write(1u<<lane);
        ddrphy_tap_select_write(usnative_dq_taps[lane*8]);
        cdelay(100);
        if (!native_tap_wait()) { ++errors; goto cleanup; }
        original_rx[lane]=ddrphy_tap_rx_count_read();
        if (original_rx[lane]>=SDRAM_PHY_DELAYS) { ++errors; goto cleanup; }
        selected_rx[lane]=original_rx[lane];
        rx_saved_mask |= 1u<<lane;
    }
    ddrphy_dly_sel_write(saved_selector);

    /* Measure the receive path before the diagnostic edits its gate,
     * phase, delay or VTC settings. Only MR3's MPR mode is changed. */
    sdram_dfii_pi0_address_write(1u<<10);
    sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
    cdelay(100);
#ifdef CONFIG_SDRAM_USNATIVE_MRS_COMMAND_TRACE
    nb_mpr_arm_mrs_trace();
#endif
    nb_mpr_mrs(NB_MPR_ENABLE_MR3);
#ifdef CONFIG_SDRAM_USNATIVE_MRS_COMMAND_TRACE
    nb_mpr_print_mrs_trace("enable");
    ddrphy_rx_trace_mrs_mode_write(0);
    cdelay(100);
#endif
    mpr_enabled=1;
    for (unsigned lane=0; lane<4; ++lane) {
        unsigned passes=0,one_edge=0,one_pop=0,valid_hits=0,exact_hits=0;
        for (unsigned read=0; read<32; ++read) {
            uint64_t words[4];
            ddrphy_debug_clear_write(1); cdelay(100);
            unsigned fifo_before=nb_mpr_lane_fifo_reads(lane);
            unsigned valid_before=nb_mpr_valid_count();
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
            unsigned edge_before=nb_mpr_dqs_wrclk_edges(lane);
#endif
            nb_mpr_lane_sweep_read(words);
            unsigned fifo_delta=nb_mpr_lane_fifo_reads(lane)-fifo_before;
            unsigned valid_delta=nb_mpr_valid_count()-valid_before;
            unsigned exact=nb_mpr_lane_word_matches(words,lane);
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
            cdelay(100);
            unsigned edge_delta=nb_mpr_dqs_wrclk_edges(lane)-edge_before;
#else
            unsigned edge_delta=0;
#endif
            one_edge+=edge_delta==1u;
            one_pop+=fifo_delta==1u;
            valid_hits+=valid_delta==1u;
            exact_hits+=exact;
            passes+=(fifo_delta==1u) && (valid_delta==1u) && exact;
            printf("DDR4 MPR 2D baseline read: lane=%u index=%u edges=%u fifo=%u valid=%u exact=%u\n",
                lane,read,edge_delta,fifo_delta,valid_delta,exact);
        }
        printf("DDR4 MPR 2D baseline: lane=%u pass=%u/32 one_edge=%u/32 one_pop=%u/32 valid=%u/32 exact=%u/32\n",
            lane,passes,one_edge,one_pop,valid_hits,exact_hits);
    }

#if defined(CSR_DDRPHY_RX_TRACE_WORD0_ADDR) && \
    defined(CSR_DDRPHY_RX_TRACE_DONE_ADDR) && \
    defined(CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR)
    nb_mpr_all_lane_cadence();
#ifdef CONFIG_SDRAM_USNATIVE_MPR_CADENCE_PERTURB
    nb_mpr_cadence_perturb();
#endif
    nb_mpr_paired_rx_trace();
#ifdef CONFIG_SDRAM_USNATIVE_MPR_GATE_COHORT
    nb_mpr_gate_cohort();
#endif
#ifdef CONFIG_SDRAM_USNATIVE_MPR_FIFO_COHORT
    nb_mpr_fifo_cohort();
#endif
#ifdef CONFIG_SDRAM_USNATIVE_MPR_RIU_GATE_COHORT
    /* Run from the bootstrap MPR state, before the 2D scan changes RX taps or
     * gate phases. The cohort restores its own RIU and gate settings. On a
     * restore failure it has asserted PHY reset and quiesced DFII, so stop
     * without issuing further DRAM commands. */
    if (!nb_mpr_riu_gate_cohort()) {
        printf("DDR4 MPR bootstrap aborted after RIU gate restore failure\n");
        return 0;
    }
#endif
#endif

#if defined(CSR_DDRPHY_GATE_PHASE0_ADDR) && \
    defined(CSR_DDRPHY_GATE_PHASE_UPPER0_ADDR) && \
    defined(CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR)
    /* A 16-read cohort at each mask phase separates a fixed modulo-four
     * DQS/FIFO cadence from phase-sensitive control gating. Keep the boot
     * whole-cycle gate, delays and VTC unchanged throughout this probe. */
    for (unsigned control=0; control<2; ++control) {
        for (unsigned phase=0; phase<4; ++phase) {
            unsigned edge_hits=0,pop_hits=0,valid_hits=0,exact_hits=0;
            if (control==0) ddrphy_gate_phase0_write(phase);
            else ddrphy_gate_phase_upper0_write(phase);
            cdelay(100);
            for (unsigned read=0; read<16; ++read) {
                uint64_t words[4];
                ddrphy_debug_clear_write(1); cdelay(100);
                unsigned edge_before=nb_mpr_dqs_wrclk_edges(0);
                unsigned fifo_before=nb_mpr_lane_fifo_reads(0);
                unsigned valid_before=nb_mpr_valid_count();
                nb_mpr_lane_sweep_read(words);
                cdelay(100);
                unsigned edge_delta=nb_mpr_dqs_wrclk_edges(0)-edge_before;
                unsigned fifo_delta=nb_mpr_lane_fifo_reads(0)-fifo_before;
                unsigned valid_delta=nb_mpr_valid_count()-valid_before;
                edge_hits |= (edge_delta==1u)<<read;
                pop_hits |= (fifo_delta==1u)<<read;
                valid_hits |= (valid_delta==1u)<<read;
                exact_hits |= nb_mpr_lane_word_matches(words,0)<<read;
            }
            printf("DDR4 MPR 2D boot phase: control=%u phase=%u edges=%04x fifo=%04x valid=%04x exact=%04x\n",
                control,phase,edge_hits,pop_hits,valid_hits,exact_hits);
        }
        if (control==0) ddrphy_gate_phase0_write(saved_phase0);
        else ddrphy_gate_phase_upper0_write(saved_phase_upper0);
    }
    cdelay(100);
#endif

    if (saved_vtc) ddrphy_en_vtc_write(0);
    ddrphy_gate_delay_write(saved_gate);
    ddrphy_gate_override_write(USNATIVE_LANE_MASK);
    for (unsigned lane=0; lane<4; ++lane)
        nb_mpr_lane_gate_width(lane,saved_lane_gate[lane],
            lane==0 ? lane0_gate_width : gate_width);
    cdelay(100);

    /* Gate sweep. Previously selected lanes keep their best gate; lanes not
     * yet scanned remain at their boot value. */
    for (unsigned lane=0; lane<4; ++lane) {
        unsigned best_passes=0,best_fifo=0,best_exact=0;
        unsigned best_distance=~0u;
        unsigned best_gate=saved_lane_gate[lane],best_phase=0,best_upper_phase=0;
        for (unsigned gi=0; gi<sizeof(gate_candidates)/sizeof(gate_candidates[0]); ++gi) {
            unsigned gate=gate_candidates[gi];
            unsigned phase_count=1;
#ifdef CSR_DDRPHY_GATE_PHASE0_ADDR
            if (lane==0) phase_count=4;
#endif
            for (unsigned phase=0; phase<phase_count; ++phase) {
            unsigned upper_phase_count=1;
#ifdef CSR_DDRPHY_GATE_PHASE_UPPER0_ADDR
            if (lane==0) upper_phase_count=4;
#endif
            for (unsigned upper_phase=0; upper_phase<upper_phase_count; ++upper_phase) {
            unsigned passes=0,fifo_hits=0,valid_hits=0,exact_hits=0;
#ifdef CSR_DDRPHY_GATE_PHASE0_ADDR
            if (lane==0) ddrphy_gate_phase0_write(phase);
#endif
#ifdef CSR_DDRPHY_GATE_PHASE_UPPER0_ADDR
            if (lane==0) ddrphy_gate_phase_upper0_write(upper_phase);
#endif
            nb_mpr_lane_gate_width(lane,gate,
                lane==0 ? lane0_gate_width : gate_width);
            cdelay(100);
            for (unsigned read=0; read<3; ++read) {
                unsigned fifo_ok,valid_ok,exact_ok;
                uint64_t lane_word=0;
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
                unsigned dqs_before=nb_mpr_dqs_wrclk_edges(lane);
#endif
                ddrphy_debug_clear_write(1); cdelay(100);
                (void)nb_mpr_2d_candidate(lane,&fifo_ok,&valid_ok,&exact_ok,
                    lane==0 ? &lane_word : 0);
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
                cdelay(100);
                unsigned dqs_delta=nb_mpr_dqs_wrclk_edges(lane)-dqs_before;
#endif
                if (lane==0) {
                    printf("DDR4 MPR 2D lane0 gate word: gate=%u phase=%u upper=%u read=%u data=%08x%08x fifo=%u valid=%u exact=%u",
                        gate,phase,upper_phase,read,(unsigned)(lane_word>>32),(unsigned)lane_word,
                        fifo_ok,valid_ok,exact_ok);
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
                    printf(" dqs_edges=%u",dqs_delta);
#endif
                    printf("\n");
                }
                fifo_hits+=fifo_ok; valid_hits+=valid_ok; exact_hits+=exact_ok;
                passes+=fifo_ok && valid_ok && exact_ok;
            }
            unsigned distance=(gate>saved_lane_gate[lane]) ?
                gate-saved_lane_gate[lane] : saved_lane_gate[lane]-gate;
            /* Prefer complete selected-pop/valid/MPR matches; use FIFO
             * reliability to break ties between equivalent candidates. */
            if (passes>best_passes ||
                    (passes==best_passes && fifo_hits>best_fifo) ||
                    (passes==best_passes && fifo_hits==best_fifo && exact_hits>best_exact) ||
                    (passes==best_passes && fifo_hits==best_fifo &&
                     exact_hits==best_exact && distance<best_distance)) {
                best_passes=passes; best_fifo=fifo_hits; best_exact=exact_hits;
                best_gate=gate; best_phase=phase; best_upper_phase=upper_phase;
                best_distance=distance;
            }
            printf("DDR4 MPR 2D gate: lane=%u tap=%u gate=%u phase=%u upper=%u pass=%u/3 fifo=%u/3 valid=%u/3 exact=%u/3\n",
                lane,original_rx[lane],gate,phase,upper_phase,passes,fifo_hits,valid_hits,exact_hits);
            }
            }
        }
        selected_gate[lane]=best_gate;
#ifdef CSR_DDRPHY_GATE_PHASE0_ADDR
        if (lane==0) ddrphy_gate_phase0_write(best_phase);
#endif
#ifdef CSR_DDRPHY_GATE_PHASE_UPPER0_ADDR
        if (lane==0) ddrphy_gate_phase_upper0_write(best_upper_phase);
#endif
        nb_mpr_lane_gate_width(lane,best_gate,
            lane==0 ? lane0_gate_width : gate_width);
        cdelay(100);
        printf("DDR4 MPR 2D gate best: lane=%u gate=%u phase=%u upper=%u pass=%u/3\n",
            lane,best_gate,best_phase,best_upper_phase,best_passes);
    }

#if defined(CSR_DDRPHY_TAP_RX_RST_ADDR) && defined(CSR_DDRPHY_TAP_RX_INC_ADDR)
    /* Start at the BISC-selected strobe tap without disturbing it. The
     * preceding coarse reset-and-reload scan skipped this exact seed and
     * could not separate tap sensitivity from reset side effects. */
    ddrphy_tap_select_write(usnative_dqs_taps[0]);
    cdelay(100);
    if (!native_tap_wait()) { ++errors; goto cleanup; }
    original_dqs_rx=ddrphy_tap_rx_count_read();
    if (original_dqs_rx>=SDRAM_PHY_DELAYS) { ++errors; goto cleanup; }
    dqs_rx_saved=1;
    unsigned seed_passes=0;
    unsigned last_dqs_rx=original_dqs_rx+16;
    if (last_dqs_rx>=SDRAM_PHY_DELAYS) last_dqs_rx=SDRAM_PHY_DELAYS-1;
    for (unsigned tap=original_dqs_rx; tap<=last_dqs_rx; ++tap) {
        unsigned passes=0,edges=0,fifo_hits=0,exact_hits=0;
        if (tap!=original_dqs_rx) {
            ddrphy_tap_rx_inc_write(1); cdelay(100);
            if (!native_tap_wait() || ddrphy_tap_rx_count_read()!=tap) {
                ++errors; goto cleanup;
            }
        }
        for (unsigned read=0; read<5; ++read) {
            unsigned fifo_ok,valid_ok,exact_ok;
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
            unsigned before=nb_mpr_dqs_wrclk_edges(0);
#endif
            ddrphy_debug_clear_write(1); cdelay(100);
            (void)nb_mpr_2d_candidate(0,&fifo_ok,&valid_ok,&exact_ok,0);
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
            cdelay(100);
            edges+=(nb_mpr_dqs_wrclk_edges(0)-before)==1u;
#endif
            fifo_hits+=fifo_ok; exact_hits+=exact_ok;
            passes+=fifo_ok && valid_ok && exact_ok;
        }
        if (tap==original_dqs_rx) seed_passes=passes;
        printf("DDR4 MPR 2D DQS RX: lane=0 tap=%u pass=%u/5 edges=%u/5 fifo=%u/5 exact=%u/5\n",
            tap,passes,edges,fifo_hits,exact_hits);
    }
    if (!nb_mpr_strobe_rx_tap(0,original_dqs_rx)) { ++errors; goto cleanup; }
    unsigned restored_passes=0,restored_edges=0;
    for (unsigned read=0; read<10; ++read) {
        unsigned fifo_ok,valid_ok,exact_ok;
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
        unsigned before=nb_mpr_dqs_wrclk_edges(0);
#endif
        ddrphy_debug_clear_write(1); cdelay(100);
        (void)nb_mpr_2d_candidate(0,&fifo_ok,&valid_ok,&exact_ok,0);
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
        cdelay(100);
        restored_edges+=(nb_mpr_dqs_wrclk_edges(0)-before)==1u;
#endif
        restored_passes+=fifo_ok && valid_ok && exact_ok;
    }
    printf("DDR4 MPR 2D DQS RX restore: lane=0 tap=%u pass=%u/10 edges=%u/10 seed_pass=%u/5\n",
        original_dqs_rx,restored_passes,restored_edges,seed_passes);
#endif

    /* RX tap scan with the complete selected per-byte gate vector. Restore
     * each lane to boot RX while searching so candidates are comparable. */
    for (unsigned lane=0; lane<4; ++lane) {
        unsigned best_passes=0,best_fifo=0,best_exact=0;
        unsigned best_distance=~0u;
        unsigned best_tap=original_rx[lane];
        for (unsigned tap=0; tap<=128; tap+=8) {
            unsigned passes=0,fifo_hits=0,valid_hits=0,exact_hits=0;
            if (!nb_delay(lane,tap,0)) { ++errors; continue; }
            for (unsigned read=0; read<3; ++read) {
                unsigned fifo_ok,valid_ok,exact_ok;
                uint64_t lane_word=0;
                ddrphy_debug_clear_write(1); cdelay(100);
                (void)nb_mpr_2d_candidate(lane,&fifo_ok,&valid_ok,&exact_ok,
                    lane==0 ? &lane_word : 0);
                if (lane==0)
                    printf("DDR4 MPR 2D lane0 RX word: tap=%u read=%u data=%08x%08x fifo=%u valid=%u exact=%u\n",
                        tap,read,(unsigned)(lane_word>>32),(unsigned)lane_word,
                        fifo_ok,valid_ok,exact_ok);
                fifo_hits+=fifo_ok; valid_hits+=valid_ok; exact_hits+=exact_ok;
                passes+=fifo_ok && valid_ok && exact_ok;
            }
            unsigned distance=(tap>original_rx[lane]) ? tap-original_rx[lane] : original_rx[lane]-tap;
            if (fifo_hits>best_fifo ||
                    (fifo_hits==best_fifo && passes>best_passes) ||
                    (fifo_hits==best_fifo && passes==best_passes && exact_hits>best_exact) ||
                    (fifo_hits==best_fifo && passes==best_passes &&
                     exact_hits==best_exact && distance<best_distance)) {
                best_passes=passes; best_fifo=fifo_hits; best_exact=exact_hits;
                best_tap=tap; best_distance=distance;
            }
            printf("DDR4 MPR 2D RX: lane=%u gate=%u tap=%u pass=%u/3 fifo=%u/3 valid=%u/3 exact=%u/3\n",
                lane,selected_gate[lane],tap,passes,fifo_hits,valid_hits,exact_hits);
        }
        selected_rx[lane]=best_tap;
        if (!nb_delay(lane,original_rx[lane],0)) { ++errors; goto cleanup; }
        printf("DDR4 MPR 2D RX best: lane=%u gate=%u tap=%u pass=%u/3 seed=%u\n",
            lane,selected_gate[lane],best_tap,best_passes,original_rx[lane]);
    }

    for (unsigned lane=0; lane<4; ++lane) {
        nb_mpr_lane_gate_width(lane,selected_gate[lane],
            lane==0 ? lane0_gate_width : gate_width);
        if (!nb_delay(lane,selected_rx[lane],0)) { ++errors; goto cleanup; }
    }
    cdelay(100);
    ddrphy_rx_trace_arm_write(1); cdelay(100);
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
    unsigned dqs_idle_before[4],dqs_idle_delta[4];
    for (unsigned lane=0; lane<4; ++lane)
        dqs_idle_before[lane]=nb_mpr_dqs_wrclk_edges(lane);
    cdelay(200); /* Approximate the read plus post-read settle interval. */
    for (unsigned lane=0; lane<4; ++lane)
        dqs_idle_delta[lane]=nb_mpr_dqs_wrclk_edges(lane)-dqs_idle_before[lane];
#endif
    for (unsigned repeat=0; repeat<10; ++repeat) {
        unsigned fifo_hits=0,valid_ok,exact_hits=0,pass=1;
        uint64_t words[4];
        unsigned before[4];
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
        unsigned dqs_before[4],dqs_delta[4];
#endif
        unsigned valid_before=nb_mpr_valid_count();
        ddrphy_debug_clear_write(1); cdelay(100);
        for (unsigned lane=0; lane<4; ++lane) before[lane]=nb_mpr_lane_fifo_reads(lane);
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
        for (unsigned lane=0; lane<4; ++lane)
            dqs_before[lane]=nb_mpr_dqs_wrclk_edges(lane);
#endif
#ifdef CSR_DDRPHY_RX_VALID_SNAPSHOT_ARM_ADDR
        ddrphy_rx_valid_snapshot_arm_write(1);
        cdelay(100);
#endif
        nb_mpr_lane_sweep_read(words);
#if defined(CSR_DDRPHY_RX_VALID_SNAPSHOT_DONE_ADDR) && \
        defined(CSR_DDRPHY_RX_VALID_SNAPSHOT_WORD7_ADDR)
        cdelay(100);
        if (ddrphy_rx_valid_snapshot_done_read()) {
            printf("DDR4 MPR valid edge: index=%u dfi=%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x\n",
                repeat,
                (unsigned)ddrphy_rx_valid_snapshot_word0_read(),
                (unsigned)ddrphy_rx_valid_snapshot_word1_read(),
                (unsigned)ddrphy_rx_valid_snapshot_word2_read(),
                (unsigned)ddrphy_rx_valid_snapshot_word3_read(),
                (unsigned)ddrphy_rx_valid_snapshot_word4_read(),
                (unsigned)ddrphy_rx_valid_snapshot_word5_read(),
                (unsigned)ddrphy_rx_valid_snapshot_word6_read(),
                (unsigned)ddrphy_rx_valid_snapshot_word7_read());
        } else {
            printf("DDR4 MPR valid edge: index=%u absent\n", repeat);
        }
#endif
#ifdef CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
        cdelay(100); /* Allow the Gray-coded counts to cross into PHY/sys. */
        for (unsigned lane=0; lane<4; ++lane)
            dqs_delta[lane]=nb_mpr_dqs_wrclk_edges(lane)-dqs_before[lane];
        printf("DDR4 MPR DQS WRCLK: index=%u idle=%u,%u,%u,%u read=%u,%u,%u,%u\n",
            repeat,dqs_idle_delta[0],dqs_idle_delta[1],dqs_idle_delta[2],dqs_idle_delta[3],
            dqs_delta[0],dqs_delta[1],dqs_delta[2],dqs_delta[3]);
#endif
        valid_ok=(nb_mpr_valid_count()-valid_before)==1u;
        for (unsigned lane=0; lane<4; ++lane) {
            unsigned fifo_ok=(nb_mpr_lane_fifo_reads(lane)-before[lane])==1u;
            unsigned exact_ok=(unsigned)nb_mpr_lane_word_matches(words,lane);
            fifo_hits+=fifo_ok; exact_hits+=exact_ok;
            if (!fifo_ok || !exact_ok) pass=0;
        }
        if (!valid_ok) pass=0;
        if (!pass) ++errors;
        printf("DDR4 MPR 2D repeat: index=%u gates=%u,%u,%u,%u rx=%u,%u,%u,%u fifo=%u/4 valid=%u exact=%u/4 result=%s\n",
            repeat,selected_gate[0],selected_gate[1],selected_gate[2],selected_gate[3],
            selected_rx[0],selected_rx[1],selected_rx[2],selected_rx[3],
            fifo_hits,valid_ok,exact_hits,pass ? "DIAGNOSTIC_PASS" : "FAIL");
    }

cleanup:
    /* One cleanup path owns every setting changed by this experiment. */
    if (mpr_enabled) {
#ifdef CONFIG_SDRAM_USNATIVE_MRS_COMMAND_TRACE
        nb_mpr_arm_mrs_trace();
#endif
        nb_mpr_mrs(NB_MPR_NORMAL_MR3);
#ifdef CONFIG_SDRAM_USNATIVE_MRS_COMMAND_TRACE
        nb_mpr_print_mrs_trace("restore");
        ddrphy_rx_trace_mrs_mode_write(0);
        cdelay(100);
#endif
    }
#if defined(CSR_DDRPHY_TAP_RX_RST_ADDR) && defined(CSR_DDRPHY_TAP_RX_INC_ADDR)
    if (dqs_rx_saved && !nb_mpr_strobe_rx_tap(0,original_dqs_rx)) restore_ok=0;
#endif
#ifdef CSR_DDRPHY_GATE_PHASE0_ADDR
    ddrphy_gate_phase0_write(saved_phase0);
#endif
#ifdef CSR_DDRPHY_GATE_PHASE_UPPER0_ADDR
    ddrphy_gate_phase_upper0_write(saved_phase_upper0);
#endif
    for (unsigned lane=0; lane<4; ++lane) {
        if ((rx_saved_mask & (1u<<lane)) && !nb_delay(lane,original_rx[lane],0))
            restore_ok=0;
    }
    ddrphy_gate_delay_write(saved_gate);
    ddrphy_gate_override_write(saved_override);
    for (unsigned lane=0; lane<4; ++lane)
        nb_mpr_lane_gate_width(lane,saved_lane_gate[lane],saved_width[lane]);
    ddrphy_dly_sel_write(saved_selector);
    ddrphy_en_vtc_write(saved_vtc);
    cdelay(100);
    for (unsigned lane=0; lane<4; ++lane) {
        ddrphy_dly_sel_write(1u<<lane);
        ddrphy_tap_select_write(usnative_dq_taps[lane*8]); cdelay(100);
        if ((rx_saved_mask & (1u<<lane)) &&
                (!native_tap_wait() || ddrphy_tap_rx_count_read()!=original_rx[lane]))
            restore_ok=0;
        if (ddrphy_gate_delay_read()!=saved_gate) restore_ok=0;
    }
    ddrphy_dly_sel_write(saved_selector);
    if (ddrphy_dly_sel_read()!=saved_selector ||
            ddrphy_gate_delay_read()!=saved_gate ||
            ddrphy_gate_override_read()!=saved_override ||
            ddrphy_en_vtc_read()!=saved_vtc)
        restore_ok=0;
    if (ddrphy_gate_delay0_read()!=saved_lane_gate[0] ||
            ddrphy_gate_delay1_read()!=saved_lane_gate[1] ||
            ddrphy_gate_delay2_read()!=saved_lane_gate[2] ||
            ddrphy_gate_delay3_read()!=saved_lane_gate[3] ||
            ddrphy_gate_width0_read()!=saved_width[0] ||
            ddrphy_gate_width1_read()!=saved_width[1] ||
            ddrphy_gate_width2_read()!=saved_width[2] ||
            ddrphy_gate_width3_read()!=saved_width[3])
        restore_ok=0;
    if (!restore_ok) {
        printf("DDR4 MPR 2D scan RESTORE FAILURE: asserting PHY reset; hard reset required\n");
        ddrphy_bisc_only_write(1);
        ddrphy_rst_write(1);
    }
    printf("DDR4 MPR 2D scan complete: diagnostic-only %s; MR3 normal-mode write=%s (no readback); RX/gate/width/VTC/selector verification=%s\n",
        (!errors && restore_ok) ? "PASS" : "FAIL",
        mpr_enabled ? "issued" : "not-needed",restore_ok ? "PASS" : "FAIL");
    return restore_ok;
}
#endif

/* The existing calibration hook calls this function only when the legacy RX
 * scan option is enabled. The new board option defines both scan macros and
 * dispatches here, preserving the established behavior for legacy builds. */
static int nb_mpr_rx_tap_scan(void)
{
#if defined(CONFIG_SDRAM_USNATIVE_MPR_2D_SCAN)
    return nb_mpr_2d_scan();
#else

    static const unsigned candidates[]={16,32,48,64,80,96};
    const unsigned saved_selector=ddrphy_dly_sel_read();
    const unsigned saved_vtc=ddrphy_en_vtc_read();
    const unsigned saved_gate=ddrphy_gate_delay_read();
    const unsigned saved_override=ddrphy_gate_override_read();
    const unsigned saved_lane_gate[4]={ddrphy_gate_delay0_read(),ddrphy_gate_delay1_read(),
        ddrphy_gate_delay2_read(),ddrphy_gate_delay3_read()};
    const unsigned saved_width[4]={ddrphy_gate_width0_read(),ddrphy_gate_width1_read(),
        ddrphy_gate_width2_read(),ddrphy_gate_width3_read()};
    unsigned original[4],best[4],best_score[4]={0,0,0,0};
    unsigned errors=0,restore_ok=1;

    if (saved_vtc) {
        printf("DDR4 MPR RX scan skipped: VTC is enabled\n");
        return 0;
    }
    for (unsigned lane=0; lane<4; ++lane) {
        ddrphy_dly_sel_write(1u<<lane);
        ddrphy_tap_select_write(usnative_dq_taps[lane*8]); cdelay(100);
        if (!native_tap_wait()) { ddrphy_dly_sel_write(saved_selector); return 0; }
        original[lane]=ddrphy_tap_rx_count_read();
        if (original[lane]>=SDRAM_PHY_DELAYS) { ddrphy_dly_sel_write(saved_selector); return 0; }
        best[lane]=original[lane];
    }
    ddrphy_dly_sel_write(saved_selector);
    printf("DDR4 MPR RX scan: fixed gates=2,1,2,2 width=2; taps=16,32,48,64,80,96; one read/candidate\n");
    ddrphy_gate_delay_write(2); ddrphy_gate_override_write(USNATIVE_LANE_MASK);
    for (unsigned lane=0; lane<4; ++lane)
        nb_mpr_lane_gate_width(lane,lane==1 ? 1 : 2,2);
    cdelay(100);
    sdram_dfii_pi0_address_write(1u<<10); sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS); cdelay(100);
    nb_mpr_mrs(NB_MPR_ENABLE_MR3);

    for (unsigned lane=0; lane<4; ++lane) {
        unsigned best_distance=~0u;
        for (unsigned i=0; i<sizeof(candidates)/sizeof(candidates[0]); ++i) {
            uint64_t words[4];
            unsigned tap=candidates[i],before,valid_before,delta,valid_delta,exact,score;
            if (tap>=SDRAM_PHY_DELAYS || !nb_delay(lane,tap,0)) { ++errors; continue; }
            ddrphy_debug_clear_write(1); cdelay(100);
            before=nb_mpr_lane_fifo_reads(lane); valid_before=nb_mpr_valid_count();
            nb_mpr_lane_sweep_read(words);
            delta=nb_mpr_lane_fifo_reads(lane)-before;
            valid_delta=nb_mpr_valid_count()-valid_before;
            exact=(unsigned)nb_mpr_lane_word_matches(words,lane);
            score=(delta==1u)+(valid_delta==1u)+exact;
            unsigned distance=(tap>original[lane]) ? tap-original[lane] : original[lane]-tap;
            if (score>best_score[lane] || (score==best_score[lane] && distance<best_distance)) {
                best_score[lane]=score; best[lane]=tap; best_distance=distance;
            }
            printf("DDR4 MPR RX candidate: lane=%u tap=%u fifo_delta=%u valid_delta=%u exact_mpr=%u score=%u/3\n",
                lane,tap,delta,valid_delta,exact,score);
        }
        printf("DDR4 MPR RX lane best: lane=%u tap=%u score=%u/3 seed=%u\n",
            lane,best[lane],best_score[lane],original[lane]);
        if (!nb_delay(lane,original[lane],0)) ++errors;
    }
    for (unsigned lane=0; lane<4; ++lane)
        if (!nb_delay(lane,best[lane],0)) ++errors;
    /* Re-arm here so the host's final trace belongs to the selected RX vector,
     * replacing any earlier gate-sweep trace. */
    ddrphy_rx_trace_arm_write(1); cdelay(100);
    for (unsigned repeat=0; repeat<2; ++repeat) {
        uint64_t words[4]; unsigned delta[4],valid_before,valid_delta,pass=1,exact=1;
        ddrphy_debug_clear_write(1); cdelay(100);
        for (unsigned lane=0; lane<4; ++lane) delta[lane]=nb_mpr_lane_fifo_reads(lane);
        valid_before=nb_mpr_valid_count(); nb_mpr_lane_sweep_read(words);
        valid_delta=nb_mpr_valid_count()-valid_before;
        for (unsigned lane=0; lane<4; ++lane) {
            delta[lane]=nb_mpr_lane_fifo_reads(lane)-delta[lane];
            if (!nb_mpr_lane_word_matches(words,lane)) exact=0;
            if (delta[lane]!=1u || !nb_mpr_lane_word_matches(words,lane)) pass=0;
        }
        if (valid_delta!=1u) pass=0;
        if (!pass) ++errors;
        printf("DDR4 MPR RX repeat: index=%u taps=%u,%u,%u,%u fifo_delta=%u,%u,%u,%u valid_delta=%u exact_mpr=%u result=%s\n",
            repeat,best[0],best[1],best[2],best[3],delta[0],delta[1],delta[2],delta[3],
            valid_delta,exact,pass ? "DIAGNOSTIC_PASS" : "FAIL");
    }
    nb_mpr_mrs(NB_MPR_NORMAL_MR3);
    for (unsigned lane=0; lane<4; ++lane)
        if (!nb_delay(lane,original[lane],0)) restore_ok=0;
    ddrphy_gate_delay_write(saved_gate); ddrphy_gate_override_write(saved_override);
    for (unsigned lane=0; lane<4; ++lane)
        nb_mpr_lane_gate_width(lane,saved_lane_gate[lane],saved_width[lane]);
    ddrphy_dly_sel_write(saved_selector); ddrphy_en_vtc_write(saved_vtc); cdelay(100);
    printf("DDR4 MPR RX scan complete: diagnostic-only %s; taps/gates/widths/MR3 restored\n",
        (!errors && restore_ok) ? "PASS" : "FAIL");
    return restore_ok;
#endif
}

#endif

/* Diagnostic-only lane gate scan. It reads the DRAM's MPR, never writes the
 * array, and leaves CK/RX/TX delay taps untouched. */
static int nb_mpr_lane_by_lane_gate_sweep(void)
{
    const unsigned saved_gate=ddrphy_gate_delay_read();
    const unsigned saved_override=ddrphy_gate_override_read();
    const unsigned saved_lane_gate[4]={
        ddrphy_gate_delay0_read(), ddrphy_gate_delay1_read(),
        ddrphy_gate_delay2_read(), ddrphy_gate_delay3_read()
    };
    const unsigned saved_lane_width[4]={
        ddrphy_gate_width0_read(), ddrphy_gate_width1_read(),
        ddrphy_gate_width2_read(), ddrphy_gate_width3_read()
    };
    unsigned best_gate[4]={2, 2, 2, 2};
    unsigned best_score[4]={0, 0, 0, 0};
    unsigned errors=0;

    printf("DDR4 MPR lane gate sweep: fixed CK/RX/TX; gates=1,2,3; width=2\n");
    ddrphy_gate_delay_write(2);
    ddrphy_gate_override_write(USNATIVE_LANE_MASK);
    for (unsigned lane=0; lane<4; ++lane) {
        switch (lane) {
        case 0: ddrphy_gate_delay0_write(2); ddrphy_gate_width0_write(2); break;
        case 1: ddrphy_gate_delay1_write(2); ddrphy_gate_width1_write(2); break;
        case 2: ddrphy_gate_delay2_write(2); ddrphy_gate_width2_write(2); break;
        default: ddrphy_gate_delay3_write(2); ddrphy_gate_width3_write(2); break;
        }
    }
    cdelay(100);
    sdram_dfii_pi0_address_write(1u<<10);
    sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
    cdelay(100);
    nb_mpr_mrs(NB_MPR_ENABLE_MR3);

    for (unsigned lane=0; lane<4; ++lane) {
        for (unsigned gate=NB_MPR_LANE_GATE_FIRST; gate<=NB_MPR_LANE_GATE_LAST; ++gate) {
            uint64_t words[4];
            unsigned before, after, delta, valid_before, valid_delta;
            unsigned pattern_ok, score;
            switch (lane) {
            case 0: ddrphy_gate_delay0_write(gate); break;
            case 1: ddrphy_gate_delay1_write(gate); break;
            case 2: ddrphy_gate_delay2_write(gate); break;
            default: ddrphy_gate_delay3_write(gate); break;
            }
            cdelay(100);
            /* Reset only the diagnostic event counters; native FIFOs and
             * trained CK/RX/TX settings are not changed. */
            ddrphy_debug_clear_write(1);
            cdelay(100);
            before=nb_mpr_lane_fifo_reads(lane);
            valid_before=nb_mpr_valid_count();
            nb_mpr_lane_sweep_read(words);
            after=nb_mpr_lane_fifo_reads(lane);
            delta=after-before;
            valid_delta=nb_mpr_valid_count()-valid_before;
            pattern_ok=(unsigned)nb_mpr_lane_word_matches(words, lane);
            score=(delta == 1u) + pattern_ok + (valid_delta == 1u);
            if (score > best_score[lane] ||
                    (score == best_score[lane] && gate == 2u)) {
                best_score[lane]=score;
                best_gate[lane]=gate;
            }
            printf("DDR4 MPR lane gate: lane=%u gate=%u fifo_delta=%u valid_delta=%u exact_mpr=%u score=%u/3\n",
                lane, gate, delta, valid_delta, pattern_ok, score);
        }
    }

    printf("DDR4 MPR lane gate best vector: %u,%u,%u,%u score=%u,%u,%u,%u\n",
        best_gate[0], best_gate[1], best_gate[2], best_gate[3],
        best_score[0], best_score[1], best_score[2], best_score[3]);
    for (unsigned lane=0; lane<4; ++lane) {
        switch (lane) {
        case 0: ddrphy_gate_delay0_write(best_gate[lane]); break;
        case 1: ddrphy_gate_delay1_write(best_gate[lane]); break;
        case 2: ddrphy_gate_delay2_write(best_gate[lane]); break;
        default: ddrphy_gate_delay3_write(best_gate[lane]); break;
        }
    }
    cdelay(100);
    ddrphy_rx_trace_arm_write(1);
    cdelay(100); /* Synchronize the arm to the PHY sampling domain. */
    for (unsigned repeat=0; repeat<2; ++repeat) {
        uint64_t words[4];
        unsigned delta[4], valid_before, valid_delta, valid=1, exact_mpr=1;
        ddrphy_debug_clear_write(1);
        cdelay(100);
        for (unsigned lane=0; lane<4; ++lane)
            delta[lane]=nb_mpr_lane_fifo_reads(lane);
        valid_before=nb_mpr_valid_count();
        nb_mpr_lane_sweep_read(words);
        valid_delta=nb_mpr_valid_count()-valid_before;
        for (unsigned lane=0; lane<4; ++lane) {
            delta[lane]=nb_mpr_lane_fifo_reads(lane)-delta[lane];
            if (!nb_mpr_lane_word_matches(words, lane)) exact_mpr=0;
            if (delta[lane] != 1u || !nb_mpr_lane_word_matches(words, lane))
                valid=0;
        }
        if (valid_delta != 1u) valid=0;
        if (!valid) ++errors;
        printf("DDR4 MPR lane gate repeat: index=%u vector=%u,%u,%u,%u fifo_delta=%u,%u,%u,%u valid_delta=%u exact_mpr=%u result=%s\n",
            repeat, best_gate[0], best_gate[1], best_gate[2], best_gate[3],
            delta[0], delta[1], delta[2], delta[3], valid_delta, exact_mpr,
            valid ? "DIAGNOSTIC_PASS" : "FAIL");
    }

    nb_mpr_mrs(NB_MPR_NORMAL_MR3);
    ddrphy_gate_delay_write(saved_gate);
    ddrphy_gate_override_write(saved_override);
    ddrphy_gate_delay0_write(saved_lane_gate[0]);
    ddrphy_gate_delay1_write(saved_lane_gate[1]);
    ddrphy_gate_delay2_write(saved_lane_gate[2]);
    ddrphy_gate_delay3_write(saved_lane_gate[3]);
    ddrphy_gate_width0_write(saved_lane_width[0]);
    ddrphy_gate_width1_write(saved_lane_width[1]);
    ddrphy_gate_width2_write(saved_lane_width[2]);
    ddrphy_gate_width3_write(saved_lane_width[3]);
    cdelay(100);
    printf("DDR4 MPR lane gate sweep complete: diagnostic-only %s; no training value retained; gates/override/widths restored\n",
        errors ? "FAIL" : "PASS");
    return errors;
}
#endif

/* Sweep a small gate range while holding the boot CK/RX/TX settings fixed.
 * This is diagnostic evidence only: it does not choose or retain a training
 * value. Keep the original shared and per-lane gate settings for cleanup. */
static int nb_mpr_gate_sweep(void)
{
    const unsigned saved_gate=ddrphy_gate_delay_read();
    const unsigned saved_lane_gate[4]={
        ddrphy_gate_delay0_read(), ddrphy_gate_delay1_read(),
        ddrphy_gate_delay2_read(), ddrphy_gate_delay3_read()
    };
#if defined(CONFIG_SDRAM_USNATIVE_MPR_GATE_WIDTH2) && \
    !defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TRACE) && \
    !defined(CONFIG_SDRAM_USNATIVE_TAGGED_READ_DIAGNOSTIC) && \
    !defined(CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP)
#error "MPR gate width 2 requires the MPR RX trace or tagged read diagnostic"
#endif

#if defined(CONFIG_SDRAM_USNATIVE_MPR_GATE_WIDTH2) && \
    defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TRACE)
    const unsigned saved_lane_width[4]={
        ddrphy_gate_width0_read(), ddrphy_gate_width1_read(),
        ddrphy_gate_width2_read(), ddrphy_gate_width3_read()
    };
#endif
    unsigned any_nonzero=0, any_match=0;
    int passed=1;

    printf("DDR4 MPR gate sweep: fixed CK/RX/TX, global_gate=[%u..%u], lane0_trace_offset=%d at gate=2, reads/gate=%u\n",
        (unsigned)NB_MPR_GATE_FIRST, (unsigned)NB_MPR_GATE_LAST,
        (int)NB_MPR_TRACE_LANE0_OFFSET,
        (unsigned)NB_MPR_READ_COUNT);
#if defined(CONFIG_SDRAM_USNATIVE_MPR_GATE_WIDTH2) && \
    defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TRACE)
    /* The optional trace experiment widens PHY_RDEN by one native cycle
     * while sweeping MPR gates. Restore the previous setting after the sweep. */
    ddrphy_gate_width0_write(2);
    ddrphy_gate_width1_write(2);
    ddrphy_gate_width2_write(2);
    ddrphy_gate_width3_write(2);
    cdelay(100);
    printf("DDR4 MPR gate sweep: PHY_RDEN width=2 for trace diagnostic only\n");
#endif
    nb_mpr_mrs(NB_MPR_ENABLE_MR3);
    for (unsigned gate=NB_MPR_GATE_FIRST; gate<=NB_MPR_GATE_LAST; ++gate) {
        unsigned nonzero_phases=0, matching_phases=0;
        for (unsigned lane=0; lane<4; ++lane) {
            switch (lane) {
            case 0: ddrphy_gate_delay0_write((unsigned)((int)gate +
                ((gate == 2) ? NB_MPR_TRACE_LANE0_OFFSET : 0))); break;
            case 1: ddrphy_gate_delay1_write(gate); break;
            case 2: ddrphy_gate_delay2_write(gate); break;
            default: ddrphy_gate_delay3_write(gate); break;
            }
        }
        ddrphy_gate_delay_write(gate);
        cdelay(100);

        for (unsigned read=0; read<NB_MPR_READ_COUNT; ++read) {
#if defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TRACE)
            /* Capture one checked MPR0 read at the diagnostic gate-2 setting.
             * Delay after arming so the PHY-side
             * trace crosses into its sample domain before the READ arrives. */
            if (gate==2 && read==NB_MPR_CHECK_FROM) {
                ddrphy_rx_trace_arm_write(1);
                cdelay(100);
            }
#endif
            sdram_dfii_pi0_address_write(0);
            sdram_dfii_pi0_baddress_write(0);
            command_p0(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
            cdelay(100);
            if (read >= NB_MPR_CHECK_FROM) {
                const uint64_t words[4]={
                    sdram_dfii_pi0_rddata_read(), sdram_dfii_pi1_rddata_read(),
                    sdram_dfii_pi2_rddata_read(), sdram_dfii_pi3_rddata_read()
                };
                for (unsigned phase=0; phase<4; ++phase) {
                    if (words[phase]) nonzero_phases |= 1u<<phase;
                    if (nb_mpr_word_matches(words[phase])) ++matching_phases;
                }
            }
        }
        if (nonzero_phases) any_nonzero=1;
        if (matching_phases) any_match=1;
        printf("DDR4 MPR gate=%u nonzero_phase_mask=%x matching=%u/16 lane0_gate=%u\n",
            gate, nonzero_phases, matching_phases,
            (unsigned)((int)gate + ((gate == 2) ?
                NB_MPR_TRACE_LANE0_OFFSET : 0)));
    }

    /* Restore memory mode and all gate registers even when every sample is
     * zero or mismatched. This routine never changes CK/RX/TX settings. */
    nb_mpr_mrs(NB_MPR_NORMAL_MR3);
    ddrphy_gate_delay_write(saved_gate);
    ddrphy_gate_delay0_write(saved_lane_gate[0]);
    ddrphy_gate_delay1_write(saved_lane_gate[1]);
    ddrphy_gate_delay2_write(saved_lane_gate[2]);
    ddrphy_gate_delay3_write(saved_lane_gate[3]);
#if defined(CONFIG_SDRAM_USNATIVE_MPR_GATE_WIDTH2) && \
    defined(CONFIG_SDRAM_USNATIVE_MPR_RX_TRACE)
    ddrphy_gate_width0_write(saved_lane_width[0]);
    ddrphy_gate_width1_write(saved_lane_width[1]);
    ddrphy_gate_width2_write(saved_lane_width[2]);
    ddrphy_gate_width3_write(saved_lane_width[3]);
#endif
    cdelay(100);
    passed=any_nonzero && any_match;
    printf("DDR4 MPR gate sweep: %s; nonzero=%u match=%u; restored MR3=%04x gate=%u lanes=%u,%u,%u,%u\n",
        passed ? "PASS" : "NO_MATCH", any_nonzero, any_match,
        (unsigned)NB_MPR_NORMAL_MR3, saved_gate, saved_lane_gate[0],
        saved_lane_gate[1], saved_lane_gate[2], saved_lane_gate[3]);
    return passed;
}

static int nb_mpr_diagnostic(void)
{
    const unsigned saved_mr3=NB_MPR_NORMAL_MR3;
    int passed=1;

    printf("DDR4 MPR probe: MR3 normal=%04x, page 0/MPR0, bounded reads=%u\n",
        saved_mr3, (unsigned)NB_MPR_READ_COUNT);

    /* Close every bank before changing MR3. A10 selects PRECHARGE ALL. */
    sdram_dfii_pi0_address_write(1u<<10);
    sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
    cdelay(100);

    nb_mpr_mrs(NB_MPR_ENABLE_MR3);
    for (unsigned read=0; read<NB_MPR_READ_COUNT; ++read) {
        /* Page 0, MPR0 is BA[1:0]=00. No REF is issued while MPR mode is
         * active; each command is separated by ample tCCD/tRTP margin. */
        sdram_dfii_pi0_address_write(0);
        sdram_dfii_pi0_baddress_write(0);
        command_p0(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
        cdelay(100);

        if (read >= NB_MPR_CHECK_FROM) {
            uint64_t words[4] = {
                sdram_dfii_pi0_rddata_read(), sdram_dfii_pi1_rddata_read(),
                sdram_dfii_pi2_rddata_read(), sdram_dfii_pi3_rddata_read()
            };
            for (unsigned phase=0; phase<4; ++phase) {
                if (!nb_mpr_word_matches(words[phase])) passed=0;
                printf("MPR read=%u phase=%u data=%08x:%08x match=%u\n",
                    read, phase, (unsigned)(words[phase]>>32),
                    (unsigned)words[phase],
                    (unsigned)nb_mpr_word_matches(words[phase]));
            }
        }
    }

    /* Always issue the known normal MR3 value before returning, including a
     * data mismatch. No normal calibration proceeds with MPR enabled. */
    nb_mpr_mrs(saved_mr3);
    if (!nb_mpr_gate_sweep()) passed=0;
    printf("DDR4 MPR probe: %s; restored MR3=%04x\n",
        passed ? "PASS" : "FAIL", saved_mr3);
    return passed;
}

#endif /* CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC */
#endif /* USNATIVE_NATIVE_MPR_PROBE_H */
