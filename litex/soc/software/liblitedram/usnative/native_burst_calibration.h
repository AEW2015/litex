/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Direct-DFII clock, read and write window searches for x16/x32 DDR4.
 * Experimental, destructive calibration; see doc/usnative_bios.md.
 */
#ifndef USNATIVE_BURST_CALIBRATION_H
#define USNATIVE_BURST_CALIBRATION_H
#include <generated/csr.h>
#include <generated/sdram_phy.h>
#include "native_status_io.h"
#include "native_burst_pattern.h"
#include "native_rx_seed.h"

struct nb_window { unsigned first, last, center, short_window; };
struct nb_result { struct nb_window ck, rx[USNATIVE_LANES], dq[USNATIVE_LANES]; unsigned gate, phase; };
static unsigned nb_seed_rx[USNATIVE_LANES];
static unsigned nb_seed_tx[USNATIVE_LANES];
#if SDRAM_PHY_PHASES == 8
static unsigned nb_probe_epoch;
#endif
#ifdef CONFIG_SDRAM_USNATIVE_DEBUG
/* Set only around the first seed probe in each direction. */
static unsigned nb_seed_probe_kind;
#endif
#ifdef CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC
/* Enables one additional compact DFI write/read trace before normal training. */
static unsigned nb_dfi_diag_active;
#endif
#if (SDRAM_PHY_DATABITS == 32 && (SDRAM_PHY_PHASES == 4 || SDRAM_PHY_PHASES == 8)) || \
    USNATIVE_REDUCED_X64_1TO8
static unsigned nb_read_phase=USNATIVE_RDPHASE;
#endif
#if defined(CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC) && \
    SDRAM_PHY_DATABITS == 32 && SDRAM_PHY_PHASES == 4
static unsigned nb_write_phase=SDRAM_PHY_WRPHASE;

static void nb_issue_write(unsigned address)
{
    unsigned command=DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS|DFII_COMMAND_WRDATA;
    switch (nb_write_phase) {
    case 0:
        sdram_dfii_pi0_address_write(address); sdram_dfii_pi0_baddress_write(0);
        command_p0(command); break;
    case 1:
        sdram_dfii_pi1_address_write(address); sdram_dfii_pi1_baddress_write(0);
        command_p1(command); break;
    case 2:
        sdram_dfii_pi2_address_write(address); sdram_dfii_pi2_baddress_write(0);
        command_p2(command); break;
    default:
        sdram_dfii_pi3_address_write(address); sdram_dfii_pi3_baddress_write(0);
        command_p3(command); break;
    }
}
#endif

static void nb_set_gate(unsigned lane, unsigned delay)
{
    switch (lane) {
    case 0: ddrphy_gate_delay0_write(delay); ddrphy_gate_width0_write(1); break;
    case 1: ddrphy_gate_delay1_write(delay); ddrphy_gate_width1_write(1); break;
#if SDRAM_PHY_DATABITS >= 32
    case 2: ddrphy_gate_delay2_write(delay); ddrphy_gate_width2_write(1); break;
    case 3: ddrphy_gate_delay3_write(delay); ddrphy_gate_width3_write(1); break;
#endif
#if SDRAM_PHY_DATABITS >= 64
    case 4: ddrphy_gate_delay4_write(delay); ddrphy_gate_width4_write(1); break;
    case 5: ddrphy_gate_delay5_write(delay); ddrphy_gate_width5_write(1); break;
    case 6: ddrphy_gate_delay6_write(delay); ddrphy_gate_width6_write(1); break;
    case 7: ddrphy_gate_delay7_write(delay); ddrphy_gate_width7_write(1); break;
#endif
    }
}

static void nb_set_all_gates(unsigned delay)
{
    ddrphy_gate_delay_write(delay);
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane)
        nb_set_gate(lane, delay);
    cdelay(100);
}

static void nb_issue_read(unsigned address)
{
#if SDRAM_PHY_DATABITS == 32 && SDRAM_PHY_PHASES == 4
    switch (nb_read_phase) {
    case 0:
        sdram_dfii_pi0_address_write(address); sdram_dfii_pi0_baddress_write(0);
        command_p0(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA); break;
    case 1:
        sdram_dfii_pi1_address_write(address); sdram_dfii_pi1_baddress_write(0);
        command_p1(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA); break;
    case 2:
        sdram_dfii_pi2_address_write(address); sdram_dfii_pi2_baddress_write(0);
        command_p2(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA); break;
    default:
        sdram_dfii_pi3_address_write(address); sdram_dfii_pi3_baddress_write(0);
        command_p3(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA); break;
    }
#elif (SDRAM_PHY_DATABITS == 32 && SDRAM_PHY_PHASES == 8) || \
      USNATIVE_REDUCED_X64_1TO8
    /* The 1:8 controller splits each native four-phase command cycle across
     * two fast slots. Read commands use native phase p in the first slot;
     * x32 and reduced x64 differ in data packing, not command-phase selection. */
    switch (nb_read_phase) {
    case 0:
        sdram_dfii_pi0_address_write(address); sdram_dfii_pi0_baddress_write(0);
        command_p0(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA); break;
    case 2:
        sdram_dfii_pi2_address_write(address); sdram_dfii_pi2_baddress_write(0);
        command_p2(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA); break;
    case 1:
        sdram_dfii_pi1_address_write(address); sdram_dfii_pi1_baddress_write(0);
        command_p1(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA); break;
    default:
        sdram_dfii_pi3_address_write(address); sdram_dfii_pi3_baddress_write(0);
        command_p3(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA); break;
    }
#else
    USNATIVE_RD_ADDRESS(address); USNATIVE_RD_BANK(0);
    USNATIVE_RD_COMMAND(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
#endif
}

#if defined(CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC) && \
    defined(CONFIG_SDRAM_USNATIVE_FIXED_ROW_DIAGNOSTIC)
#if !defined(SDRAM_PHY_DDR4) || SDRAM_PHY_DATABITS != 32 || \
    SDRAM_PHY_DFI_DATABITS != 64 || SDRAM_PHY_PHASES != 4 || \
    CONFIG_CLOCK_FREQUENCY != 200000000 || SDRAM_PHY_CL != 11 || \
    SDRAM_PHY_CWL != 9
#error "Fixed-row diagnostic is restricted to AES-KU40 DDR4-1600 1:4"
#endif

#define NB_FIXED_ROW_SETTLE_CYCLES 1000u

static unsigned nb_fixed_row_capture(uint64_t data[4], unsigned valid_delta)
{
    const uint64_t expected[4]={
        0x96a53cc35aa5c33cu, 0x1e4b78d2a569c30fu,
        0xf00f69969669f00fu, 0x8731c6e25a0d4bf8u
    };
    data[0]=sdram_dfii_pi0_rddata_read();
    data[1]=sdram_dfii_pi1_rddata_read();
    data[2]=sdram_dfii_pi2_rddata_read();
    data[3]=sdram_dfii_pi3_rddata_read();

    unsigned errors=0;
    for (unsigned phase=0; phase<4; ++phase)
        if (data[phase] != expected[phase]) ++errors;

    printf("DDR4 fixed-row read: wr_cmd_phase=%u rd_cmd_phase=%u valid_delta=%u ready=%u fifo=%08x status=%08x errors=%u",
        nb_write_phase, nb_read_phase, valid_delta,
        (unsigned)ddrphy_ready_read(),
        (unsigned)csr_read_simple(CSR_DDRPHY_FIFO_EMPTY_ADDR),
        (unsigned)ddrphy_debug_read(), errors);
    for (unsigned phase=0; phase<4; ++phase)
        printf(" p%u=%08x:%08x", phase,
            (unsigned)(data[phase]>>32), (unsigned)data[phase]);
    printf("\n");
    return errors;
}

/* Destructive, diagnostic-only access to one fixed array row. Each phase
 * pair starts with PREA, ACT, one BL8 write, two separately issued reads,
 * then PREA. Long controller-clock delays intentionally remove tRP/tRCD/
 * tWR/tRTP/tCCD pressure from the phase-mapping experiment. */
static unsigned nb_fixed_row_diagnostic(void)
{
    static const uint64_t pattern[4]={
        0x96a53cc35aa5c33cu, 0x1e4b78d2a569c30fu,
        0xf00f69969669f00fu, 0x8731c6e25a0d4bf8u
    };
    unsigned errors=0;
    unsigned trace_armed=0;

    printf("DDR4 fixed-row diagnostic: row=0x040 bank=0, two reads/write, settle=%u cycles\n",
        (unsigned)NB_FIXED_ROW_SETTLE_CYCLES);
    for (unsigned write_phase=0; write_phase<4; ++write_phase) {
        nb_write_phase=write_phase;
        ddrphy_wrphase_write(write_phase);
        for (unsigned read_phase=0; read_phase<4; ++read_phase) {
            uint64_t captured[4];
            nb_read_phase=read_phase;
            ddrphy_rdphase_write(read_phase);

            /* Close all banks before opening the same row for this trial. */
            sdram_dfii_pi0_address_write(1u<<10);
            sdram_dfii_pi0_baddress_write(0);
            command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
            cdelay(NB_FIXED_ROW_SETTLE_CYCLES); /* tRP */
            sdram_dfii_pi0_address_write(0x040);
            sdram_dfii_pi0_baddress_write(0);
            command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CS);
            cdelay(NB_FIXED_ROW_SETTLE_CYCLES); /* tRCD */

            for (unsigned phase=0; phase<4; ++phase)
                switch (phase) {
                case 0: sdram_dfii_pi0_wrdata_write(pattern[phase]); break;
                case 1: sdram_dfii_pi1_wrdata_write(pattern[phase]); break;
                case 2: sdram_dfii_pi2_wrdata_write(pattern[phase]); break;
                default: sdram_dfii_pi3_wrdata_write(pattern[phase]); break;
                }
            nb_issue_write(0);
            cdelay(NB_FIXED_ROW_SETTLE_CYCLES); /* tWR and command spacing */
            printf("DDR4 fixed-row write: wr_cmd_phase=%u rd_cmd_phase=%u",
                write_phase, read_phase);
            for (unsigned phase=0; phase<4; ++phase)
                printf(" p%u=%08x:%08x", phase,
                    (unsigned)(pattern[phase]>>32), (unsigned)pattern[phase]);
            printf("\n");

            for (unsigned repeat=0; repeat<2; ++repeat) {
                unsigned valid_before = native_dfi_activity_rddata_valid_cycles_read();
                /* Keep one receive trace centered on a single known command
                 * phase pair. The PHY arm pulse crosses into its capture
                 * clock domain; wait before issuing the read trigger. */
                if (!trace_armed && write_phase==2 && read_phase==0 && repeat==0) {
                    ddrphy_rx_trace_arm_write(1);
                    cdelay(100);
                    trace_armed=1;
                }
                nb_issue_read(0);
                cdelay(NB_FIXED_ROW_SETTLE_CYCLES); /* read-data settle */
                unsigned valid_after = native_dfi_activity_rddata_valid_cycles_read();
                unsigned valid_delta = valid_after - valid_before;
                errors += nb_fixed_row_capture(captured, valid_delta);
                printf("DDR4 fixed-row sample: wr_cmd_phase=%u rd_cmd_phase=%u repeat=%u valid_delta=%u\n",
                    write_phase, read_phase, repeat, valid_delta);
            }

            cdelay(NB_FIXED_ROW_SETTLE_CYCLES); /* tRTP */
            sdram_dfii_pi0_address_write(1u<<10);
            sdram_dfii_pi0_baddress_write(0);
            command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
            cdelay(NB_FIXED_ROW_SETTLE_CYCLES);
        }
    }
    printf("DDR4 fixed-row diagnostic complete: samples=32 compare_errors=%u\n", errors);
    return errors;
}
#endif

#if defined(CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC) && \
    defined(CONFIG_SDRAM_USNATIVE_TAGGED_READ_DIAGNOSTIC)
#if !defined(SDRAM_PHY_DDR4) || SDRAM_PHY_DATABITS != 32 || \
    SDRAM_PHY_DFI_DATABITS != 64 || SDRAM_PHY_PHASES != 4 || \
    CONFIG_CLOCK_FREQUENCY != 200000000 || SDRAM_PHY_CL != 11 || \
    SDRAM_PHY_CWL != 9
#error "Tagged read diagnostic is restricted to AES-KU40 DDR4-1600 1:4"
#endif

#define NB_TAGGED_READ_SETTLE_CYCLES 1000u

static unsigned nb_tagged_read_capture(unsigned burst, const uint64_t expected[4],
    unsigned valid_delta)
{
    const uint64_t data[4]={
        sdram_dfii_pi0_rddata_read(), sdram_dfii_pi1_rddata_read(),
        sdram_dfii_pi2_rddata_read(), sdram_dfii_pi3_rddata_read()
    };
    unsigned errors=0;
    if (valid_delta != 1) ++errors;
    for (unsigned phase=0; phase<4; ++phase)
        if (data[phase] != expected[phase]) ++errors;

    printf("DDR4 tagged read: burst=%u column=%u valid_delta=%u fifo=%08x errors=%u",
        burst, burst ? 8u : 0u, valid_delta,
        (unsigned)csr_read_simple(CSR_DDRPHY_FIFO_EMPTY_ADDR), errors);
    for (unsigned phase=0; phase<4; ++phase)
        printf(" p%u=%08x:%08x", phase,
            (unsigned)(data[phase]>>32), (unsigned)data[phase]);
    printf("\n");
#if defined(CSR_DDRPHY_RX_LANE_FAULT_ADDR) && \
    defined(CSR_DDRPHY_RX_LANE_MISSING_ADDR) && \
    defined(CSR_DDRPHY_RX_LANE_READS_ADDR)
    printf("DDR4 tagged lane return: burst=%u reads=%u missing=%x fault=%u\n",
        burst, (unsigned)ddrphy_rx_lane_reads_read(),
        (unsigned)ddrphy_rx_lane_missing_read(),
        (unsigned)ddrphy_rx_lane_fault_read());
#endif
    return errors;
}

/* Destructive diagnostic of the complete receive association path. The two
 * adjacent BL8 columns hold unrelated tags. The second READ is traced after
 * the first has been captured, so stale Q from burst A is distinguishable
 * from burst B at both native FIFO Q and the returned DFI phases. The test
 * overwrites bank 0, row 0x040, columns 0 and 8; it leaves DFI in software
 * control, closes the row, and restores the prior command phase registers. */
static unsigned nb_tagged_read_diagnostic(void)
{
    static const uint64_t tag_a[4]={
        0x13579bdf2468ace0ull, 0x0f1e2d3c4b5a6978ull,
        0x55aa33cc699696f0ull, 0x89abcdef01234567ull
    };
    static const uint64_t tag_b[4]={
        0xcafef00d5a5aa5a5ull, 0xdeadbeef87654321ull,
        0xa55ac33c9669690full, 0x76543210fedcba98ull
    };
    const unsigned saved_write_phase=nb_write_phase;
    const unsigned saved_read_phase=nb_read_phase;
    const unsigned saved_gate=ddrphy_gate_delay_read();
    const unsigned saved_lane_gate[4]={
        ddrphy_gate_delay0_read(), ddrphy_gate_delay1_read(),
        ddrphy_gate_delay2_read(), ddrphy_gate_delay3_read()
    };
#ifdef CONFIG_SDRAM_USNATIVE_MPR_GATE_WIDTH2
    const unsigned saved_lane_width[4]={
        ddrphy_gate_width0_read(), ddrphy_gate_width1_read(),
        ddrphy_gate_width2_read(), ddrphy_gate_width3_read()
    };
#endif
    unsigned errors=0;

    printf("DDR4 tagged two-burst diagnostic: row=0x040 bank=0 columns=0,8; destructive\n");
#ifdef CONFIG_SDRAM_USNATIVE_MPR_GATE_WIDTH2
    printf("DDR4 tagged RX setup: gate=2 lane0=%u PHY_RDEN width=2\n",
        (unsigned)(2 + CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET));
#else
    printf("DDR4 tagged RX setup: gate=2 lane0=%u PHY_RDEN width=saved\n",
        (unsigned)(2 + CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET));
#endif
    ddrphy_gate_delay_write(2);
    ddrphy_gate_delay0_write((unsigned)(2 + CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET));
    ddrphy_gate_delay1_write(2);
    ddrphy_gate_delay2_write(2);
    ddrphy_gate_delay3_write(2);
#ifdef CONFIG_SDRAM_USNATIVE_MPR_GATE_WIDTH2
    ddrphy_gate_width0_write(2);
    ddrphy_gate_width1_write(2);
    ddrphy_gate_width2_write(2);
    ddrphy_gate_width3_write(2);
#endif
    cdelay(100);
    nb_write_phase=2;
    ddrphy_wrphase_write(nb_write_phase);
    nb_read_phase=0;
    ddrphy_rdphase_write(nb_read_phase);

    sdram_dfii_pi0_address_write(1u<<10);
    sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
    cdelay(NB_TAGGED_READ_SETTLE_CYCLES);
    sdram_dfii_pi0_address_write(0x040);
    sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CS);
    cdelay(NB_TAGGED_READ_SETTLE_CYCLES);

    for (unsigned phase=0; phase<4; ++phase)
        switch (phase) {
        case 0: sdram_dfii_pi0_wrdata_write(tag_a[phase]); break;
        case 1: sdram_dfii_pi1_wrdata_write(tag_a[phase]); break;
        case 2: sdram_dfii_pi2_wrdata_write(tag_a[phase]); break;
        default: sdram_dfii_pi3_wrdata_write(tag_a[phase]); break;
        }
    nb_issue_write(0);
    cdelay(NB_TAGGED_READ_SETTLE_CYCLES);
    for (unsigned phase=0; phase<4; ++phase)
        switch (phase) {
        case 0: sdram_dfii_pi0_wrdata_write(tag_b[phase]); break;
        case 1: sdram_dfii_pi1_wrdata_write(tag_b[phase]); break;
        case 2: sdram_dfii_pi2_wrdata_write(tag_b[phase]); break;
        default: sdram_dfii_pi3_wrdata_write(tag_b[phase]); break;
        }
    nb_issue_write(8);
    cdelay(NB_TAGGED_READ_SETTLE_CYCLES);

    unsigned valid_before=native_dfi_activity_rddata_valid_cycles_read();
    nb_issue_read(0);
    cdelay(NB_TAGGED_READ_SETTLE_CYCLES);
    unsigned valid_delta=native_dfi_activity_rddata_valid_cycles_read()-valid_before;
    errors += nb_tagged_read_capture(0, tag_a, valid_delta);

    /* Trace the second burst specifically: its distinct payload makes an
     * old FIFO Q word from burst A visible instead of a repeated-pattern hit. */
    ddrphy_rx_trace_arm_write(1);
    cdelay(100);
    valid_before=native_dfi_activity_rddata_valid_cycles_read();
    nb_issue_read(8);
    cdelay(NB_TAGGED_READ_SETTLE_CYCLES);
    valid_delta=native_dfi_activity_rddata_valid_cycles_read()-valid_before;
    errors += nb_tagged_read_capture(1, tag_b, valid_delta);

    sdram_dfii_pi0_address_write(1u<<10);
    sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
    cdelay(NB_TAGGED_READ_SETTLE_CYCLES);
    ddrphy_gate_delay_write(saved_gate);
    ddrphy_gate_delay0_write(saved_lane_gate[0]);
    ddrphy_gate_delay1_write(saved_lane_gate[1]);
    ddrphy_gate_delay2_write(saved_lane_gate[2]);
    ddrphy_gate_delay3_write(saved_lane_gate[3]);
#ifdef CONFIG_SDRAM_USNATIVE_MPR_GATE_WIDTH2
    ddrphy_gate_width0_write(saved_lane_width[0]);
    ddrphy_gate_width1_write(saved_lane_width[1]);
    ddrphy_gate_width2_write(saved_lane_width[2]);
    ddrphy_gate_width3_write(saved_lane_width[3]);
#endif
    nb_write_phase=saved_write_phase;
    ddrphy_wrphase_write(saved_write_phase);
    nb_read_phase=saved_read_phase;
    ddrphy_rdphase_write(saved_read_phase);
    cdelay(100);
    printf("DDR4 tagged two-burst diagnostic complete: errors=%u; row closed, gates/widths/phases restored\n", errors);
    return errors;
}
#endif

static unsigned nb_fail(unsigned error)
{
    sdram_software_control_on();
    ddrphy_training_error_write(error);
    ddrphy_bisc_only_write(1);
    ddrphy_rst_write(1);
    return error;
}

static int nb_bisc(void)
{
    ddrphy_bisc_only_write(1); ddrphy_rst_write(1);
    ddrphy_en_vtc_write(1); ddrphy_wlevel_en_write(0);
    sdram_dfii_control_write(0); cdelay(100000);
    ddrphy_rst_write(0);
    for (unsigned i=0; i<100000; ++i) {
        if (ddrphy_ready_read()) return 1;
        cdelay(100);
    }
    return 0;
}

static int nb_gate(unsigned nibble)
{
    unsigned actual;
    if (!native_riu_access(nibble, 0x30, 6<<9, 1, &actual)) return 0;
    if (!native_riu_access(nibble, 0x30, 0, 0, &actual)) return 0;
    return (actual & 0x1fff) == (6<<9);
}

static int nb_delay(unsigned lane, unsigned value, int transmit)
{
    if (value >= SDRAM_PHY_DELAYS) {
        printf("Native DQ tap request out of range: lane=%u dir=%s requested=%u tap_count=%u\n",
            lane, transmit ? "TX" : "RX", value, (unsigned)SDRAM_PHY_DELAYS);
        return 0;
    }
    ddrphy_dly_sel_write(1<<lane);
    if (transmit) ddrphy_wdly_dq_rst_write(1);
    else ddrphy_rdly_dq_rst_write(1);
    cdelay(100);
    for (unsigned i=0; i<value; ++i) {
        if (transmit) ddrphy_wdly_dq_inc_write(1);
        else ddrphy_rdly_dq_inc_write(1);
        cdelay(100);
    }
    ddrphy_tap_select_write(usnative_dq_taps[lane * 8]); cdelay(100);
    if (!native_tap_wait()) {
        printf("Native DQ tap timeout: lane=%u dir=%s expected=%u selector=%u allowed=%u valid=%u\n",
            lane, transmit ? "TX" : "RX", value, usnative_dq_taps[lane * 8],
            ddrphy_tap_allowed_read(), ddrphy_tap_status_valid_read());
        return 0;
    }
    unsigned actual = transmit ? ddrphy_tap_tx_count_read() :
        ddrphy_tap_rx_count_read();
    if (actual != value) {
        printf("Native DQ tap mismatch: lane=%u dir=%s expected=%u actual=%u selector=%u allowed=%u valid=%u\n",
            lane, transmit ? "TX" : "RX", value, actual,
            usnative_dq_taps[lane * 8], ddrphy_tap_allowed_read(),
            ddrphy_tap_status_valid_read());
        return 0;
    }
    return 1;
}

static int nb_boot(unsigned ck)
{
    if (!nb_bisc()) {
        printf("Native boot failed: BISC ready timeout, ck=%u\n", ck);
        return 0;
    }
    ddrphy_en_vtc_write(0); cdelay(1000);
    ddrphy_cdly_rst_write(1); cdelay(100);
    for (unsigned i=0; i<ck; ++i) { ddrphy_cdly_inc_write(1); cdelay(100); }
    ddrphy_tap_select_write(usnative_ck_taps[0]); cdelay(100);
    if (!native_tap_wait()) {
        printf("Native boot failed: CK tap status timeout, ck=%u\n", ck);
        return 0;
    }
    if (ddrphy_tap_tx_count_read()!=ck) {
        printf("Native boot failed: CK tap readback, expected=%u actual=%u\n",
            ck, ddrphy_tap_tx_count_read());
        return 0;
    }
    /* Clock moves only while the device is held reset. */
    ddrphy_bisc_only_write(0); init_sequence(); cdelay(10000);
#if (SDRAM_PHY_DATABITS == 32 && (SDRAM_PHY_PHASES == 4 || SDRAM_PHY_PHASES == 8)) || \
    USNATIVE_REDUCED_X64_1TO8
    ddrphy_rdphase_write(nb_read_phase);
#else
    ddrphy_rdphase_write(USNATIVE_RDPHASE);
#endif
#if defined(CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC) && \
    SDRAM_PHY_DATABITS == 32 && SDRAM_PHY_PHASES == 4
    ddrphy_wrphase_write(nb_write_phase);
#else
    ddrphy_wrphase_write(SDRAM_PHY_WRPHASE);
#endif
    ddrphy_tx_dqs_pre_write(0x55); ddrphy_tx_dqs_post_write(0x55);
    ddrphy_tx_dqs_idle_write(0x55);
    ddrphy_dly_sel_write(USNATIVE_LANE_MASK);
    ddrphy_rdly_dq_bitslip_rst_write(1); ddrphy_wdly_dq_bitslip_rst_write(1);
    ddrphy_gate_delay_write(USNATIVE_GATE_DELAY);
    ddrphy_gate_override_write(USNATIVE_LANE_MASK);
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane)
        nb_set_gate(lane, USNATIVE_GATE_DELAY);
    ddrphy_fifo_lane_mode_write(0);
    for (unsigned n=0; n<SDRAM_PHY_USNATIVE_DATA_CONTROLS_COUNT; ++n)
        if (!nb_gate(usnative_data_controls[n])) {
            printf("Native boot failed: RIU gate, control=%u\n",
                usnative_data_controls[n]);
            return 0;
        }
    ddrphy_dly_sel_write(USNATIVE_LANE_MASK); ddrphy_wdly_dqs_rst_write(1); cdelay(100);
    unsigned dqs_delay=68;
#if SDRAM_PHY_DATABITS == 32
    /* Keep x32 write strobes and data aligned while CK is moved. */
    dqs_delay+=ck;
#endif
    for (unsigned i=0; i<dqs_delay; ++i) { ddrphy_wdly_dqs_inc_write(1); cdelay(100); }
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane) {
        ddrphy_dly_sel_write(1<<lane); cdelay(100);
        if (ddrphy_wdly_dqs_inc_count_read()!=dqs_delay) {
            printf("Native boot failed: DQS tap readback, lane=%u expected=%u actual=%u\n",
                lane, dqs_delay, ddrphy_wdly_dqs_inc_count_read());
            return 0;
        }
        unsigned tx_delay=nb_seed_tx[lane];
#if SDRAM_PHY_DATABITS == 32
        tx_delay+=ck;
#endif
        if (!nb_delay(lane,tx_delay,1) || !nb_delay(lane,nb_seed_rx[lane],0)) {
            printf("Native boot failed: DQ tap readback, lane=%u TX=%u RX=%u\n",
                lane, tx_delay, nb_seed_rx[lane]);
            return 0;
        }
    }
    return 1;
}

#if USNATIVE_REDUCED_X64_1TO8
#if defined(CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC)
#include "native_mpr_x64.h"
#endif
#else
#if defined(CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC) || \
    defined(CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP)
#include "native_mpr_probe.h"
#endif
#endif

static unsigned nb_check(unsigned count, unsigned lanes)
{
    unsigned errors=0;
#if SDRAM_PHY_PHASES == 8
    /* There is no DFII CSR for rddata_valid. Give each scan call fresh data so
     * a stale status word from the prior tap/call cannot pass by repetition. */
    unsigned epoch=++nb_probe_epoch;
#endif
    usnative_dfi_word_t mask=0;
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane)
        if (lanes & (1u<<lane))
            mask |= (usnative_dfi_word_t)0xff << (lane*8);
#if SDRAM_PHY_DFI_DATABITS == 2*SDRAM_PHY_DATABITS
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane)
        if (lanes & (1u<<lane))
            mask |= (usnative_dfi_word_t)0xff << (lane*8+SDRAM_PHY_DATABITS);
#endif
    /* Replace startup FIFO contents before scoring a new delay point. */
    for (unsigned i=0; i<count+8; ++i) {
#if SDRAM_PHY_PHASES == 8
        usnative_dfi_word_t q[8];
        unsigned pattern_index=i + epoch*0x9e3779b9u;
        /* Each pattern is one physical BL8. Legacy x32 mode splits its four
         * 64-bit groups across eight 32-bit phases. In full-width direct-DFII
         * calibration, each physical slot needs the same isolated BL8 pattern:
         * the command stream still issues only one CAS for this probe. */
#if USNATIVE_DUAL_SLOT_FULL_WIDTH
        for (unsigned phase=0; phase<4; ++phase)
            q[phase]=(usnative_dfi_word_t)usnative_burst_phase_pattern(phase, pattern_index);
        for (unsigned phase=0; phase<4; ++phase) q[phase+4]=q[phase];
#elif SDRAM_PHY_DATABITS == 64
        uint64_t phase_data[8];
        usnative_x64_1to8_patterns(phase_data, pattern_index);
        for (unsigned phase=0; phase<8; ++phase) q[phase]=phase_data[phase];
#else
        for (unsigned phase=0; phase<4; ++phase) {
            uint64_t pattern=usnative_burst_phase_pattern(phase, pattern_index);
            q[2*phase]=(uint32_t)pattern;
            q[2*phase+1]=(uint32_t)(pattern>>32);
        }
#endif
#else
        usnative_dfi_word_t q0=0x12345678u^(i*0x1020304u);
        usnative_dfi_word_t q1=0xa55a5aa5u^(i*0x01010101u);
        usnative_dfi_word_t q2=0xdeadbeefu+i, q3=0x55aa00ffu-i;
#if SDRAM_PHY_DATABITS == 32
        q0 |= (usnative_dfi_word_t)(0x89abcdefu^(i*0x01020408u)) << 32;
        q1 |= (usnative_dfi_word_t)(0x76543210u^(i*0x08040201u)) << 32;
        q2 |= (usnative_dfi_word_t)(0x0f1e2d3cu+i) << 32;
        q3 |= (usnative_dfi_word_t)(0xf0e1d2c3u-i) << 32;
#endif
#endif
#ifdef CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC
        if (nb_dfi_diag_active && i==8) {
#ifdef CONFIG_SDRAM_USNATIVE_WRITE_MONITOR
            ddrphy_probe_fast_clear_write(1);
            cdelay(100);
#endif
        }
#endif
        sdram_dfii_pi0_address_write(i*64); sdram_dfii_pi0_baddress_write(0);
        command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CS); cdelay(1000);
#if SDRAM_PHY_PHASES == 8
        sdram_dfii_pi0_wrdata_write(q[0]); sdram_dfii_pi1_wrdata_write(q[1]);
        sdram_dfii_pi2_wrdata_write(q[2]); sdram_dfii_pi3_wrdata_write(q[3]);
        sdram_dfii_pi4_wrdata_write(q[4]); sdram_dfii_pi5_wrdata_write(q[5]);
        sdram_dfii_pi6_wrdata_write(q[6]); sdram_dfii_pi7_wrdata_write(q[7]);
#else
        sdram_dfii_pi0_wrdata_write(q0); sdram_dfii_pi1_wrdata_write(q1);
        sdram_dfii_pi2_wrdata_write(q2); sdram_dfii_pi3_wrdata_write(q3);
#endif
        unsigned write_address=(i*8)&0x3ff;
#if SDRAM_PHY_PHASES == 8
        sdram_dfii_pi0_address_write(write_address);
        USNATIVE_WR_ADDRESS(write_address); USNATIVE_WR_BANK(0);
        USNATIVE_WR_COMMAND(DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS|DFII_COMMAND_WRDATA);
#else
        sdram_dfii_pi0_address_write((i*8)&0x3ff);
#if defined(CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC) && \
    SDRAM_PHY_DATABITS == 32 && SDRAM_PHY_PHASES == 4
        if (nb_dfi_diag_active)
            nb_issue_write((i*8)&0x3ff);
        else {
            USNATIVE_WR_ADDRESS((i*8)&0x3ff); USNATIVE_WR_BANK(0);
            USNATIVE_WR_COMMAND(DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS|DFII_COMMAND_WRDATA);
        }
#else
        USNATIVE_WR_ADDRESS((i*8)&0x3ff); USNATIVE_WR_BANK(0);
        USNATIVE_WR_COMMAND(DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS|DFII_COMMAND_WRDATA);
#endif
#endif
        cdelay(1000);
#if SDRAM_PHY_PHASES == 8
        nb_issue_read(write_address);
#else
        nb_issue_read((i*8)&0x3ff); cdelay(1000);
#endif
        cdelay(1000);
        usnative_dfi_word_t r0=sdram_dfii_pi0_rddata_read(), r1=sdram_dfii_pi1_rddata_read();
        usnative_dfi_word_t r2=sdram_dfii_pi2_rddata_read(), r3=sdram_dfii_pi3_rddata_read();
#if SDRAM_PHY_PHASES == 8 && !USNATIVE_DUAL_SLOT_FULL_WIDTH
        usnative_dfi_word_t r4=sdram_dfii_pi4_rddata_read(), r5=sdram_dfii_pi5_rddata_read();
        usnative_dfi_word_t r6=sdram_dfii_pi6_rddata_read(), r7=sdram_dfii_pi7_rddata_read();
#endif
        if (i==8 && count==256) {
#if SDRAM_PHY_PHASES == 8 && !USNATIVE_DUAL_SLOT_FULL_WIDTH
            USNATIVE_DEBUG("CK_PROBE ready=%u fifo=%08x status=%08x expected-p0=%08x%08x expected-p1=%08x%08x\n",
                (unsigned)ddrphy_ready_read(),
                (unsigned)csr_read_simple(CSR_DDRPHY_FIFO_EMPTY_ADDR),
                (unsigned)ddrphy_debug_read(), (unsigned)(q[0]>>32), (unsigned)q[0],
                (unsigned)(q[1]>>32), (unsigned)q[1]);
            USNATIVE_DEBUG("CK_PROBE p0=%08x%08x p1=%08x%08x p2=%08x%08x p3=%08x%08x p4=%08x%08x p5=%08x%08x p6=%08x%08x p7=%08x%08x\n",
                (unsigned)(r0>>32), (unsigned)r0, (unsigned)(r1>>32), (unsigned)r1,
                (unsigned)(r2>>32), (unsigned)r2, (unsigned)(r3>>32), (unsigned)r3,
                (unsigned)(r4>>32), (unsigned)r4, (unsigned)(r5>>32), (unsigned)r5,
                (unsigned)(r6>>32), (unsigned)r6, (unsigned)(r7>>32), (unsigned)r7);
#elif USNATIVE_DUAL_SLOT_FULL_WIDTH
            USNATIVE_DEBUG("CK_PROBE ready=%u fifo=%08x status=%08x expected-p0=%08x%08x expected-p1=%08x%08x\n",
                (unsigned)ddrphy_ready_read(),
                (unsigned)csr_read_simple(CSR_DDRPHY_FIFO_EMPTY_ADDR),
                (unsigned)ddrphy_debug_read(), (unsigned)(q[0]>>32), (unsigned)q[0],
                (unsigned)(q[1]>>32), (unsigned)q[1]);
            USNATIVE_DEBUG("CK_PROBE p0=%08x%08x p1=%08x%08x p2=%08x%08x p3=%08x%08x\n",
                (unsigned)(r0>>32), (unsigned)r0, (unsigned)(r1>>32), (unsigned)r1,
                (unsigned)(r2>>32), (unsigned)r2, (unsigned)(r3>>32), (unsigned)r3);
#else
            USNATIVE_DEBUG("CK_PROBE ready=%u fifo=%08x status=%08x expected=%08x:%08x\n",
                (unsigned)ddrphy_ready_read(),
                (unsigned)csr_read_simple(CSR_DDRPHY_FIFO_EMPTY_ADDR),
                (unsigned)ddrphy_debug_read(), (unsigned)((uint64_t)q0>>32), (unsigned)q0);
            USNATIVE_DEBUG("CK_PROBE p0=%08x:%08x p1=%08x:%08x p2=%08x:%08x p3=%08x:%08x\n",
                (unsigned)((uint64_t)r0>>32), (unsigned)r0,
                (unsigned)((uint64_t)r1>>32), (unsigned)r1,
                (unsigned)((uint64_t)r2>>32), (unsigned)r2,
                (unsigned)((uint64_t)r3>>32), (unsigned)r3);
#endif
        }
#ifdef CONFIG_SDRAM_USNATIVE_DEBUG
        if (nb_seed_probe_kind && i==8 && count==64) {
            unsigned kind=nb_seed_probe_kind;
            unsigned probe_lane=0;
            nb_seed_probe_kind=0;
            while (!(lanes & (1u<<probe_lane))) ++probe_lane;
#if (SDRAM_PHY_DATABITS == 32 && (SDRAM_PHY_PHASES == 4 || SDRAM_PHY_PHASES == 8)) || \
    USNATIVE_REDUCED_X64_1TO8
            unsigned read_command_phase=nb_read_phase;
#else
            unsigned read_command_phase=USNATIVE_RDPHASE;
#endif
            USNATIVE_DEBUG("NATIVE_SEED_PROBE dir=%s lane=%u tap=%u rd_cmd_phase=%u wr_cmd_phase=%u ready=%u fifo=%08x status=%08x\n",
                kind==1 ? "RX" : "TX", probe_lane,
                kind==1 ? ddrphy_tap_rx_count_read() : ddrphy_tap_tx_count_read(),
                read_command_phase, (unsigned)SDRAM_PHY_WRPHASE,
                (unsigned)ddrphy_ready_read(),
                (unsigned)csr_read_simple(CSR_DDRPHY_FIFO_EMPTY_ADDR),
                (unsigned)ddrphy_debug_read());
#if SDRAM_PHY_PHASES == 8 && USNATIVE_DUAL_SLOT_FULL_WIDTH
            USNATIVE_DEBUG("NATIVE_SEED_PROBE expected=%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x observed=%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x\n",
                (unsigned)((uint64_t)q[0]>>32), (unsigned)q[0],
                (unsigned)((uint64_t)q[1]>>32), (unsigned)q[1],
                (unsigned)((uint64_t)q[2]>>32), (unsigned)q[2],
                (unsigned)((uint64_t)q[3]>>32), (unsigned)q[3],
                (unsigned)((uint64_t)r0>>32), (unsigned)r0,
                (unsigned)((uint64_t)r1>>32), (unsigned)r1,
                (unsigned)((uint64_t)r2>>32), (unsigned)r2,
                (unsigned)((uint64_t)r3>>32), (unsigned)r3);
#elif SDRAM_PHY_PHASES == 8
            USNATIVE_DEBUG("NATIVE_SEED_PROBE expected=%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x observed=%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x\n",
                (unsigned)((uint64_t)q[0]>>32), (unsigned)q[0],
                (unsigned)((uint64_t)q[1]>>32), (unsigned)q[1],
                (unsigned)((uint64_t)q[2]>>32), (unsigned)q[2],
                (unsigned)((uint64_t)q[3]>>32), (unsigned)q[3],
                (unsigned)((uint64_t)q[4]>>32), (unsigned)q[4],
                (unsigned)((uint64_t)q[5]>>32), (unsigned)q[5],
                (unsigned)((uint64_t)q[6]>>32), (unsigned)q[6],
                (unsigned)((uint64_t)q[7]>>32), (unsigned)q[7],
                (unsigned)((uint64_t)r0>>32), (unsigned)r0,
                (unsigned)((uint64_t)r1>>32), (unsigned)r1,
                (unsigned)((uint64_t)r2>>32), (unsigned)r2,
                (unsigned)((uint64_t)r3>>32), (unsigned)r3,
                (unsigned)((uint64_t)r4>>32), (unsigned)r4,
                (unsigned)((uint64_t)r5>>32), (unsigned)r5,
                (unsigned)((uint64_t)r6>>32), (unsigned)r6,
                (unsigned)((uint64_t)r7>>32), (unsigned)r7);
#else
            USNATIVE_DEBUG("NATIVE_SEED_PROBE expected=%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x observed=%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x\n",
                (unsigned)((uint64_t)q0>>32), (unsigned)q0,
                (unsigned)((uint64_t)q1>>32), (unsigned)q1,
                (unsigned)((uint64_t)q2>>32), (unsigned)q2,
                (unsigned)((uint64_t)q3>>32), (unsigned)q3,
                (unsigned)((uint64_t)r0>>32), (unsigned)r0,
                (unsigned)((uint64_t)r1>>32), (unsigned)r1,
                (unsigned)((uint64_t)r2>>32), (unsigned)r2,
                (unsigned)((uint64_t)r3>>32), (unsigned)r3);
#endif
        }
#endif
#ifdef CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC
        if (nb_dfi_diag_active && i==8) {
#if SDRAM_PHY_PHASES == 8
            USNATIVE_DEBUG("NATIVE_DFI_TX rdphase=%u wrphase=%u wr_cmd_phase=%u address=%03x chunks=%016llx,%016llx,%016llx,%016llx,%016llx,%016llx,%016llx,%016llx\n",
                nb_read_phase, SDRAM_PHY_WRPHASE, SDRAM_PHY_WRPHASE, write_address,
                (unsigned long long)q[0], (unsigned long long)q[1],
                (unsigned long long)q[2], (unsigned long long)q[3],
                (unsigned long long)q[4], (unsigned long long)q[5],
                (unsigned long long)q[6], (unsigned long long)q[7]);
            USNATIVE_DEBUG("NATIVE_DFI_RX rd_cmd_phase=%u errors=%u chunks=%016llx,%016llx,%016llx,%016llx\n",
                nb_read_phase, (unsigned)((((r0^q[0])|(r1^q[1])|(r2^q[2])|(r3^q[3]))&mask)!=0),
                (unsigned long long)r0, (unsigned long long)r1,
                (unsigned long long)r2, (unsigned long long)r3);
#else
            USNATIVE_DEBUG("NATIVE_DFI_TX rdphase=%u wrphase=%u wr_cmd_phase=%u address=%03x chunks=%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x\n",
                nb_read_phase, nb_write_phase, nb_write_phase, write_address,
                (unsigned)(q0>>32), (unsigned)q0,
                (unsigned)(q1>>32), (unsigned)q1,
                (unsigned)(q2>>32), (unsigned)q2,
                (unsigned)(q3>>32), (unsigned)q3);
            USNATIVE_DEBUG("NATIVE_DFI_RX rd_cmd_phase=%u errors=%u chunks=%08x:%08x,%08x:%08x,%08x:%08x,%08x:%08x\n",
                nb_read_phase, (unsigned)((((r0^q0)|(r1^q1)|(r2^q2)|(r3^q3))&mask)!=0),
                (unsigned)(r0>>32), (unsigned)r0,
                (unsigned)(r1>>32), (unsigned)r1,
                (unsigned)(r2>>32), (unsigned)r2,
                (unsigned)(r3>>32), (unsigned)r3);
#endif
#ifdef CONFIG_SDRAM_USNATIVE_WRITE_MONITOR
            USNATIVE_DEBUG("NATIVE_DFI_FAST_TX phase3_cmd=%02x address=%05x wren=%u oe=%u dqs=%u not_ready=%u chunks=%08x,%08x,%08x,%08x,%08x,%08x\n",
                (unsigned)ddrphy_probe_fast_command_read(),
                (unsigned)ddrphy_probe_fast_address_read(),
                (unsigned)ddrphy_probe_fast_wren_read(),
                (unsigned)ddrphy_probe_fast_oe_read(),
                (unsigned)ddrphy_probe_dqs_drive_read(),
                (unsigned)ddrphy_probe_not_ready_read(),
                (unsigned)ddrphy_probe_word0_read(), (unsigned)ddrphy_probe_word1_read(),
                (unsigned)ddrphy_probe_word2_read(), (unsigned)ddrphy_probe_word3_read(),
                (unsigned)ddrphy_probe_word4_read(), (unsigned)ddrphy_probe_word5_read());
#ifdef CSR_DDRPHY_PROBE_SNAPSHOT_WORD0_ADDR
            /* Word groups: recent CA history, all current CA phases, then
             * all write data/mask/enable phases. Dump raw words so no 64-bit
             * printf implementation can hide the upper DFI data half. */
            USNATIVE_DEBUG("NATIVE_DFI_SNAPSHOT valid=%u count=%u command=%02x address=%05x\n",
                (unsigned)ddrphy_probe_snapshot_valid_read(),
                (unsigned)ddrphy_probe_snapshot_count_read(),
                (unsigned)ddrphy_probe_fast_command_read(),
                (unsigned)ddrphy_probe_fast_address_read());
#ifdef CSR_DDRPHY_PROBE_SNAPSHOT_WORD17_ADDR
            const unsigned snapshot_words=18;
#else
            const unsigned snapshot_words=13;
#endif
            for (unsigned word=0; word<snapshot_words; ++word)
                USNATIVE_DEBUG("NATIVE_DFI_SNAPSHOT_WORD index=%u value=%08x\n",
                    word,(unsigned)csr_read_simple(CSR_DDRPHY_PROBE_SNAPSHOT_WORD0_ADDR+4*word));
#endif
#endif
        }
#endif
#if SDRAM_PHY_PHASES == 8 && !USNATIVE_DUAL_SLOT_FULL_WIDTH
        usnative_dfi_word_t wrong=(r0^q[0]) | (r1^q[1]) | (r2^q[2]) | (r3^q[3]) |
                                  (r4^q[4]) | (r5^q[5]) | (r6^q[6]) | (r7^q[7]);
#ifdef CONFIG_USNATIVE_VERBOSE_PROBE
        USNATIVE_DEBUG("NATIVE_PROBE epoch=%u match=%u\n", epoch, (wrong&mask)==0);
#endif
        if (i>=8 && (wrong&mask)) ++errors;
#elif SDRAM_PHY_PHASES == 8
        usnative_dfi_word_t wrong=(r0^q[0]) | (r1^q[1]) | (r2^q[2]) | (r3^q[3]);
#ifdef CONFIG_USNATIVE_VERBOSE_PROBE
        USNATIVE_DEBUG("NATIVE_PROBE epoch=%u match=%u\n", epoch, (wrong&mask)==0);
#endif
        if (i>=8 && (wrong&mask)) ++errors;
#else
        usnative_dfi_word_t wrong=(r0^q0) | (r1^q1) | (r2^q2) | (r3^q3);
        if (i>=8 && (wrong&mask)) ++errors;
#endif
        sdram_dfii_pi0_address_write(1<<10);
        command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS); cdelay(1000);
    }
    return errors;
}

#ifdef CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC
/* Capture one known BL8 write, then read it at each DFI command phase. The
 * first eight bursts flush startup FIFO contents; only the ninth is reported.
 * Normal calibration starts from the original read phase and tap seeds. */
static int nb_dfi_diagnostic(void)
{
#if SDRAM_PHY_DATABITS == 32 && \
    ((SDRAM_PHY_PHASES == 8 && USNATIVE_DUAL_SLOT_FULL_WIDTH) || \
     (SDRAM_PHY_PHASES == 4 && SDRAM_PHY_DFI_DATABITS == 64 && \
      CONFIG_CLOCK_FREQUENCY == 200000000 && \
      SDRAM_PHY_CL == 11 && SDRAM_PHY_CWL == 9 && \
      SDRAM_PHY_RDPHASE == 0 && SDRAM_PHY_WRPHASE == 2))
    unsigned saved_phase=nb_read_phase;
#if SDRAM_PHY_PHASES == 4
    unsigned saved_write_phase=nb_write_phase;
    printf("Native DFI diagnostic: one BL8 write per write/read phase pair (4x4); calibration follows.\n");
#else
    printf("Native DFI diagnostic: one BL8 write, read phases 0..3; calibration follows.\n");
#endif
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane) {
        nb_seed_rx[lane]=USNATIVE_BOOT_RX_DELAY;
        nb_seed_tx[lane]=USNATIVE_BOOT_TX_DELAY;
    }
    nb_dfi_diag_active=0;
    nb_read_phase=USNATIVE_RDPHASE;
    if (!nb_boot(0)) goto restore;
#ifdef CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC
    /* MPR runs after a fresh init at the known bootstrap taps. The probe
     * restores MR3 to the EDY4016A 1x-refresh init value before this array
     * write/read phase sweep starts. A data failure is diagnostic only. */
#ifdef CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP
    unsigned mpr_gate_errors=nb_mpr_lane_by_lane_gate_sweep();
#ifdef CONFIG_SDRAM_USNATIVE_MPR_RX_TAP_SCAN
    if (mpr_gate_errors && !nb_mpr_rx_tap_scan()) {
        printf("Native DFI diagnostic stopped: MPR RX scan could not confirm safe restoration.\n");
#ifdef CONFIG_SDRAM_USNATIVE_MPR_2D_SCAN
        /* The 2D probe asserted PHY reset after a failed restore. Propagate
         * failure to the caller rather than running nb_boot(0), which could
         * release that reset and hide the unsafe diagnostic state. */
        return 0;
#else
        goto restore;
#endif
    }
#endif
#else
    (void)nb_mpr_diagnostic();
#endif
    /* On AES-KU40 DDR4-1600, the bounded MPR sweep saw its largest
     * receive window at gate 2. Test array BL8 commands there without
     * changing the production calibration seed; nb_boot(0) restores the
     * nominal gate after this diagnostic phase sweep. */
    nb_set_all_gates(2);
    printf("Native DFI diagnostic: MPR-informed gate=2 for BL8 phase sweep.\n");
#endif
    nb_dfi_diag_active=1;
#ifdef CONFIG_SDRAM_USNATIVE_FIXED_ROW_DIAGNOSTIC
    (void)nb_fixed_row_diagnostic();
#endif
#ifdef CONFIG_SDRAM_USNATIVE_TAGGED_READ_DIAGNOSTIC
    (void)nb_tagged_read_diagnostic();
#endif
#if SDRAM_PHY_PHASES == 4
    for (unsigned write_phase=0; write_phase<4; ++write_phase) {
        nb_write_phase=write_phase;
        ddrphy_wrphase_write(write_phase);
        cdelay(100);
        for (unsigned phase=0; phase<4; ++phase) {
            nb_read_phase=phase;
            ddrphy_rdphase_write(phase);
            unsigned errors=nb_check(1,USNATIVE_LANE_MASK);
            printf("Native DFI diagnostic result: wr_cmd_phase=%u rd_cmd_phase=%u errors=%u\n",
                write_phase,phase,errors);
        }
    }
#else
    for (unsigned phase=0; phase<4; ++phase) {
        nb_read_phase=phase;
        ddrphy_rdphase_write(phase);
        unsigned errors=nb_check(1,USNATIVE_LANE_MASK);
        printf("Native DFI diagnostic result: rd_cmd_phase=%u errors=%u\n",phase,errors);
    }
#endif
    nb_dfi_diag_active=0;
restore:
    nb_dfi_diag_active=0;
    nb_read_phase=saved_phase;
    ddrphy_rdphase_write(saved_phase);
#if SDRAM_PHY_PHASES == 4
    nb_write_phase=saved_write_phase;
    ddrphy_wrphase_write(saved_write_phase);
#endif
    cdelay(100);
    if (!nb_boot(0)) {
        printf("Native DFI diagnostic could not restore nominal boot state.\n");
        return 0;
    }
    printf("Native DFI diagnostic complete; nominal read phase and boot taps restored.\n");
    return 1;
#else
    printf("Native DFI diagnostic requires x32 1:4 1600 or dual-slot 1:8 DFI.\n");
    return 0;
#endif
}
#endif

static int nb_center(unsigned lane, int transmit, struct nb_window *window)
{
    unsigned start=transmit ? 32 : 0, end=transmit ? USNATIVE_TX_SCAN_END : 128;
    unsigned step=transmit ? 2 : 4;
    unsigned required=32/step+1;
    unsigned run=0, best=0, best_end=0;
    const char *direction=transmit ? "TX" : "RX";
    window->short_window=0;
    for (unsigned tap=start; tap<=end; tap+=step) {
        if (!nb_delay(lane,tap,transmit)) {
            printf("Native %s delay failed: lane=%u tap=%u\n",direction,lane,tap);
            return 0;
        }
        if (nb_check(128,1<<lane)) run=0; else ++run;
        if (run>best) { best=run; best_end=tap; }
    }
    window->first=best ? best_end-step*(best-1) : 0;
    window->last=best_end;
    window->center=best ? window->first+step*((best-1)/2) : 0;
    if (best<required) { /* Require at least 32 taps of measured width. */
        window->short_window=1;
        printf("Native %s window too short: lane=%u samples=%u required=%u step=%u first=%u last=%u\n",
            direction,lane,best,required,step,window->first,window->last);
        return 0;
    }
    if (!nb_delay(lane,window->center,transmit)) {
        printf("Native %s center delay failed: lane=%u center=%u\n",direction,lane,window->center);
        return 0;
    }
    unsigned errors=nb_check(256,1<<lane);
    USNATIVE_DEBUG("NATIVE_WINDOW direction=%s lane=%u first=%u last=%u center=%u errors=%u\n",
        direction,lane,window->first,window->last,window->center,errors);
    if (errors) {
        printf("Native %s center burst failed: lane=%u center=%u errors=%u\n",
            direction,lane,window->center,errors);
        return 0;
    }
    return 1;
}

/* At 3200 or on an x32 1:8 controller, a read sampling point can limit the
 * observed write window.
 * Retry only an undersized window, at four nearby RX points inside its measured
 * bounds. Accept an intersection of two complete TX scans, never a relaxed
 * width or a failed delay/center confirmation. Other profiles are unchanged. */
static int nb_center_tx(unsigned lane, struct nb_window *rx, struct nb_window *tx)
{
    if (nb_center(lane,1,tx)) return 1;
#if CONFIG_CLOCK_FREQUENCY == 400000000 || (SDRAM_PHY_DATABITS == 32 && SDRAM_PHY_PHASES == 8)
    if (!tx->short_window || !tx->center) return 0;
    /* Keep the best measured TX point while probing a different RX point.
     * The boot TX value may be far outside this lane's observed write eye. */
    const unsigned tx_probe=tx->center;
    const int offsets[4]={4,8,-4,-8};
    for (unsigned attempt=0; attempt<4; ++attempt) {
        int candidate=(int)rx->center+offsets[attempt];
        if (candidate<(int)rx->first+4 || candidate>(int)rx->last-4) continue;
        printf("Native TX window retry: lane=%u attempt=%u RX=%d\n",lane,attempt+1,candidate);
        if (!nb_delay(lane,tx_probe,1) ||
            !nb_delay(lane,(unsigned)candidate,0)) return 0;
        if (nb_check(256,1<<lane)) continue;
        if (!nb_center(lane,1,tx)) {
            if (!tx->short_window) return 0;
            continue;
        }
        struct nb_window confirm;
        if (!nb_center(lane,1,&confirm)) {
            if (!confirm.short_window) return 0;
            continue;
        }
        unsigned first=tx->first>confirm.first ? tx->first : confirm.first;
        unsigned last=tx->last<confirm.last ? tx->last : confirm.last;
        if (last<first || last-first<32) continue;
        tx->first=first; tx->last=last; tx->center=first+4*((last-first)/8);
        if (!nb_delay(lane,tx->center,1) || nb_check(256,1<<lane)) return 0;
        rx->center=(unsigned)candidate;
        printf("Native TX retry accepted: lane=%u RX=%u TX=[%u..%u] center=%u\n",
            lane,rx->center,tx->first,tx->last,tx->center);
        return 1;
    }
#endif
    return 0;
}

/* Return zero only after margin searches and a VTC-enabled burst check.
 * DFI remains under software ownership; controller memtest is a separate
 * acceptance step performed by the caller after successful calibration. */
static unsigned nb_calibrate(struct nb_result *result)
{
    unsigned best=0, best_end=0;
    unsigned ck_scan_end=128;
#if !(SDRAM_PHY_DATABITS == 32 && (SDRAM_PHY_PHASES == 4 || SDRAM_PHY_PHASES == 8)) && \
    !USNATIVE_REDUCED_X64_1TO8
    unsigned run=0;
#endif
    unsigned best_gate=USNATIVE_GATE_DELAY;
    unsigned best_phase=USNATIVE_RDPHASE;
    ddrphy_training_error_write(0); ddrphy_training_stage_write(3);
#if (SDRAM_PHY_DATABITS == 32 && (SDRAM_PHY_PHASES == 4 || SDRAM_PHY_PHASES == 8)) || \
    USNATIVE_REDUCED_X64_1TO8
#if SDRAM_PHY_DATABITS == 32
    ck_scan_end=192;
#endif
    nb_read_phase=USNATIVE_RDPHASE;
#endif
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane) {
        nb_seed_rx[lane]=USNATIVE_BOOT_RX_DELAY;
        nb_seed_tx[lane]=USNATIVE_BOOT_TX_DELAY;
    }
#if SDRAM_PHY_DATABITS == 32 || USNATIVE_REDUCED_X64_1TO8
    /* On a wider board the fixed XEM8320 RX seed may put one byte outside
     * its eye. Find an independently readable coarse point in each lane
     * before requiring a clean aggregate CK window. The later full RX/TX
     * searches and memory test remain mandatory. */
    if (!nb_boot(0)) return nb_fail(1);
#if defined(CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC) && USNATIVE_REDUCED_X64_1TO8
    /* MPR is a read-path discriminator only. Always reset/reinitialize after
     * it, even on a match, before touching the array or continuing training. */
    (void)nb_mpr_x64_probe();
    if (!nb_boot(0)) return nb_fail(1);
#endif
    unsigned rx_seed_min[USNATIVE_LANES], tx_seed_min[USNATIVE_LANES];
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane) {
        rx_seed_min[lane]=~0u; tx_seed_min[lane]=~0u;
    }
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane) {
        unsigned current=0, longest=0, longest_end=0;
        unsigned lowest_errors=~0u, lowest_tap=USNATIVE_BOOT_RX_DELAY;
        for (unsigned tap=0; tap<=96; tap+=8) {
            if (!nb_delay(lane,tap,0)) return nb_fail(1);
#ifdef CONFIG_SDRAM_USNATIVE_DEBUG
            if (lane==0 && tap==USNATIVE_BOOT_RX_DELAY) nb_seed_probe_kind=1;
#endif
            unsigned errors=nb_check(64,1u<<lane);
            if (errors<rx_seed_min[lane]) rx_seed_min[lane]=errors;
            USNATIVE_DEBUG("RX_SEED lane=%u tap=%u errors=%u\n",lane,tap,errors);
            if (errors<lowest_errors) { lowest_errors=errors; lowest_tap=tap; }
            current=errors ? 0 : current+1;
            if (current>longest) { longest=current; longest_end=tap; }
        }
        unsigned candidate=longest ? longest_end-8*((longest-1)/2) : lowest_tap;
        nb_seed_rx[lane]=usnative_coarse_seed_select(nb_seed_rx[lane],candidate,
            lowest_errors,64);
        printf("Native RX seed: lane=%u tap=%u clean_samples=%u min_errors=%u%s\n",
            lane,nb_seed_rx[lane],longest,lowest_errors,
            lowest_errors>=64 ? " (retained prior seed)" : "");
        if (!nb_delay(lane,nb_seed_rx[lane],0)) return nb_fail(1);
    }
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane) {
        unsigned current=0, longest=0, longest_end=0;
        unsigned lowest_errors=~0u, lowest_tap=USNATIVE_BOOT_TX_DELAY;
        for (unsigned tap=32; tap<=176; tap+=8) {
            if (!nb_delay(lane,tap,1)) return nb_fail(1);
#ifdef CONFIG_SDRAM_USNATIVE_DEBUG
            if (lane==0 && tap==USNATIVE_BOOT_TX_DELAY) nb_seed_probe_kind=2;
#endif
            unsigned errors=nb_check(64,1u<<lane);
            if (errors<tx_seed_min[lane]) tx_seed_min[lane]=errors;
            USNATIVE_DEBUG("TX_SEED lane=%u tap=%u errors=%u\n",lane,tap,errors);
            if (errors<lowest_errors) { lowest_errors=errors; lowest_tap=tap; }
            current=errors ? 0 : current+1;
            if (current>longest) { longest=current; longest_end=tap; }
        }
        unsigned candidate=longest ? longest_end-4*(longest-1) : lowest_tap;
        nb_seed_tx[lane]=usnative_coarse_seed_select(nb_seed_tx[lane],candidate,
            lowest_errors,64);
        printf("Native TX seed: lane=%u tap=%u clean_samples=%u min_errors=%u%s\n",
            lane,nb_seed_tx[lane],longest,lowest_errors,
            lowest_errors>=64 ? " (retained prior seed)" : "");
        if (!nb_delay(lane,nb_seed_tx[lane],1)) return nb_fail(1);
    }
#ifdef CONFIG_SDRAM_USNATIVE_JOINT_SEED
#if SDRAM_PHY_DATABITS == 32 && SDRAM_PHY_PHASES == 8
    /* AES-KU40 1:8 bootstrap escape hatch: only search a lane's bounded
     * 13x19 coarse grid if both coordinate scans found zero partial reads.
     * Every candidate is scored with both taps programmed together. */
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane) {
        if (rx_seed_min[lane]<64 || tx_seed_min[lane]<64) continue;
        struct usnative_joint_seed best={0,0,0,0};
        for (unsigned rx=0; rx<=96; rx+=8) {
            for (unsigned tx=32; tx<=176; tx+=8) {
                if (!nb_delay(lane,rx,0) || !nb_delay(lane,tx,1)) return nb_fail(1);
                unsigned errors=nb_check(64,1u<<lane);
                usnative_joint_seed_consider(&best,rx,tx,errors,
                    USNATIVE_BOOT_RX_DELAY,USNATIVE_BOOT_TX_DELAY);
            }
        }
        printf("Native joint seed sweep: lane=%u points=247 best_RX=%u best_TX=%u errors=%u\n",
            lane,best.rx,best.tx,best.errors);
        if (!best.valid || best.errors>=64) {
            printf("Native joint seed failed: lane=%u no partial read in RX=[0..96]/8 TX=[32..176]/8\n",lane);
            return nb_fail(1);
        }
        nb_seed_rx[lane]=best.rx; nb_seed_tx[lane]=best.tx;
        printf("Native joint seed accepted: lane=%u RX=%u TX=%u errors=%u\n",
            lane,best.rx,best.tx,best.errors);
        if (!nb_delay(lane,best.rx,0) || !nb_delay(lane,best.tx,1)) return nb_fail(1);
    }
#endif /* x32 1:8 only */
#endif
#endif
#if (SDRAM_PHY_DATABITS == 32 && (SDRAM_PHY_PHASES == 4 || SDRAM_PHY_PHASES == 8)) || \
    USNATIVE_REDUCED_X64_1TO8
    /* Preserve the preferred default, default-1, default+1 order, then
     * include default-2/-3 to reach early capture apertures. Gate CSRs are
     * 5-bit values; omit invalid candidates at profile boundaries. */
    const int gate_offsets[5]={0,-1,1,-2,-3};
    unsigned gate_candidates[sizeof(gate_offsets)/sizeof(gate_offsets[0])];
    unsigned gate_candidate_count=0;
    for (unsigned i=0; i<sizeof(gate_offsets)/sizeof(gate_offsets[0]); ++i) {
        int gate=(int)USNATIVE_GATE_DELAY+gate_offsets[i];
        if (gate<0 || gate>31) continue;
        gate_candidates[gate_candidate_count++]=(unsigned)gate;
    }
    unsigned phase_run[4][sizeof(gate_offsets)/sizeof(gate_offsets[0])]={{0}};
    unsigned phase_best[4][sizeof(gate_offsets)/sizeof(gate_offsets[0])]={{0}};
    unsigned phase_end[4][sizeof(gate_offsets)/sizeof(gate_offsets[0])]={{0}};
    unsigned phase_min[4][sizeof(gate_offsets)/sizeof(gate_offsets[0])];
    unsigned phase_min_ck[4][sizeof(gate_offsets)/sizeof(gate_offsets[0])]={{0}};
    for (unsigned seed_attempt=0; seed_attempt<3; ++seed_attempt) {
        best=0; best_end=0; best_phase=USNATIVE_RDPHASE;
        best_gate=USNATIVE_GATE_DELAY;
        for (unsigned phase=0; phase<4; ++phase) {
            for (unsigned slot=0; slot<gate_candidate_count; ++slot) {
                phase_run[phase][slot]=0;
                phase_best[phase][slot]=0;
                phase_end[phase][slot]=0;
                phase_min[phase][slot]=~0u;
                phase_min_ck[phase][slot]=0;
            }
        }
        /* Each phase gets a fresh DDR/BISC initialization. A failed read phase
         * can leave native FIFO words behind, so reusing one boot for all phases
         * makes later candidates incomparable. Probe the known phase first. */
        for (unsigned order=0; order<4; ++order) {
            unsigned phase=order ? order-1 : USNATIVE_RDPHASE;
            if (order && phase>=USNATIVE_RDPHASE) ++phase;
            nb_read_phase=phase;
            /* x32 uses its established 0..192 sweep; reduced x64 keeps the
             * conservative 0..128 range while adding phase/gate candidates. */
            for (unsigned ck=0; ck<=ck_scan_end; ck+=8) {
                if (!nb_boot(ck)) {
                    printf("Native CK initialization failed: phase=%u tap=%u\n",phase,ck);
                    return nb_fail(1);
                }
                for (unsigned gate_index=0; gate_index<gate_candidate_count; ++gate_index) {
                    unsigned slot=gate_index;
                    unsigned gate=gate_candidates[slot];
                    nb_set_all_gates(gate);
                    unsigned errors=nb_check(256,USNATIVE_LANE_MASK);
                    USNATIVE_DEBUG("NATIVE_CK seed_attempt=%u phase=%u gate=%u tap=%u errors=%u\n",
                        seed_attempt,phase,gate,ck,errors);
                    if (errors<phase_min[phase][slot]) {
                        phase_min[phase][slot]=errors;
                        phase_min_ck[phase][slot]=ck;
                    }
                    if (errors) phase_run[phase][slot]=0; else ++phase_run[phase][slot];
                    if (phase_run[phase][slot]>phase_best[phase][slot]) {
                        phase_best[phase][slot]=phase_run[phase][slot];
                        phase_end[phase][slot]=ck;
                    }
                }
            }
        }
        for (unsigned phase=0; phase<4; ++phase) {
            for (unsigned slot=0; slot<gate_candidate_count; ++slot) {
                unsigned gate=gate_candidates[slot];
                printf("Native CK phase %u gate %u: clean_samples=%u last=%u min_errors=%u at=%u RX_seed_attempt=%u\n",
                    phase,gate,phase_best[phase][slot],phase_end[phase][slot],
                    phase_min[phase][slot],phase_min_ck[phase][slot],seed_attempt);
                if (phase_best[phase][slot]>best) {
                    best=phase_best[phase][slot]; best_end=phase_end[phase][slot];
                    best_gate=gate; best_phase=phase;
                }
            }
        }
        if (best>=5 || seed_attempt==2) break;
        unsigned fallback=usnative_rx_seed_fallback(nb_seed_rx,USNATIVE_LANES,
            seed_attempt,0,96,8);
        printf("Native CK retry: no five-point window; applying common RX seed %u to %u lanes (attempt %u)\n",
            fallback,USNATIVE_LANES,seed_attempt+1);
        for (unsigned lane=0; lane<USNATIVE_LANES; ++lane)
            nb_seed_rx[lane]=fallback;
        /* The previous phase sweep leaves nb_read_phase at its last candidate.
         * Probe retry TX seeds from the known operating phase instead. */
        nb_read_phase=USNATIVE_RDPHASE;
        if (!nb_boot(0)) return nb_fail(1);
        /* Re-estimate TX seeds at the fallback RX point before repeating CK. */
        for (unsigned lane=0; lane<USNATIVE_LANES; ++lane) {
            unsigned current=0, longest=0, longest_end=0;
            unsigned lowest_errors=~0u, lowest_tap=USNATIVE_BOOT_TX_DELAY;
            for (unsigned tap=32; tap<=176; tap+=8) {
                if (!nb_delay(lane,tap,1)) return nb_fail(1);
                unsigned errors=nb_check(64,1u<<lane);
                if (errors<lowest_errors) { lowest_errors=errors; lowest_tap=tap; }
                current=errors ? 0 : current+1;
                if (current>longest) { longest=current; longest_end=tap; }
            }
            unsigned candidate=longest ? longest_end-4*(longest-1) : lowest_tap;
            nb_seed_tx[lane]=usnative_tx_seed_select(nb_seed_tx[lane],candidate,longest);
            printf("Native TX seed retry: lane=%u tap=%u clean_samples=%u min_errors=%u%s\n",
                lane,nb_seed_tx[lane],longest,lowest_errors,
                longest ? "" : " (retained prior seed)");
            if (!nb_delay(lane,nb_seed_tx[lane],1)) return nb_fail(1);
        }
    }
#elif SDRAM_PHY_DATABITS == 32
    unsigned gate_run[3]={0}, gate_best[3]={0}, gate_end[3]={0};
#endif
#if !(SDRAM_PHY_DATABITS == 32 && (SDRAM_PHY_PHASES == 4 || SDRAM_PHY_PHASES == 8)) && \
    !USNATIVE_REDUCED_X64_1TO8
    for (unsigned ck=0; ck<=128; ck+=8) {
        if (!nb_boot(ck)) {
            printf("Native CK initialization failed: tap=%u\n",ck);
            return nb_fail(1);
        }
#if SDRAM_PHY_DATABITS == 32
        /* Moving CK can move the received DQS burst across a fixed gate
         * boundary. Search the neighboring gate cycles before rejecting CK.
         * Keep the five-point requirement for a single gate setting. */
        for (unsigned slot=0; slot<3; ++slot) {
            unsigned gate=USNATIVE_GATE_DELAY+slot-1;
            nb_set_all_gates(gate);
            unsigned errors=nb_check(256,USNATIVE_LANE_MASK);
            USNATIVE_DEBUG("NATIVE_CK gate=%u tap=%u errors=%u\n",gate,ck,errors);
            if (errors) gate_run[slot]=0; else ++gate_run[slot];
            if (gate_run[slot]>gate_best[slot]) {
                gate_best[slot]=gate_run[slot]; gate_end[slot]=ck;
            }
        }
#else
        unsigned errors=nb_check(256,USNATIVE_LANE_MASK);
        USNATIVE_DEBUG("NATIVE_CK tap=%u errors=%u\n",ck,errors);
        if (errors) run=0; else ++run;
        if (run>best) { best=run; best_end=ck; }
#endif
    }
#endif
#if SDRAM_PHY_DATABITS == 32 && !(SDRAM_PHY_PHASES == 4 || SDRAM_PHY_PHASES == 8)
    for (unsigned slot=0; slot<3; ++slot) {
        printf("Native CK gate %u: clean_samples=%u last=%u\n",
            USNATIVE_GATE_DELAY+slot-1,gate_best[slot],gate_end[slot]);
        if (gate_best[slot]>best) {
            best=gate_best[slot]; best_end=gate_end[slot];
            best_gate=USNATIVE_GATE_DELAY+slot-1;
        }
    }
#endif
    if (best<5) {
        printf("Native stage3 CK_NO_WINDOW: scan=[0..%u] step=8 phase=%u gate=%u "
            "clean_samples=%u last=%u required=5; no passing window found (center confirmation not attempted)\n",
            ck_scan_end,best_phase,best_gate,best,best_end);
        return nb_fail(6);
    }
    result->phase=best_phase;
    result->gate=best_gate;
    result->ck.first=best_end-8*(best-1); result->ck.last=best_end;
    result->ck.center=result->ck.first+8*((best-1)/2);
#if (SDRAM_PHY_DATABITS == 32 && (SDRAM_PHY_PHASES == 4 || SDRAM_PHY_PHASES == 8)) || \
    USNATIVE_REDUCED_X64_1TO8
    nb_read_phase=result->phase;
#endif
    if (!nb_boot(result->ck.center)) {
        printf("Native CK center initialization failed: window=[%u..%u] center=%u\n",
            result->ck.first,result->ck.last,result->ck.center);
        return nb_fail(6);
    }
    nb_set_all_gates(result->gate);
    unsigned errors=nb_check(256,USNATIVE_LANE_MASK);
    if (errors) {
        printf("Native CK center burst failed: window=[%u..%u] center=%u errors=%u\n",
            result->ck.first,result->ck.last,result->ck.center,errors);
        return nb_fail(6);
    }
    ddrphy_training_stage_write(4);
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane)
        if (!nb_center(lane,0,&result->rx[lane])) return nb_fail(7);
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane)
        if (!nb_center_tx(lane,&result->rx[lane],&result->dq[lane])) return nb_fail(8);
    if (nb_check(4096,USNATIVE_LANE_MASK)) return nb_fail(9);
    ddrphy_gate_override_write(0); ddrphy_en_vtc_write(1);
    unsigned i;
    for (i=0; i<100000; ++i) {
        if (ddrphy_ready_read() && ddrphy_vtc_rdy_read()==USNATIVE_CONTROL_MASK) break;
        cdelay(100);
    }
    if (i==100000) return nb_fail(5);
    if (nb_check(256,USNATIVE_LANE_MASK)) return nb_fail(9);
    ddrphy_training_stage_write(5); USNATIVE_SNAPSHOT();
    return 0;
}
#endif
