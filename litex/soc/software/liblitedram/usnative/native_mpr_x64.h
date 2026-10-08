/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Read-only DDR4 MPR0 discriminator for reduced-width native x64 1:8.
 * The caller runs after normal init while DFII is software-owned, then must
 * reinitialize the PHY/DRAM before any array calibration or memory traffic.
 * This probe never writes the DRAM array or MPR contents and does not alter
 * calibration admission. */
#ifndef USNATIVE_NATIVE_MPR_X64_H
#define USNATIVE_NATIVE_MPR_X64_H

#if defined(CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC)

#if !defined(SDRAM_PHY_DDR4) || SDRAM_PHY_DATABITS != 64 || \
    SDRAM_PHY_DFI_DATABITS != 64 || SDRAM_PHY_PHASES != 8 || \
    !USNATIVE_REDUCED_X64_1TO8
#error "x64 MPR diagnostic requires reduced DDR4 x64 1:8 (eight 64-bit phases)"
#endif

#ifndef CSR_DDRPHY_FIFO_EMPTY_ADDR
#error "x64 MPR diagnostic requires the native FIFO_EMPTY status CSR"
#endif

#include <stdint.h>

#define NB_MPR_X64_NORMAL_MR3 0x0000u /* generated BCU profile: 1x refresh */
#define NB_MPR_X64_ENABLE_MR3 0x0004u /* MR3 A2=1, page 0, serial format */
#define NB_MPR_X64_GATE_FIRST 0u
#define NB_MPR_X64_GATE_LAST  5u
#define NB_MPR_X64_SETTLE     100u
#define NB_MPR_X64_WARMUP     4u

static uint64_t nb_mpr_x64_rddata(unsigned phase)
{
    switch (phase) {
    case 0: return sdram_dfii_pi0_rddata_read();
    case 1: return sdram_dfii_pi1_rddata_read();
    case 2: return sdram_dfii_pi2_rddata_read();
    case 3: return sdram_dfii_pi3_rddata_read();
    case 4: return sdram_dfii_pi4_rddata_read();
    case 5: return sdram_dfii_pi5_rddata_read();
    case 6: return sdram_dfii_pi6_rddata_read();
    default:return sdram_dfii_pi7_rddata_read();
    }
}

static unsigned nb_mpr_x64_gate_delay_read(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_gate_delay0_read();
    case 1: return ddrphy_gate_delay1_read();
    case 2: return ddrphy_gate_delay2_read();
    case 3: return ddrphy_gate_delay3_read();
    case 4: return ddrphy_gate_delay4_read();
    case 5: return ddrphy_gate_delay5_read();
    case 6: return ddrphy_gate_delay6_read();
    default:return ddrphy_gate_delay7_read();
    }
}

static unsigned nb_mpr_x64_gate_width_read(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_gate_width0_read();
    case 1: return ddrphy_gate_width1_read();
    case 2: return ddrphy_gate_width2_read();
    case 3: return ddrphy_gate_width3_read();
    case 4: return ddrphy_gate_width4_read();
    case 5: return ddrphy_gate_width5_read();
    case 6: return ddrphy_gate_width6_read();
    default:return ddrphy_gate_width7_read();
    }
}

static void nb_mpr_x64_gate_write(unsigned lane, unsigned delay, unsigned width)
{
    switch (lane) {
    case 0: ddrphy_gate_delay0_write(delay); ddrphy_gate_width0_write(width); break;
    case 1: ddrphy_gate_delay1_write(delay); ddrphy_gate_width1_write(width); break;
    case 2: ddrphy_gate_delay2_write(delay); ddrphy_gate_width2_write(width); break;
    case 3: ddrphy_gate_delay3_write(delay); ddrphy_gate_width3_write(width); break;
    case 4: ddrphy_gate_delay4_write(delay); ddrphy_gate_width4_write(width); break;
    case 5: ddrphy_gate_delay5_write(delay); ddrphy_gate_width5_write(width); break;
    case 6: ddrphy_gate_delay6_write(delay); ddrphy_gate_width6_write(width); break;
    default: ddrphy_gate_delay7_write(delay); ddrphy_gate_width7_write(width); break;
    }
}

static unsigned nb_mpr_x64_fifo_reads(unsigned lane)
{
    switch (lane) {
    case 0: return ddrphy_fifo_reads0_read();
    case 1: return ddrphy_fifo_reads1_read();
    case 2: return ddrphy_fifo_reads2_read();
    case 3: return ddrphy_fifo_reads3_read();
    case 4: return ddrphy_fifo_reads4_read();
    case 5: return ddrphy_fifo_reads5_read();
    case 6: return ddrphy_fifo_reads6_read();
    default:return ddrphy_fifo_reads7_read();
    }
}

static void nb_mpr_x64_fifo_empty(uint32_t words[4])
{
    /* Compound CSR address order is most-significant word first. */
    for (unsigned word=0; word<4; ++word)
        words[word]=csr_read_simple(CSR_DDRPHY_FIFO_EMPTY_ADDR + 4*word);
}

static int nb_mpr_x64_signature(const uint64_t words[8])
{
    /* Micron DDR4 MPR0 page 0 is 01010101 (A7..A0), serial UI0..UI7.
     * Unknown capture phase can invert the common starting parity, but all
     * 64 DQ bits must still alternate on every adjacent external DFI phase. */
    for (unsigned first=0; first<2; ++first) {
        unsigned match=1;
        for (unsigned phase=0; phase<8 && match; ++phase) {
            const unsigned expected=first ^ (phase & 1u);
            for (unsigned dq=0; dq<64; ++dq) {
                if (((words[phase] >> dq) & 1u) != expected) {
                    match=0;
                    break;
                }
            }
        }
        if (match) return 1;
    }
    return 0;
}

static void nb_mpr_x64_mrs(unsigned mr3)
{
    sdram_dfii_pi0_address_write(mr3);
    sdram_dfii_pi0_baddress_write(3); /* Mode Register 3 */
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
    cdelay(NB_MPR_X64_SETTLE); /* >= tMRD/tMOD at the supported controller rate */
}

static void nb_mpr_x64_issue_read(unsigned command_phase, uint64_t words[8])
{
    sdram_dfii_pix_address_write((unsigned char)command_phase, 0); /* BL8 A[2:0]=000 */
    sdram_dfii_pix_baddress_write((unsigned char)command_phase, 0); /* page 0, MPR0 */
    command_px((unsigned char)command_phase,
        DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA);
    cdelay(NB_MPR_X64_SETTLE);
    for (unsigned phase=0; phase<8; ++phase)
        words[phase]=nb_mpr_x64_rddata(phase);
}

/* Diagnostic only. Return 1 if any gate/command-phase candidate yields the
 * strict x64 MPR0 signature. The caller must always reset/reinitialize before
 * array training, regardless of this return value. */
static int nb_mpr_x64_probe(void)
{
    unsigned saved_delay[8], saved_width[8];
    int signature_seen=0;

    for (unsigned lane=0; lane<8; ++lane) {
        saved_delay[lane]=nb_mpr_x64_gate_delay_read(lane);
        saved_width[lane]=nb_mpr_x64_gate_width_read(lane);
    }

    /* PRECHARGE ALL before MR3 MPR access mode; this does not alter array data. */
    sdram_dfii_pi0_address_write(1u<<10);
    sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
    cdelay(NB_MPR_X64_SETTLE);
    nb_mpr_x64_mrs(NB_MPR_X64_ENABLE_MR3);

    printf("DDR4 x64 MPR0 diagnostic: gate=%u..%u command_phase=0..7, no array writes\n",
        NB_MPR_X64_GATE_FIRST, NB_MPR_X64_GATE_LAST);
    for (unsigned gate=NB_MPR_X64_GATE_FIRST; gate<=NB_MPR_X64_GATE_LAST; ++gate) {
        for (unsigned lane=0; lane<8; ++lane)
            nb_mpr_x64_gate_write(lane, gate, saved_width[lane]);
        for (unsigned command_phase=0; command_phase<8; ++command_phase) {
            uint64_t words[8];
            unsigned before[8];
            uint32_t empty[4];
            for (unsigned lane=0; lane<8; ++lane)
                before[lane]=nb_mpr_x64_fifo_reads(lane);
            for (unsigned warmup=0; warmup<NB_MPR_X64_WARMUP; ++warmup)
                nb_mpr_x64_issue_read(command_phase, words);
            nb_mpr_x64_issue_read(command_phase, words);
            nb_mpr_x64_fifo_empty(empty);

            unsigned matched=(unsigned)nb_mpr_x64_signature(words);
            if (matched) signature_seen=1;
            printf("DDR4 x64 MPR row: gate=%u cmdphase=%u match=%u fifo_empty=%08x,%08x,%08x,%08x pops=",
                gate, command_phase, matched, empty[0], empty[1], empty[2], empty[3]);
            for (unsigned lane=0; lane<8; ++lane)
                printf("%s%u", lane ? "," : "",
                    nb_mpr_x64_fifo_reads(lane)-before[lane]);
            printf(" data=");
            for (unsigned phase=0; phase<8; ++phase)
                printf("%s%08x%08x", phase ? "," : "",
                    (unsigned)(words[phase] >> 32), (unsigned)words[phase]);
            printf("\n");
        }
    }

    for (unsigned lane=0; lane<8; ++lane)
        nb_mpr_x64_gate_write(lane, saved_delay[lane], saved_width[lane]);
    /* Best-effort MR3 restore; caller's mandatory nb_boot(0) is authoritative. */
    nb_mpr_x64_mrs(NB_MPR_X64_NORMAL_MR3);
    printf("DDR4 x64 MPR diagnostic complete: signature_seen=%u; MR3 restore issued\n",
        (unsigned)signature_seen);
    return signature_seen;
}

#endif /* CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC */
#endif /* USNATIVE_NATIVE_MPR_X64_H */
