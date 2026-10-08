/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Per-DQ read deskew using the generated logical tap mapping.
 * Experimental, destructive calibration; see doc/usnative_bios.md.
 */

#include "native_data_map.h"

struct nd_score {
    unsigned errors[SDRAM_PHY_DATABITS];
    unsigned word_errors_by_phase[8];
    unsigned first_address[4],first_actual[4],first_expected[4],first_count;
};
static int nd_error(unsigned address,unsigned actual,unsigned expected,void *arg)
{
    struct nd_score *score=arg; unsigned diff=actual^expected;
#if SDRAM_PHY_DATABITS == 64
    /* Each 64-bit phase spans two adjacent 32-bit CPU words. */
    ++score->word_errors_by_phase[(address>>3)&7];
#else
    ++score->word_errors_by_phase[(address>>2)&7];
#endif
    if(score->first_count<4) {
        unsigned index=score->first_count++;
        score->first_address[index]=address;
        score->first_actual[index]=actual;
        score->first_expected[index]=expected;
    }
    for(unsigned bit=0;bit<SDRAM_PHY_DATABITS;++bit) {
        unsigned mask=usnative_cpu_word_dq_mask(bit,address>>2);
        if(diff & mask) ++score->errors[bit];
    }
    return 0;
}
static unsigned nd_check(struct nd_score *score,unsigned readonly)
{
    struct memtest_config config;
    for(unsigned bit=0;bit<SDRAM_PHY_DATABITS;++bit) score->errors[bit]=0;
    for(unsigned phase=0;phase<8;++phase) score->word_errors_by_phase[phase]=0;
    score->first_count=0;
    config.show_progress=0;config.read_only=readonly;config.on_error=nd_error;config.arg=score;
    memtest_data((unsigned*)0x41000000,0x200000,1,&config);
    unsigned errors=0;for(unsigned bit=0;bit<SDRAM_PHY_DATABITS;++bit) errors+=score->errors[bit];
    return errors;
}
static int nd_program(const unsigned *centers,unsigned lane_mask,int offset)
{
    flush_cpu_dcache();flush_l2_cache();cdelay(1000);
    sdram_dfii_control_write(DFII_CONTROL_CKE|DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
    ddrphy_en_vtc_write(0);cdelay(1000);
    for(unsigned bit=0;bit<SDRAM_PHY_DATABITS;++bit) {
        if(!(lane_mask & (1u<<(bit/8)))) continue;
        int tap=(int)centers[bit]+offset;
        if(tap<0 || tap>=(int)SDRAM_PHY_DELAYS) {
            printf("Native per-bit tap request out of range: bit=%u requested=%d tap_count=%u\n",
                bit,tap,(unsigned)SDRAM_PHY_DELAYS);
            return 0;
        }
        ddrphy_tap_select_write(usnative_dq_taps[bit]);cdelay(100);
        if(!ddrphy_tap_allowed_read()) return 0;
        ddrphy_tap_rx_rst_write(1);cdelay(100);
        for(int i=0;i<tap;++i) {ddrphy_tap_rx_inc_write(1);cdelay(100);}
        if(!native_tap_wait()) return 0;
        if(ddrphy_tap_rx_count_read()!=(unsigned)tap) return 0;
    }
    sdram_software_control_off();cdelay(10000);
    return nc_wait_vtc();
}

/* Check that controller writes replace retained DDR contents before a
 * destructive per-bit sweep. One BL8 per pass isolates startup write
 * acceptance from the later continuous 2 MiB pattern. */
static unsigned nd_write_probe(void)
{
#if SDRAM_PHY_DATABITS == 32 && SDRAM_PHY_PHASES == 8
    volatile unsigned *memory = (volatile unsigned *)0x40000000;
#else
    volatile unsigned *memory = (volatile unsigned *)0x41000000;
#endif
    unsigned errors = 0;
    const unsigned words_per_burst = USNATIVE_BL8_CPU_WORDS;

    for (unsigned pass = 0; pass < 2; ++pass) {
#ifdef CONFIG_SDRAM_USNATIVE_WRITE_MONITOR
        native_dfi_activity_clear_write(1);
        ddrphy_probe_fast_clear_write(1);
        cdelay(100);
#endif
        for (unsigned word = 0; word < words_per_burst; ++word)
            memory[word] = 0x12345678u ^ (pass * 0x9e3779b9u) ^ (word * 0x01020408u);
        flush_cpu_dcache_range((void *)memory, words_per_burst * sizeof(*memory));
        flush_l2_cache();
        for (unsigned word = 0; word < words_per_burst; ++word) {
            unsigned expected = 0x12345678u ^ (pass * 0x9e3779b9u) ^ (word * 0x01020408u);
            unsigned actual = memory[word];

            if (actual != expected) {
                ++errors;
                if (word < 4)
                    printf("Native CPU write probe: pass=%u word=%u actual=%08x expected=%08x\n",
                        pass, word, actual, expected);
            }
        }
#ifdef CONFIG_SDRAM_USNATIVE_WRITE_MONITOR
        printf("Native DFI activity: pass=%u control=%u write_cmd=%u write_data=%u read_cmd=%u "
            "mask=%08x word0=%08x word7=%08x\n",
            pass, (unsigned)sdram_dfii_control_read(),
            (unsigned)native_dfi_activity_write_commands_read(),
            (unsigned)native_dfi_activity_write_data_read(),
            (unsigned)native_dfi_activity_read_commands_read(),
            (unsigned)native_dfi_activity_write_mask_read(),
            (unsigned)native_dfi_activity_write_word0_read(),
            (unsigned)native_dfi_activity_write_word7_read());
        printf("Native fast write: pass=%u wren=%u oe=%u dqs_drive=%u not_ready=%u "
            "first_oe_word0=%08x debug=%08x faults=%x\n",
            pass, (unsigned)ddrphy_probe_fast_wren_read(),
            (unsigned)ddrphy_probe_fast_oe_read(),
            (unsigned)ddrphy_probe_dqs_drive_read(),
            (unsigned)ddrphy_probe_not_ready_read(),
            (unsigned)ddrphy_probe_first_oe_word0_read(),
            (unsigned)ddrphy_debug_read(), (unsigned)ddrphy_faults_read());
        printf("Native fast trace: pass=%u command=%x address=%x "
            "words=%08x,%08x,%08x,%08x,%08x,%08x\n",
            pass, (unsigned)ddrphy_probe_fast_command_read(),
            (unsigned)ddrphy_probe_fast_address_read(),
            (unsigned)ddrphy_probe_word0_read(),
            (unsigned)ddrphy_probe_word1_read(),
            (unsigned)ddrphy_probe_word2_read(),
            (unsigned)ddrphy_probe_word3_read(),
            (unsigned)ddrphy_probe_word4_read(),
            (unsigned)ddrphy_probe_word5_read());
#endif
    }
    printf("Native CPU write probe: errors=%u\n", errors);
#if SDRAM_PHY_DATABITS == 32 && SDRAM_PHY_PHASES == 8
    /* Compare the CPU result with a direct DFII read of the first BL8.
     * This separates a missing physical write from a stale controller read.
     * Preserve the fail-closed result of the CPU probe either way. */
    sdram_software_control_on();
    cdelay(1000);
    sdram_dfii_pi0_address_write(1 << 10);
    command_p0(DFII_COMMAND_RAS | DFII_COMMAND_WE | DFII_COMMAND_CS);
    cdelay(1000);
    sdram_dfii_pi0_address_write(0);
    sdram_dfii_pi0_baddress_write(0);
    command_p0(DFII_COMMAND_RAS | DFII_COMMAND_CS);
    cdelay(1000);
    nb_issue_read(0);
    cdelay(1000);
    unsigned direct0 = sdram_dfii_pi0_rddata_read();
#if USNATIVE_DUAL_SLOT_FULL_WIDTH
    unsigned direct7 = (unsigned)(sdram_dfii_pi3_rddata_read() >> 32);
#else
    unsigned direct7 = sdram_dfii_pi7_rddata_read();
#endif
    unsigned expected0 = 0x12345678u ^ 0x9e3779b9u;
    unsigned expected7 = expected0 ^ (7u * 0x01020408u);
    printf("Native direct BL8 crosscheck: word0=%08x expected0=%08x "
        "word7=%08x expected7=%08x\n",
        direct0, expected0, direct7, expected7);
    sdram_dfii_pi0_address_write(1 << 10);
    command_p0(DFII_COMMAND_RAS | DFII_COMMAND_WE | DFII_COMMAND_CS);
    cdelay(1000);
    sdram_software_control_off();
    cdelay(1000);
    if (!nc_wait_vtc()) {
        printf("Native direct BL8 crosscheck: VTC did not become ready\n");
        ++errors;
    }
#endif
    return errors;
}

static unsigned nd_calibrate(unsigned lane_mask,unsigned *centers,
    const unsigned *coarse_centers)
{
    struct nd_score score;
    unsigned run[SDRAM_PHY_DATABITS],best[SDRAM_PHY_DATABITS],end[SDRAM_PHY_DATABITS];
    unsigned min_errors[SDRAM_PHY_DATABITS],min_tap[SDRAM_PHY_DATABITS];
    for(unsigned bit=0;bit<SDRAM_PHY_DATABITS;++bit) {
        run[bit]=0;best[bit]=0;end[bit]=0;
        min_errors[bit]=~0u;min_tap[bit]=0;
    }
    ddrphy_training_stage_write(7);sdram_software_control_off();
    if (nd_write_probe()) return nb_fail(16);
    /* Seeding is required even when diagnostic arguments are compiled out. */
    unsigned seed_errors=nd_check(&score,0);
    unsigned seed_phase_errors[8],seed_addresses[4],seed_actual[4],seed_expected[4];
    unsigned seed_first_count=score.first_count;
    for(unsigned phase=0;phase<8;++phase)
        seed_phase_errors[phase]=score.word_errors_by_phase[phase];
    for(unsigned index=0;index<seed_first_count;++index) {
        seed_addresses[index]=score.first_address[index];
        seed_actual[index]=score.first_actual[index];
        seed_expected[index]=score.first_expected[index];
    }
    USNATIVE_DEBUG("Bit RX deskew seed errors: %u\n",seed_errors);
    (void)seed_errors;
    for(unsigned lane=0;lane<USNATIVE_LANES;++lane) {
        if(!(lane_mask & (1u<<lane))) continue;
        /* Scan the same 56-tap span around the lane center found by burst
         * training. A fixed [12..68] range can clip a valid per-bit eye when
         * that center moves between fresh FPGA configurations. */
        unsigned coarse=coarse_centers[lane];
        unsigned first=coarse>28?coarse-28:0;
        if(first>454) first=454;
        first&=~1u;
        unsigned last=first+56;
        for(unsigned tap=first;tap<=last;tap+=2) {
            if(!nc_set_rx(lane,tap)) return nb_fail(12);
            nd_check(&score,1);
            USNATIVE_DEBUG("DESKEW_SCAN lane=%u tap=%u",lane,tap);
            for(unsigned bit=lane*8;bit<lane*8+8;++bit) {
                unsigned errors=score.errors[bit];USNATIVE_DEBUG(" %u",errors);
                if(errors<min_errors[bit]) {min_errors[bit]=errors;min_tap[bit]=tap;}
                if(errors) run[bit]=0;else ++run[bit];
                if(run[bit]>best[bit]) {best[bit]=run[bit];end[bit]=tap;}
            }
            USNATIVE_DEBUG("\n");
        }
        for(unsigned bit=lane*8;bit<lane*8+8;++bit) {
            if(best[bit]<9) {
                printf("Native per-bit RX window too short: lane=%u bit=%u "
                    "samples=%u required=9 scan=[%u..%u] coarse=%u seed_errors=%u\n",
                    lane,bit,best[bit],first,last,coarse,seed_errors);
#if SDRAM_PHY_DATABITS == 64
                printf("  Seed word errors by 64-bit phase:");
#else
                printf("  Seed word errors by 32-bit phase:");
#endif
                for(unsigned phase=0;phase<8;++phase)
                    printf(" %u:%u",phase,seed_phase_errors[phase]);
                printf("\n");
                for(unsigned index=0;index<seed_first_count;++index)
                    printf("  Seed mismatch @%08x actual=%08x expected=%08x\n",
                        seed_addresses[index],seed_actual[index],seed_expected[index]);
                for(unsigned candidate=lane*8;candidate<lane*8+8;++candidate)
                    printf("  DQ%u best=%u min_errors=%u at=%u\n",candidate,
                        best[candidate],min_errors[candidate],min_tap[candidate]);
                return nb_fail(13); /* >=16 measured taps per bit */
            }
            unsigned window_first=end[bit]-2*(best[bit]-1);
            centers[bit]=window_first+2*((best[bit]-1)/2);
            USNATIVE_DEBUG("DESKEW_BIT %u WINDOW %u %u CENTER %u\n",bit,window_first,end[bit],centers[bit]);
        }
        if(!nd_program(centers,1u<<lane,0)) return nb_fail(12);
    }
    for(int offset=-4;offset<=4;offset+=4) {
        if(!nd_program(centers,lane_mask,offset)) return nb_fail(12);
        unsigned errors=nd_check(&score,0);
        USNATIVE_DEBUG("DESKEW_GUARD offset=%d errors=%u\n",offset,errors);
        if(errors) return nb_fail(15);
    }
    if(!nd_program(centers,lane_mask,0)) return nb_fail(12);
    unsigned errors=nd_check(&score,0);
    USNATIVE_DEBUG("DESKEW_FINAL_ERRORS %u\n",errors);
    if(errors) return nb_fail(14);
    ddrphy_training_stage_write(5);USNATIVE_SNAPSHOT();return 0;
}
