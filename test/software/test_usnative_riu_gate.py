#
# This file is part of LiteX.
#
# SPDX-License-Identifier: BSD-2-Clause

"""Host C tests for native DDR4 RIU gate-cohort state restoration."""

import unittest

from test.software import test_usnative as firmware


@unittest.skipUnless(firmware.CC, "Host C compiler required")
class TestUSNativeRIUGateCohort(unittest.TestCase):
    compile_run = firmware.TestUSNativeFirmware.compile_run

    def test_mpr_ui_rotation_masks_require_unique_fresh_triplets_and_lane_coherence(self):
        self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#include "native_mpr_rotation.h"

static unsigned population(uint8_t value) {
    unsigned count=0;
    for(unsigned i=0;i<8;++i) count+=(value>>i)&1u;
    return count;
}
int main(void) {
    static const uint8_t locations[12]={0,1,2,0,2,1,0,1,2,1,0,2};
    static const uint8_t register_patterns[3]={0x55u,0x33u,0x0fu};
    static const uint8_t ui_patterns[3]={0xaau,0xccu,0xf0u};
    /* Table 30 example: register [7:0]=01111111 emits UI0=0, UI1..7=1. */
    assert(usnative_mpr_serial_ui_byte(0x7fu)==0xfeu);
    for(unsigned p=0;p<3;++p)
        assert(usnative_mpr_serial_ui_byte(register_patterns[p])==ui_patterns[p]);
    uint8_t captured[12],mask=0;
    for(unsigned shift=0;shift<8;++shift) {
        uint8_t dq_masks[8];
        for(unsigned dq=0;dq<8;++dq) {
            for(unsigned read=0;read<12;++read)
                captured[read]=usnative_mpr_rotate_right8(
                    ui_patterns[locations[read]],shift);
            assert(usnative_mpr_rotation_cohort_mask(captured,locations,&mask));
            assert(mask==(1u<<shift));
            /* Applying NativeRXBitslip's right rotation by the inverse shift
             * restores every byte to the MPR UI order. */
            assert(usnative_mpr_rotate_right8(captured[1],(8u-shift)&7u)==0xccu);
            dq_masks[dq]=mask;
        }
        assert(usnative_mpr_rotation_lane_mask(dq_masks,&mask));
        assert(mask==(1u<<shift));
    }

    /* MPR0's 0x55 alone cannot identify a UI rotation. */
    mask=usnative_mpr_rotation_mask_for_pattern(0x55u,0x55u);
    assert(population(mask)>1u);

    /* A repeated/missing location invalidates the prescribed 12-read cohort. */
    for(unsigned read=0;read<12;++read)
        captured[read]=usnative_mpr_rotate_right8(ui_patterns[locations[read]],3);
    uint8_t bad_locations[12];
    for(unsigned read=0;read<12;++read) bad_locations[read]=locations[read];
    bad_locations[2]=1;
    assert(!usnative_mpr_rotation_cohort_mask(captured,bad_locations,&mask));
    assert(mask==0);

    /* An old repeating MPR0 word matches neither changing location signature. */
    for(unsigned read=0;read<12;++read) captured[read]=0x55u;
    assert(!usnative_mpr_rotation_cohort_mask(captured,locations,&mask));
    assert(mask==0);

    /* A damaged/stale sample must not be interpreted as a rotation. */
    for(unsigned read=0;read<12;++read)
        captured[read]=usnative_mpr_rotate_right8(ui_patterns[locations[read]],5);
    captured[7]^=0x01u;
    assert(!usnative_mpr_rotation_cohort_mask(captured,locations,&mask));
    assert(mask==0);

    /* All DQs in a lane must agree; a single differently rotated DQ fails. */
    uint8_t dq_masks[8];
    for(unsigned dq=0;dq<8;++dq) dq_masks[dq]=(uint8_t)(1u<<2);
    dq_masks[6]=(uint8_t)(1u<<3);
    assert(!usnative_mpr_rotation_lane_mask(dq_masks,&mask));
    assert(mask==0);
    for(unsigned dq=0;dq<8;++dq) dq_masks[dq]=(uint8_t)(1u<<6);
    assert(usnative_mpr_rotation_lane_mask(dq_masks,&mask));
    assert(mask==(1u<<6));
    dq_masks[0]=0x81u;
    assert(!usnative_mpr_rotation_lane_mask(dq_masks,&mask));
    return 0;
}
''')

    def test_optional_scheduled_return_sweep_restores_controls_and_fails_closed(self):
        self.compile_run(r'''
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#define SDRAM_PHY_DDR4
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_DFI_DATABITS 64
#define SDRAM_PHY_PHASES 4
#define SDRAM_PHY_CL 11
#define SDRAM_PHY_CWL 9
#define CONFIG_CLOCK_FREQUENCY 200000000
#define CONFIG_SDRAM_USNATIVE_MPR_RIU_GATE_COHORT
#define CONFIG_SDRAM_USNATIVE_SCHEDULED_FIFO_RETURN
#define CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC
#define CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP
#define CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
#define CONFIG_CSR_ORDERING_BIG
#define SDRAM_PHY_USNATIVE_LANE_COUNT 4
#define SDRAM_PHY_USNATIVE_DATA_CONTROLS_COUNT 8
#define SDRAM_PHY_USNATIVE_CONTROL_COUNT 32
#define CSR_DDRPHY_SCHEDULED_FIFO_MODE_ADDR 1
#define CSR_DDRPHY_SCHEDULED_FIFO_DELAY_ADDR 2
#define CSR_DDRPHY_SCHEDULED_FIFO_UNDERFLOWS_ADDR 3
#define CSR_DDRPHY_SCHEDULED_FIFO_OVERLAPS_ADDR 4
#define CSR_DDRPHY_SCHEDULED_FIFO_MISSING_LAST_ADDR 5
#define CSR_DDRPHY_FIFO_EMPTY_ADDR 0x100
#define CSR_DDRPHY_FIFO_EMPTY_SIZE 3
#define CSR_DDRPHY_FIFO_EPOCH_FLUSH_ADDR 0x120
#define CSR_DDRPHY_FIFO_EPOCH_FLUSH_COUNT_ADDR 0x124
#define CSR_DDRPHY_GATE_PHASE0_ADDR 10
#define CSR_DDRPHY_GATE_PHASE1_ADDR 11
#define CSR_DDRPHY_GATE_PHASE2_ADDR 12
#define CSR_DDRPHY_GATE_PHASE3_ADDR 13
#define CSR_DDRPHY_GATE_PHASE_UPPER0_ADDR 14
#define CSR_DDRPHY_GATE_PHASE_UPPER1_ADDR 15
#define CSR_DDRPHY_GATE_PHASE_UPPER2_ADDR 16
#define CSR_DDRPHY_GATE_PHASE_UPPER3_ADDR 17
#define USNATIVE_LANE_MASK 0x0fu
#define SDRAM_PHY_USNATIVE_TAP_COUNT 65
#define SDRAM_PHY_USNATIVE_DQ_TAPS_COUNT 32
#define SDRAM_PHY_DELAYS 512
#define USNATIVE_ARRAY_COUNT(a) (sizeof(a)/sizeof((a)[0]))
#define DFII_COMMAND_CAS 4u
#define DFII_COMMAND_CS 1u
#define DFII_COMMAND_RDDATA 32u
#define NB_MPR_RIU_CTRL0_WRITABLE 0x0d7fu
#define NB_MPR_RIU_RL_WRITABLE 0x1fffu

static const unsigned usnative_data_controls[8]={3,5,7,9,11,13,15,17};
static const unsigned usnative_dq_taps[32]={
    26,27,28,29,31,32,33,34,36,37,38,39,41,42,43,44,
    46,47,48,49,51,52,53,54,56,57,58,59,61,62,63,64};
static unsigned tap_counts[65],original_tap_counts[65],tap_select_value,dly_sel_value;
static unsigned tap_updates,fail_lane3_tap,fail_epoch_lane3_tap,fail_joint_after;
static unsigned lane3_sweep_begins,lane3_tap_summaries,lane3_tap_errors;
static unsigned lane3_point_begins,lane3_point_ends,lane3_vector_alt;
static unsigned lane3_joint_confirmed,lane3_joint_not_confirmed;
static unsigned lane3_joint_skipped,lane3_retained_all_other_phases;
static unsigned lane3_tap_candidates,lane3_tap_selected;
static unsigned incoherent_lane3,incoherent_fresh_summaries;
static unsigned ctrl0[32], rl[32], gate_delay[4], gate_width[4];
static unsigned lower_phase[4], upper_phase[4], ideal_lower[4]={0,2,3,0};
static unsigned ideal_upper[4]={0,0,0,0}, lane_fifo_count[4], lane_edge_count[4];
static unsigned gate_global=11, gate_override=5, mode_value, fifo_delay=7;
static unsigned underflows, overlaps, missing_last, vtc_value;
static unsigned csr_write_count, riu_write_count, mpr_command_count;
static unsigned probe_count, scheduled_warmups, sequence_error, fail_restore, reset_asserted, dfii_value;
static unsigned reset_reads, reset_transactions, reset_fail_at, reset_path_active;
static unsigned fifo_empty_words[3], cdelay_calls, switch_fifo_empty_after;
static unsigned inject_partial_after, flush_count_value, flush_words_remaining;
static unsigned flush_rejected, flush_stuck;
static unsigned phase_restore_fail, sweep_count, partial_after_sweep;
static unsigned measured_row_logs, epoch_context_logs;
static int quiet_printf(const char *format, ...) {
    char message[512];
    va_list args;
    va_start(args,format);
    vsnprintf(message,sizeof(message),format,args);
    va_end(args);
    if (strstr(format,"DDR4 MPR RIU gate row:")) ++measured_row_logs;
    if (strstr(format,"DDR4 MPR RIU candidate epoch:")) ++epoch_context_logs;
    if (strstr(format,"DDR4 MPR lane3 RX-tap sweep begin:")) ++lane3_sweep_begins;
    if (strstr(format,"DDR4 MPR lane3 RX-tap summary:")) ++lane3_tap_summaries;
    if (strstr(message,"DDR4 MPR lane3 RX-tap summary:") && strstr(message,"candidate=1")) ++lane3_tap_candidates;
    if (incoherent_lane3 && strstr(message,"DDR4 MPR lane3 RX-tap summary:") &&
        strstr(message,"target_fresh=12") && strstr(message,"exact_edgepop_reads=12") &&
        strstr(message,"candidate=0")) ++incoherent_fresh_summaries;
    if (strstr(message,"DDR4 MPR lane3 RX-tap selected:")) ++lane3_tap_selected;
    if (strstr(message,"DDR4 MPR lane3 RX-tap point begin:")) ++lane3_point_begins;
    if (strstr(message,"DDR4 MPR lane3 RX-tap point end:")) ++lane3_point_ends;
    if (strstr(message,"DDR4 MPR lane3 RX-tap point end:") && strstr(message,"tap_error")) ++lane3_tap_errors;
    if (strstr(message,"DDR4 MPR lane3 RX-tap sweep begin:") &&
        strstr(message,"phase_vector=1,3,2,0/")) ++lane3_vector_alt;
    if (strstr(message,"DDR4 MPR lane3 RX-tap sweep begin:") &&
        strstr(message,"other_lane_qualified_mask=0 retained_phase_mask=7"))
        ++lane3_retained_all_other_phases;
    if (strstr(message,"DDR4 MPR lane-phase joint skipped:")) ++lane3_joint_skipped;
    if (strstr(message,"DDR4 MPR lane3 RX-tap joint verdict:") && strstr(message,"status=confirmed")) ++lane3_joint_confirmed;
    if (strstr(message,"DDR4 MPR lane3 RX-tap joint verdict:") && strstr(message,"status=not_confirmed")) ++lane3_joint_not_confirmed;
    return 0;
}
#define printf quiet_printf
static int usnative_mapping_validate(void) {return 1;}
static void mock_all_dq_empty(void) {
    fifo_empty_words[0]=1u;fifo_empty_words[1]=0xffffffffu;fifo_empty_words[2]=0xffffffffu;
}
static void mock_lane3_nonempty(void) {
    mock_all_dq_empty();
    for(unsigned dq=24;dq<32;++dq) {
        const unsigned tap=usnative_dq_taps[dq];
        const unsigned word_index=2u-tap/32u;
        fifo_empty_words[word_index]&=~(1u<<(tap%32u));
    }
}
static void cdelay(int n) {
    (void)n;
    ++cdelay_calls;
    if (switch_fifo_empty_after && cdelay_calls>=switch_fifo_empty_after) {
        mock_all_dq_empty();
    }
    if (inject_partial_after && cdelay_calls>=inject_partial_after) {
        inject_partial_after=0;
        mock_lane3_nonempty();
    }
}
static unsigned ddrphy_fifo_epoch_flush_count_read(void) {return flush_count_value;}
static void ddrphy_fifo_epoch_flush_write(unsigned value) {
    assert(value==1u);
    if (flush_rejected) return;
    ++flush_count_value;
    if (flush_stuck) return;
    if (flush_words_remaining && --flush_words_remaining) return;
    mock_all_dq_empty();
}
static unsigned long csr_read_simple(unsigned long address) {
    assert(address>=CSR_DDRPHY_FIFO_EMPTY_ADDR &&
        address<CSR_DDRPHY_FIFO_EMPTY_ADDR+12u && ((address-CSR_DDRPHY_FIFO_EMPTY_ADDR)%4u)==0);
    return fifo_empty_words[(address-CSR_DDRPHY_FIFO_EMPTY_ADDR)/4u];
}
static void ddrphy_rst_write(unsigned value) {reset_asserted=value;}
static void sdram_dfii_control_write(unsigned value) {dfii_value=value;}
static unsigned ddrphy_gate_override_read(void) {return gate_override;}
static void ddrphy_gate_override_write(unsigned value) {gate_override=value;++csr_write_count;}
static unsigned ddrphy_gate_delay_read(void) {return gate_global;}
static void ddrphy_gate_delay_write(unsigned value) {gate_global=value;++csr_write_count;}
static unsigned ddrphy_gate_delay0_read(void) {return gate_delay[0];}
static unsigned ddrphy_gate_delay1_read(void) {return gate_delay[1];}
static unsigned ddrphy_gate_delay2_read(void) {return gate_delay[2];}
static unsigned ddrphy_gate_delay3_read(void) {return gate_delay[3];}
static unsigned ddrphy_gate_width0_read(void) {return gate_width[0];}
static unsigned ddrphy_gate_width1_read(void) {return gate_width[1];}
static unsigned ddrphy_gate_width2_read(void) {return gate_width[2];}
static unsigned ddrphy_gate_width3_read(void) {return gate_width[3];}
static unsigned ddrphy_gate_phase0_read(void) {return lower_phase[0];}
static unsigned ddrphy_gate_phase1_read(void) {return lower_phase[1];}
static unsigned ddrphy_gate_phase2_read(void) {return lower_phase[2];}
static unsigned ddrphy_gate_phase3_read(void) {return lower_phase[3];}
static unsigned ddrphy_gate_phase_upper0_read(void) {return upper_phase[0];}
static unsigned ddrphy_gate_phase_upper1_read(void) {return upper_phase[1];}
static unsigned ddrphy_gate_phase_upper2_read(void) {return upper_phase[2];}
static unsigned ddrphy_gate_phase_upper3_read(void) {return upper_phase[3];}
static void phase_write(unsigned lane,unsigned upper,unsigned value) {
    if (phase_restore_fail && lane_fifo_count[0]>=780u) return;
    if (upper) upper_phase[lane]=value; else lower_phase[lane]=value;
}
static void ddrphy_gate_phase0_write(unsigned v) {phase_write(0,0,v);++csr_write_count;}
static void ddrphy_gate_phase1_write(unsigned v) {phase_write(1,0,v);++csr_write_count;}
static void ddrphy_gate_phase2_write(unsigned v) {phase_write(2,0,v);++csr_write_count;}
static void ddrphy_gate_phase3_write(unsigned v) {phase_write(3,0,v);++csr_write_count;}
static void ddrphy_gate_phase_upper0_write(unsigned v) {phase_write(0,1,v);++csr_write_count;}
static void ddrphy_gate_phase_upper1_write(unsigned v) {phase_write(1,1,v);++csr_write_count;}
static void ddrphy_gate_phase_upper2_write(unsigned v) {phase_write(2,1,v);++csr_write_count;}
static void ddrphy_gate_phase_upper3_write(unsigned v) {phase_write(3,1,v);++csr_write_count;}
static void ddrphy_gate_delay0_write(unsigned v) {gate_delay[0]=v;++csr_write_count;}
static void ddrphy_gate_delay1_write(unsigned v) {gate_delay[1]=v;++csr_write_count;}
static void ddrphy_gate_delay2_write(unsigned v) {gate_delay[2]=v;++csr_write_count;}
static void ddrphy_gate_delay3_write(unsigned v) {gate_delay[3]=v;++csr_write_count;}
static void ddrphy_gate_width0_write(unsigned v) {gate_width[0]=v;++csr_write_count;}
static void ddrphy_gate_width1_write(unsigned v) {gate_width[1]=v;++csr_write_count;}
static void ddrphy_gate_width2_write(unsigned v) {gate_width[2]=v;++csr_write_count;}
static void ddrphy_gate_width3_write(unsigned v) {gate_width[3]=v;++csr_write_count;}
static unsigned ddrphy_scheduled_fifo_mode_read(void) {return mode_value;}
static void ddrphy_scheduled_fifo_mode_write(unsigned value) {
    if (fail_restore && value==1 && probe_count==39 && fifo_delay==7) return;
    mode_value=value;++csr_write_count;
}
static unsigned ddrphy_scheduled_fifo_delay_read(void) {return fifo_delay;}
static void ddrphy_scheduled_fifo_delay_write(unsigned value) {fifo_delay=value;++csr_write_count;}
static unsigned ddrphy_scheduled_fifo_underflows_read(void) {return underflows;}
static unsigned ddrphy_scheduled_fifo_overlaps_read(void) {return overlaps;}
static unsigned ddrphy_scheduled_fifo_missing_last_read(void) {return missing_last;}
static unsigned ddrphy_fifo_reads0_read(void) {return lane_fifo_count[0];}
static unsigned ddrphy_fifo_reads1_read(void) {return lane_fifo_count[1];}
static unsigned ddrphy_fifo_reads2_read(void) {return lane_fifo_count[2];}
static unsigned ddrphy_fifo_reads3_read(void) {return lane_fifo_count[3];}
static unsigned ddrphy_dqs_wrclk_edges0_read(void) {return lane_edge_count[0];}
static unsigned ddrphy_dqs_wrclk_edges1_read(void) {return lane_edge_count[1];}
static unsigned ddrphy_dqs_wrclk_edges2_read(void) {return lane_edge_count[2];}
static unsigned ddrphy_dqs_wrclk_edges3_read(void) {return lane_edge_count[3];}
static unsigned nb_mpr_dqs_wrclk_edges(unsigned lane) {return lane_edge_count[lane];}
static int native_tap_wait(void) {return 1;}
static unsigned ddrphy_tap_select_read(void) {return tap_select_value;}
static void ddrphy_tap_select_write(unsigned value) {tap_select_value=value;}
static unsigned ddrphy_tap_allowed_read(void) {return tap_select_value<65u;}
static unsigned ddrphy_tap_rx_count_read(void) {return tap_counts[tap_select_value];}
static void ddrphy_tap_rx_rst_write(unsigned value) {
    assert(value==1u && tap_select_value<65u);tap_counts[tap_select_value]=0;++tap_updates;
}
static void ddrphy_tap_rx_inc_write(unsigned value) {
    assert(value==1u && tap_select_value<65u);++tap_counts[tap_select_value];++tap_updates;
}
static unsigned ddrphy_dly_sel_read(void) {return dly_sel_value;}
static void ddrphy_dly_sel_write(unsigned value) {dly_sel_value=value;}
static int nb_delay(unsigned lane,unsigned tap,int transmit) {
    assert(lane<4u && tap<=96u && (tap&7u)==0 && !transmit);
    if (lane==3u && fail_lane3_tap) {fail_lane3_tap=0;return 0;}
    dly_sel_value=1u<<lane;
    for(unsigned bit=lane*8u;bit<lane*8u+8u;++bit)
        tap_counts[usnative_dq_taps[bit]]=tap;
    if (lane==3u && fail_epoch_lane3_tap==tap) {
        fail_epoch_lane3_tap=~0u;
        mock_lane3_nonempty();
        flush_rejected=1u;
    }
    return 1;
}
static unsigned ddrphy_en_vtc_read(void) {return vtc_value;}
static void ddrphy_en_vtc_write(unsigned value) {assert(value==0u);++csr_write_count;}
static void nb_mpr_lane_sweep_read(uint64_t words[4]) {
    ++sweep_count;
    if (partial_after_sweep && sweep_count>=partial_after_sweep) {
        partial_after_sweep=0;
        mock_lane3_nonempty();
    }
    if (mode_value) ++scheduled_warmups;
    for (unsigned i=0;i<4;++i) words[i]=0;
}
static void nb_mpr_lane_location_read(uint64_t words[4],unsigned location) {
    ++mpr_command_count;
    static const unsigned pattern[3]={0xaau,0xccu,0xf0u};
    for(unsigned i=0;i<4;++i) words[i]=0;
    for(unsigned lane=0;lane<4;++lane) {
        ++lane_fifo_count[lane];++lane_edge_count[lane];
        if (lower_phase[lane]!=ideal_lower[lane] ||
            upper_phase[lane]!=ideal_upper[lane]) continue;
        if (lane==3u && (tap_counts[usnative_dq_taps[24]]<32u ||
                         tap_counts[usnative_dq_taps[24]]>48u)) continue;
        if (lane==2u && fail_joint_after && mpr_command_count>=fail_joint_after) continue;
        for(unsigned dq=8u*lane;dq<8u*lane+8u;++dq) {
            for(unsigned ui=0;ui<8u;++ui) {
                unsigned serial_pattern=pattern[location];
                if (lane==3u && incoherent_lane3 &&
                    tap_counts[usnative_dq_taps[24]]>=32u &&
                    tap_counts[usnative_dq_taps[24]]<=48u &&
                    (mpr_command_count&1u))
                    serial_pattern=((serial_pattern<<1)|(serial_pattern>>7))&0xffu;
                if ((serial_pattern>>ui)&1u) {
                    if (ui&1u) words[ui/2u]|=1ull<<(dq+32u);
                    else words[ui/2u]|=1ull<<dq;
                }
            }
        }
    }
}
static int nb_mpr_lane_word_matches(const uint64_t words[4],unsigned lane) {
    (void)words;(void)lane;return 0;
}
static int nb_mpr_pattern_lane_matches(const uint64_t words[4],unsigned lane,
                                      unsigned pattern) {
    for(unsigned dq=8u*lane;dq<8u*lane+8u;++dq) {
        unsigned captured=0,matched=0;
        for(unsigned phase=0;phase<4u;++phase) {
            captured|=((unsigned)(words[phase]>>dq)&1u)<<(2u*phase);
            captured|=((unsigned)(words[phase]>>(dq+32u))&1u)<<(2u*phase+1u);
        }
        for(unsigned rotation=0;rotation<8u;++rotation) {
            unsigned rotated=((pattern<<rotation)|(pattern>>(8u-rotation)))&0xffu;
            if(captured==rotated) matched=1;
        }
        if(!matched) return 0;
    }
    return 1;
}
static int nb_mpr_pattern_probe_prepared(unsigned delay,unsigned width,unsigned coarse,
    int (*prepare)(void *,unsigned,unsigned),void *context,unsigned *completed_reads) {
    static const unsigned locations[12]={0,1,2,0,2,1,0,1,2,1,0,2};
    (void)delay;(void)width;(void)coarse;
    for(unsigned read=0;read<12;++read) {
        reset_path_active=1;
        const int prepared=prepare(context,read,locations[read]);
        reset_path_active=0;
        if(!prepared) {*completed_reads=read;return 0;}
        ++reset_reads;
        *completed_reads=read+1;
    }
    return 1;
}
static void nb_mpr_pattern_probe(unsigned delay,unsigned width,unsigned coarse) {
    if (!mode_value) return;
    const unsigned expected_delay=4u+probe_count/3u;
    const unsigned expected_coarse=(unsigned[]){3u,4u,5u}[probe_count%3u];
    if (delay!=1u || width!=1u || coarse!=expected_coarse ||
        fifo_delay!=expected_delay) sequence_error=1;
    ++probe_count; ++underflows; ++overlaps; missing_last=4;
}
static int native_riu_access(unsigned control,unsigned reg,unsigned data,
                             unsigned write,unsigned *result) {
    if(write) ++riu_write_count;
    if (reset_path_active) {
        ++reset_transactions;
        if (reset_fail_at && reset_transactions==reset_fail_at) return 0;
    }
    if (write) {
        if (reg==0x00) {
            ctrl0[control]=(ctrl0[control]&~NB_MPR_RIU_CTRL0_WRITABLE) |
                (data&NB_MPR_RIU_CTRL0_WRITABLE); *result=ctrl0[control];
        } else if (reg==0x30) {
            rl[control]=data&NB_MPR_RIU_RL_WRITABLE; *result=rl[control];
        } else return 0;
    } else *result=reg==0x00 ? ctrl0[control] : reg==0x30 ? rl[control] : 0xffffffffu;
    return 1;
}
#include "native_mpr_riu_gate_cohort.h"
static void init(void) {
    vtc_value=0;csr_write_count=riu_write_count=mpr_command_count=0;
    mode_value=0;fifo_delay=7;probe_count=scheduled_warmups=sequence_error=underflows=overlaps=0;
    reset_reads=reset_transactions=reset_fail_at=reset_path_active=0;
    fifo_empty_words[0]=1u;fifo_empty_words[1]=0xffffffffu;fifo_empty_words[2]=0xffffffffu;
    cdelay_calls=switch_fifo_empty_after=0;
    inject_partial_after=flush_count_value=flush_words_remaining=flush_rejected=flush_stuck=0;
    sweep_count=partial_after_sweep=measured_row_logs=epoch_context_logs=0;
    phase_restore_fail=0;
    tap_select_value=7u;dly_sel_value=5u;tap_updates=0;
    fail_lane3_tap=0;fail_epoch_lane3_tap=~0u;fail_joint_after=0;
    lane3_sweep_begins=lane3_tap_summaries=lane3_tap_errors=0;
    lane3_joint_confirmed=lane3_joint_not_confirmed=0;
    lane3_joint_skipped=lane3_retained_all_other_phases=0;
    lane3_tap_candidates=lane3_tap_selected=0;
    lane3_point_begins=lane3_point_ends=lane3_vector_alt=0;
    incoherent_lane3=incoherent_fresh_summaries=0;
    for(unsigned i=0;i<65;++i) {
        tap_counts[i]=100u+i;
        original_tap_counts[i]=tap_counts[i];
    }
    missing_last=0;fail_restore=reset_asserted=0;dfii_value=0x55;
    gate_global=11;gate_override=5;
    gate_delay[0]=6;gate_delay[1]=7;gate_delay[2]=9;gate_delay[3]=10;
    gate_width[0]=2;gate_width[1]=3;gate_width[2]=4;gate_width[3]=5;
    lower_phase[0]=3;lower_phase[1]=2;lower_phase[2]=1;lower_phase[3]=0;
    upper_phase[0]=2;upper_phase[1]=1;upper_phase[2]=0;upper_phase[3]=3;
    memset(lane_fifo_count,0,sizeof(lane_fifo_count));
    memset(lane_edge_count,0,sizeof(lane_edge_count));
    for(unsigned i=0;i<8;++i) {unsigned c=usnative_data_controls[i];ctrl0[c]=0x41u+i;rl[c]=0x102u+13u*i;}
}
static void check_restored(unsigned expected_mode) {
    assert(mode_value==expected_mode && fifo_delay==7);
    assert(gate_global==11 && gate_override==5);
    assert(gate_delay[0]==6 && gate_delay[1]==7 && gate_delay[2]==9 && gate_delay[3]==10);
    assert(gate_width[0]==2 && gate_width[1]==3 && gate_width[2]==4 && gate_width[3]==5);
    assert(lower_phase[0]==3 && lower_phase[1]==2 && lower_phase[2]==1 && lower_phase[3]==0);
    assert(upper_phase[0]==2 && upper_phase[1]==1 && upper_phase[2]==0 && upper_phase[3]==3);
    assert(tap_select_value==7u && dly_sel_value==5u);
    for(unsigned bit=24;bit<32;++bit)
        assert(tap_counts[usnative_dq_taps[bit]]==original_tap_counts[usnative_dq_taps[bit]]);
    for(unsigned i=0;i<8;++i) {unsigned c=usnative_data_controls[i];assert(rl[c]==0x102u+13u*i);assert((ctrl0[c]&NB_MPR_RIU_CTRL0_WRITABLE)==(0x41u+i));}
}
int main(void) {
    unsigned epoch_mask=0,epoch_stable=0;
    init();assert(nb_mpr_riu_prepare_dq_epoch(&epoch_mask,&epoch_stable));
    assert(epoch_mask==0xffffffffu && epoch_stable==3u);
    /* Taps are logical CSR bit numbers; big-order CSR words put tap 64 in
     * word 0 bit 0 and taps 26..31 in word 2. Check both ends explicitly. */
    assert(nb_mpr_riu_dq_empty_mask()==0xffffffffu);
    fifo_empty_words[0]=0u;
    assert((nb_mpr_riu_dq_empty_mask() & (1u<<31))==0u);
    assert(nb_mpr_riu_dq_empty_mask()==0x7fffffffu);
    init();inject_partial_after=1u;
    assert(nb_mpr_riu_prepare_dq_epoch(&epoch_mask,&epoch_stable));
    assert(epoch_mask==0xffffffffu && epoch_stable==3u && flush_count_value==1u);
    for(unsigned depth_index=0;depth_index<2;++depth_index) {
        const unsigned depth=depth_index ? 64u : 24u;
        init();mock_lane3_nonempty();flush_words_remaining=depth;
        assert(nb_mpr_riu_prepare_dq_epoch(&epoch_mask,&epoch_stable));
        assert(epoch_mask==0xffffffffu && epoch_stable==3u && flush_count_value==depth);
    }
    init();mock_lane3_nonempty();flush_rejected=1u;
    assert(!nb_mpr_riu_prepare_dq_epoch(&epoch_mask,&epoch_stable));
    assert(epoch_mask!=0xffffffffu && epoch_stable==0u && flush_count_value==0u);
    init();mock_lane3_nonempty();flush_stuck=1u;
    assert(!nb_mpr_riu_prepare_dq_epoch(&epoch_mask,&epoch_stable));
    assert(epoch_mask!=0xffffffffu && epoch_stable==0u && flush_count_value==100u);
    init();
    ideal_lower[0]=1u;ideal_lower[1]=3u;ideal_lower[2]=2u;
    ideal_upper[0]=2u;ideal_upper[1]=1u;ideal_upper[2]=3u;
    assert(nb_mpr_riu_lane_phase_cohort());check_restored(0);
    assert(lane3_sweep_begins==1u && lane3_vector_alt==1u);
    assert(lane3_point_begins==13u && lane3_point_ends==13u);
    assert(lane3_tap_summaries==13u && lane3_tap_candidates>0u);
    assert(lane3_tap_selected==1u);
    /* With no passing phase candidate in any other lane, the RXDQ fallback
     * still runs at all 13 points using saved phases, but cannot qualify a
     * four-lane joint result. */
    init();
    ideal_lower[0]=ideal_lower[1]=ideal_lower[2]=4u;
    ideal_lower[3]=0u;
    memset(ideal_upper,0,sizeof(ideal_upper));
    assert(nb_mpr_riu_lane_phase_cohort());check_restored(0);
    assert(lane3_sweep_begins==1u && lane3_point_begins==13u && lane3_point_ends==13u);
    assert(lane3_tap_summaries==13u && lane3_tap_selected==1u);
    assert(lane3_retained_all_other_phases==1u && lane3_joint_skipped==1u);
    assert(lane3_joint_confirmed==0u && lane3_joint_not_confirmed==0u);
    ideal_lower[0]=0u;ideal_lower[1]=2u;ideal_lower[2]=3u;
    ideal_upper[0]=ideal_upper[1]=ideal_upper[2]=0u;
    init();incoherent_lane3=1u;
    assert(nb_mpr_riu_lane_phase_cohort());check_restored(0);
    assert(lane3_tap_summaries==13u && incoherent_fresh_summaries==3u);
    assert(lane3_tap_candidates==0u && lane3_tap_selected==0u);
    init();fail_lane3_tap=1u;
    assert(nb_mpr_riu_lane_phase_cohort());check_restored(0);
    assert(lane3_sweep_begins==1u && lane3_tap_errors==1u);
    assert(lane3_tap_summaries==0u && lane3_tap_selected==0u);
    init();fail_epoch_lane3_tap=0u;
    assert(nb_mpr_riu_lane_phase_cohort());check_restored(0);
    assert(lane3_sweep_begins==1u && lane3_point_begins==1u && lane3_point_ends==1u);
    assert(lane3_tap_summaries==0u && lane3_tap_selected==0u);
    init();fail_joint_after=925u; /* 64 phase rows + 13 tap rows, then joint read 1. */
    assert(nb_mpr_riu_lane_phase_cohort());check_restored(0);
    assert(lane3_tap_selected==1u && lane3_joint_not_confirmed==1u);
    assert(lane3_joint_confirmed==0u);
    init();fifo_empty_words[0]=0u;flush_rejected=1u;
    assert(nb_mpr_riu_lane_phase_cohort());check_restored(0);
    for(unsigned lane=0;lane<4;++lane)
        assert(lane_fifo_count[lane]==0u && lane_edge_count[lane]==0u);
    init();vtc_value=1;
    assert(!nb_mpr_riu_lane_phase_cohort());
    assert(csr_write_count==0 && riu_write_count==0 && mpr_command_count==0);
    assert(gate_global==11 && gate_override==5 && mode_value==0 && fifo_delay==7);
    init();mode_value=1;
    assert(!nb_mpr_riu_lane_phase_cohort());
    assert(csr_write_count==0 && riu_write_count==0 && mpr_command_count==0);
    assert(gate_global==11 && gate_override==5 && mode_value==1 && fifo_delay==7);
    init();assert(nb_mpr_riu_gate_cohort());check_restored(0);
    assert(probe_count==39 && !sequence_error && underflows==39 && overlaps==39);
    assert(scheduled_warmups==39u*4u);
    assert(reset_reads==36 && reset_transactions==36u*8u*5u);
    assert(missing_last==4 && !reset_asserted && dfii_value==0x55);
    for(unsigned lane=0;lane<4;++lane)
        assert(lane_fifo_count[lane]==936u && lane_edge_count[lane]==936u);
    init();mode_value=1;assert(nb_mpr_riu_gate_cohort());check_restored(1);
    assert(probe_count==39 && !sequence_error && scheduled_warmups==39u*4u && !reset_asserted);
    assert(reset_reads==36 && reset_transactions==36u*8u*5u);
    init();reset_fail_at=5;
    assert(nb_mpr_riu_gate_cohort());check_restored(0);
    assert(reset_reads==0 && reset_transactions==5 && !reset_asserted);
    init();mode_value=1;fail_restore=1;assert(!nb_mpr_riu_gate_cohort());
    assert(reset_asserted==1 && dfii_value==0 && mode_value==0);
    init();fifo_empty_words[0]=0u;flush_rejected=1u;
    assert(nb_mpr_riu_gate_cohort());check_restored(0);
    assert(probe_count==0 && mode_value==0 && fifo_delay==7);
    assert(sweep_count==0 && measured_row_logs==0 && epoch_context_logs==1);
    init();partial_after_sweep=4u;flush_rejected=1u;
    assert(nb_mpr_riu_gate_cohort());check_restored(0);
    assert(sweep_count==4u && measured_row_logs==0 && probe_count==0);
    assert(epoch_context_logs==2u);
    init();phase_restore_fail=1;
    assert(!nb_mpr_riu_gate_cohort());
    assert(reset_asserted==1 && dfii_value==0 && mode_value==0);
    return 0;
}
''')

    def test_cohort_restores_or_quiesces_on_every_exit_path(self):
        output = self.compile_run(r'''
#include <assert.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#define SDRAM_PHY_DDR4
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_DFI_DATABITS 64
#define SDRAM_PHY_PHASES 4
#define SDRAM_PHY_CL 11
#define SDRAM_PHY_CWL 9
#define CONFIG_CLOCK_FREQUENCY 200000000
#define CONFIG_SDRAM_USNATIVE_MPR_RIU_GATE_COHORT
#define CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC
#define CONFIG_SDRAM_USNATIVE_MPR_LANE_SWEEP
#define CONFIG_SDRAM_USNATIVE_DQS_WRCLK_MONITOR
#define SDRAM_PHY_USNATIVE_LANE_COUNT 4
#define SDRAM_PHY_USNATIVE_DATA_CONTROLS_COUNT 8
#define SDRAM_PHY_USNATIVE_CONTROL_COUNT 32
#define USNATIVE_ARRAY_COUNT(a) (sizeof(a)/sizeof((a)[0]))
#define DFII_COMMAND_CAS 4u
#define DFII_COMMAND_CS 1u
#define DFII_COMMAND_RDDATA 32u
#define NB_MPR_RIU_CTRL0_WRITABLE 0x0d7fu
#define NB_MPR_RIU_RL_WRITABLE 0x1fffu

static const unsigned usnative_data_controls[8]={3,5,7,9,11,13,15,17};
static unsigned mapping_valid=1;
static unsigned ctrl0[32], original_ctrl0[32], rl[32];
static unsigned gate_global, gate_override, gate_delay[4], gate_width[4];
static unsigned gate_global_writes, gate_override_writes, gate_lane_writes;
static unsigned riu_reads, riu_writes, memory_writes, mpr_reads, edge_counts[4];
static unsigned reset_asserted, dfii_control, restore_phase;
static unsigned scenario, failed_once, failed_save;
static unsigned log_abort, log_restore_failure, log_invalid_mapping;

static int test_printf(const char *format, ...) {
    va_list args;
    va_start(args,format);
    if (strstr(format,"cohort %s; restored")) {
        const char *status=va_arg(args,const char *);
        if (strcmp(status,"aborted with experiment error")==0) ++log_abort;
    }
    if (strstr(format,"RESTORE FAILURE")) ++log_restore_failure;
    if (strstr(format,"invalid generated control mapping")) ++log_invalid_mapping;
    va_end(args);
    return 0;
}
#define printf test_printf

static int usnative_mapping_validate(void) { return mapping_valid; }
static void cdelay(int n) {(void)n;}
static void ddrphy_rst_write(unsigned value) {reset_asserted=value;}
static void sdram_dfii_control_write(unsigned value) {dfii_control=value;}
static unsigned ddrphy_gate_override_read(void) {return gate_override;}
static void ddrphy_gate_override_write(unsigned value) {gate_override=value;++gate_override_writes;}
static unsigned ddrphy_gate_delay_read(void) {return gate_global;}
static void ddrphy_gate_delay_write(unsigned value) {gate_global=value;++gate_global_writes;}
static unsigned ddrphy_gate_delay0_read(void) {return gate_delay[0];}
static unsigned ddrphy_gate_delay1_read(void) {return gate_delay[1];}
static unsigned ddrphy_gate_delay2_read(void) {return gate_delay[2];}
static unsigned ddrphy_gate_delay3_read(void) {return gate_delay[3];}
static unsigned ddrphy_gate_width0_read(void) {return gate_width[0];}
static unsigned ddrphy_gate_width1_read(void) {return gate_width[1];}
static unsigned ddrphy_gate_width2_read(void) {return gate_width[2];}
static unsigned ddrphy_gate_width3_read(void) {return gate_width[3];}
static void set_gate(unsigned lane,unsigned value) {
    if (scenario==3 && restore_phase && lane==2 && value==9) return;
    gate_delay[lane]=value;++gate_lane_writes;
}
static void ddrphy_gate_delay0_write(unsigned value) {set_gate(0,value);}
static void ddrphy_gate_delay1_write(unsigned value) {set_gate(1,value);}
static void ddrphy_gate_delay2_write(unsigned value) {set_gate(2,value);}
static void ddrphy_gate_delay3_write(unsigned value) {set_gate(3,value);}
static void ddrphy_gate_width0_write(unsigned value) {gate_width[0]=value;++gate_lane_writes;}
static void ddrphy_gate_width1_write(unsigned value) {gate_width[1]=value;++gate_lane_writes;}
static void ddrphy_gate_width2_write(unsigned value) {gate_width[2]=value;++gate_lane_writes;}
static void ddrphy_gate_width3_write(unsigned value) {gate_width[3]=value;++gate_lane_writes;}
static unsigned ddrphy_fifo_reads0_read(void) {return 0;}
static unsigned ddrphy_fifo_reads1_read(void) {return 0;}
static unsigned ddrphy_fifo_reads2_read(void) {return 0;}
static unsigned ddrphy_fifo_reads3_read(void) {return 0;}
static unsigned nb_mpr_dqs_wrclk_edges(unsigned lane) {return edge_counts[lane];}
static void nb_mpr_lane_sweep_read(uint64_t words[4]) {
    ++mpr_reads;
    for (unsigned phase=0;phase<4;++phase) words[phase]=0;
}
static void nb_mpr_pattern_probe(unsigned delay,unsigned width,unsigned coarse) {
    if (scenario>=2 && delay==4 && width==2 && coarse==15) restore_phase=1;
}
static int nb_mpr_pattern_probe_prepared(unsigned delay,unsigned width,unsigned coarse,
    int (*prepare)(void *,unsigned,unsigned),void *context,unsigned *completed_reads) {
    static const unsigned locations[12]={0,1,2,0,2,1,0,1,2,1,0,2};
    (void)delay;(void)width;(void)coarse;
    restore_phase=0;
    for(unsigned read=0;read<12;++read) {
        if(!prepare(context,read,locations[read])) {
            *completed_reads=read;restore_phase=1;return 0;
        }
        ++mpr_reads;
        *completed_reads=read+1;
    }
    restore_phase=1;
    return 1;
}
static int nb_mpr_lane_word_matches(const uint64_t words[4],unsigned lane) {
    (void)words;(void)lane;return 1;
}
static int native_riu_access(unsigned control,unsigned reg,unsigned data,
                             unsigned write,unsigned *result) {
    ++riu_reads;
    if (write) {
        ++riu_writes;
        if (scenario==1 && !failed_once && reg==0x30 && restore_phase==0) {
            failed_once=1;return 0;
        }
        if (restore_phase && scenario==2 && reg==0x30 && data==0x123u) {
            return 0;
        }
        if (restore_phase && scenario==4 && reg==0x00 &&
            data==original_ctrl0[control]) return 0;
        if (reg==0x00) {
            ctrl0[control]=(ctrl0[control]&~NB_MPR_RIU_CTRL0_WRITABLE) |
                (data&NB_MPR_RIU_CTRL0_WRITABLE);
            *result=ctrl0[control];
        } else if (reg==0x30) {
            rl[control]=data&NB_MPR_RIU_RL_WRITABLE;
            *result=rl[control];
        } else return 0;
    } else {
        if (failed_save) {failed_save=0;return 0;}
        *result=reg==0x00 ? ctrl0[control] : reg==0x30 ? rl[control] : 0xffffffffu;
    }
    return 1;
}

#include "native_mpr_riu_gate_cohort.h"

static void reset_state(void) {
    mapping_valid=1;gate_global=11;gate_override=5;
    gate_delay[0]=6;gate_delay[1]=7;gate_delay[2]=9;gate_delay[3]=10;
    gate_width[0]=2;gate_width[1]=3;gate_width[2]=4;gate_width[3]=5;
    for(unsigned i=0;i<8;++i) {
        unsigned c=usnative_data_controls[i];
        ctrl0[c]=0x1200u|((0x41u+17u*i)&NB_MPR_RIU_CTRL0_WRITABLE);
        original_ctrl0[c]=ctrl0[c]&NB_MPR_RIU_CTRL0_WRITABLE;
        rl[c]=0x102u+13u*i;
    }
    gate_global_writes=gate_override_writes=gate_lane_writes=0;
    riu_reads=riu_writes=memory_writes=mpr_reads=restore_phase=0;
    reset_asserted=0;dfii_control=0x55;scenario=failed_once=failed_save=0;
    log_abort=log_restore_failure=log_invalid_mapping=0;
    for(unsigned lane=0;lane<4;++lane) edge_counts[lane]=0;
}
static void assert_original_state(void) {
    assert(gate_global==11 && gate_override==5);
    assert(gate_delay[0]==6 && gate_delay[1]==7 && gate_delay[2]==9 && gate_delay[3]==10);
    assert(gate_width[0]==2 && gate_width[1]==3 && gate_width[2]==4 && gate_width[3]==5);
    for(unsigned i=0;i<8;++i) {
        unsigned c=usnative_data_controls[i];
        assert(rl[c]==(0x102u+13u*i));
        assert((ctrl0[c]&NB_MPR_RIU_CTRL0_WRITABLE)==
            ((0x41u+17u*i)&NB_MPR_RIU_CTRL0_WRITABLE));
    }
}
int main(void) {
    reset_state();
    assert(nb_mpr_riu_gate_cohort());
    assert_original_state();
    assert(!reset_asserted && dfii_control==0x55 && !memory_writes);
    assert(mpr_reads==3248 && log_abort==0 && log_restore_failure==0);

    reset_state();failed_save=1;
    assert(nb_mpr_riu_gate_cohort());
    assert(gate_global_writes==0 && gate_override_writes==0 && gate_lane_writes==0);
    assert(riu_writes==0 && !reset_asserted && mpr_reads==0);

    reset_state();scenario=1;
    /* Fail the first experiment write; production cleanup must restore state. */
    assert(nb_mpr_riu_gate_cohort());
    assert_original_state();
    assert(failed_once && log_abort==1 && !log_restore_failure);
    assert(!reset_asserted && dfii_control==0x55 && mpr_reads==0);

    for(unsigned failure=2;failure<=4;++failure) {
        reset_state();scenario=failure;
        for(unsigned i=0;i<8;++i) rl[usnative_data_controls[i]]=0x123u;
        assert(!nb_mpr_riu_gate_cohort());
        assert(reset_asserted==1 && dfii_control==0);
        assert(log_restore_failure==1);
    }

    reset_state();mapping_valid=0;
    assert(nb_mpr_riu_gate_cohort());
    assert(log_invalid_mapping==1 && riu_reads==0 && riu_writes==0);
    assert(gate_global_writes==0 && gate_lane_writes==0 && !reset_asserted);
    return 0;
}
''')
        self.assertEqual(output, "")

