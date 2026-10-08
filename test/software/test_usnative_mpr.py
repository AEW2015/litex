#
# This file is part of LiteX.
#
# SPDX-License-Identifier: BSD-2-Clause

"""Host tests for the guarded direct-DFII DDR4 MPR probe."""

import unittest

from test.software import test_usnative as firmware


@unittest.skipUnless(firmware.CC, "Host C compiler required")
class TestUSNativeMPR(unittest.TestCase):
    compile_run = firmware.TestUSNativeFirmware.compile_run

    def test_x64_mpr_header_rejects_x32_geometry(self):
        self.compile_run(r'''
#define SDRAM_PHY_DDR4
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_DFI_DATABITS 64
#define SDRAM_PHY_PHASES 8
#define USNATIVE_REDUCED_X64_1TO8 0
#define CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC
#define CSR_DDRPHY_FIFO_EMPTY_ADDR 0x1000u
#include "native_mpr_x64.h"
int main(void) { return 0; }
''', success=False)

    def test_reduced_x64_mpr_probe_restores_mr3_and_gate_then_caller_reboots(self):
        # Compile and execute the actual x64 firmware header with mocked CSR
        # accessors. The read signature appears at only one of 48 candidates.
        source = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#undef assert
static void mock_check(int condition, int line) {
    if (!condition) exit((line % 200) + 1);
}
#define assert(condition) mock_check((condition), __LINE__)
#define SDRAM_PHY_DDR4
#define SDRAM_PHY_DATABITS 64
#define SDRAM_PHY_DFI_DATABITS 64
#define SDRAM_PHY_PHASES 8
#define USNATIVE_REDUCED_X64_1TO8 1
#define CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC
#define CSR_DDRPHY_FIFO_EMPTY_ADDR 0x1000u
#define DFII_COMMAND_RAS 0x01u
#define DFII_COMMAND_CAS 0x02u
#define DFII_COMMAND_WE 0x04u
#define DFII_COMMAND_CS 0x08u
#define DFII_COMMAND_RDDATA 0x10u
static unsigned mr3_values[4], mr3_count, precharges, reads;
static unsigned address, bank, gate_delay[8], gate_width[8], pops[8];
static unsigned command_phase_reads[8], address_phase, address_value, bank_value;
static unsigned fifo_csr_reads, fifo_csr_addresses[192], fifo_word_index;
static unsigned match_candidate=1;
static uint64_t read_words[8];
static void cdelay(int cycles) { assert(cycles==100); }
static void sdram_dfii_pi0_address_write(unsigned x) { address=x; }
static void sdram_dfii_pi0_baddress_write(unsigned x) { bank=x; }
static void sdram_dfii_pix_address_write(unsigned char p, unsigned x) {
    address_phase=p; address_value=x;
}
static void sdram_dfii_pix_baddress_write(unsigned char p, unsigned x) {
    assert(p==address_phase); bank_value=x;
}
static void command_p0(unsigned cmd) {
    if ((cmd&(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS)) ==
        (DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS)) {
        assert(bank==3 && mr3_count<4);
        mr3_values[mr3_count++]=address;
    } else {
        assert(cmd==(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS));
        assert(address==(1u<<10) && bank==0);
        ++precharges;
    }
}
static void command_px(unsigned char phase, unsigned cmd) {
    assert(phase<8 && cmd==(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA));
    assert(address_phase==phase && address_value==0 && bank_value==0);
    ++command_phase_reads[phase]; ++reads;
    for (unsigned lane=0; lane<8; ++lane) ++pops[lane];
    for (unsigned p=0; p<8; ++p) {
        const unsigned bit=match_candidate && gate_delay[0]==3 && phase==5 && (p&1u);
        read_words[p]=bit ? UINT64_MAX : 0;
    }
}
#define RD_PHASE(n) static uint64_t sdram_dfii_pi##n##_rddata_read(void) { return read_words[n]; }
RD_PHASE(0) RD_PHASE(1) RD_PHASE(2) RD_PHASE(3)
RD_PHASE(4) RD_PHASE(5) RD_PHASE(6) RD_PHASE(7)
#define GATE(n) \
static unsigned ddrphy_gate_delay##n##_read(void) { return gate_delay[n]; } \
static void ddrphy_gate_delay##n##_write(unsigned x) { gate_delay[n]=x; } \
static unsigned ddrphy_gate_width##n##_read(void) { return gate_width[n]; } \
static void ddrphy_gate_width##n##_write(unsigned x) { gate_width[n]=x; } \
static unsigned ddrphy_fifo_reads##n##_read(void) { return pops[n]; }
GATE(0) GATE(1) GATE(2) GATE(3) GATE(4) GATE(5) GATE(6) GATE(7)
static uint32_t csr_read_simple(unsigned address) {
    assert(fifo_csr_reads<192);
    fifo_csr_addresses[fifo_csr_reads++]=address;
    return 0xA5000000u | fifo_word_index++;
}
#include "native_mpr_x64.h"
static void reset_state(unsigned match) {
    mr3_count=precharges=reads=fifo_csr_reads=fifo_word_index=0;
    address=bank=address_phase=address_value=bank_value=0;
    match_candidate=match;
    for(unsigned i=0;i<8;i++) {
        mr3_values[i<4 ? i : 3]=0; gate_delay[i]=10+i;
        gate_width[i]=1+i%3; pops[i]=command_phase_reads[i]=0;
    }
}
static void check_restore(void) {
    assert(precharges==1 && mr3_count==2);
    assert(mr3_values[0]==NB_MPR_X64_ENABLE_MR3);
    assert(mr3_values[1]==NB_MPR_X64_NORMAL_MR3);
    assert(reads==240 && fifo_csr_reads==192);
    for(unsigned i=0;i<8;i++) {
        assert(gate_delay[i]==10+i && gate_width[i]==1+i%3);
        assert(pops[i]==240 && command_phase_reads[i]==30);
    }
    for(unsigned i=0;i<192;i++)
        assert(fifo_csr_addresses[i]==CSR_DDRPHY_FIFO_EMPTY_ADDR+4*(i%4));
}
int main(void) {
    reset_state(1);
    assert(nb_mpr_x64_probe()==1);
    check_restore();
    reset_state(0);
    assert(nb_mpr_x64_probe()==0);
    check_restore();
    return 0;
}
'''
        output = self.compile_run(source)
        self.assertEqual(output.count("DDR4 x64 MPR row:"), 96)
        self.assertIn("signature_seen=1; MR3 restore issued", output)
        self.assertIn("signature_seen=0; MR3 restore issued", output)

        # Integration contract: after the probe, a fresh nb_boot(0) must
        # complete before the RX/TX array-pattern seed scans begin.
        calibration = (firmware.INCLUDE / "native_burst_calibration.h").read_text()
        start = calibration.index("static unsigned nb_calibrate(struct nb_result *result)")
        body = calibration[start:]
        initial_boot = body.index("if (!nb_boot(0)) return nb_fail(1);")
        probe = body.index("(void)nb_mpr_x64_probe();")
        reboot = body.index("if (!nb_boot(0)) return nb_fail(1);", initial_boot + 1)
        seed_scan = body.index("unsigned rx_seed_min[USNATIVE_LANES]")
        self.assertLess(initial_boot, probe)
        self.assertLess(probe, reboot)
        self.assertLess(reboot, seed_scan)
        self.assertIn("defined(CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC) && USNATIVE_REDUCED_X64_1TO8", body)

    def test_calibration_ck_gate_candidates_cover_early_capture_without_reordering(self):
        calibration = (firmware.INCLUDE / "native_burst_calibration.h").read_text()
        start = calibration.index("    const int gate_offsets[5]")
        end = calibration.index("\n    unsigned phase_run", start)
        candidate_construction = calibration[start:end]

        # Execute the candidate-construction code verbatim from the firmware
        # header for nominal and boundary profile values. Exact expected lists
        # preserve the original preferred default/default-1/default+1 order,
        # while the structural checks guard against duplicates or CSR overflow.
        expected = {
            0: [0, 1],
            1: [1, 0, 2],
            3: [3, 2, 4, 1, 0],
            4: [4, 3, 5, 2, 1],
            5: [5, 4, 6, 3, 2],
            30: [30, 29, 31, 28, 27],
            31: [31, 30, 29, 28],
        }
        for default, gates in expected.items():
            with self.subTest(default=default):
                output = self.compile_run(f'''\
#include <assert.h>
#include <stddef.h>
#define USNATIVE_GATE_DELAY {default}
int main(void) {{
{candidate_construction}
    const unsigned expected[] = {{{", ".join(map(str, gates))}}};
    assert(gate_candidate_count == sizeof(expected)/sizeof(expected[0]));
    for (unsigned i=0; i<gate_candidate_count; ++i) {{
        assert(gate_candidates[i] == expected[i]);
        assert(gate_candidates[i] <= 31);
        for (unsigned j=0; j<i; ++j) assert(gate_candidates[i] != gate_candidates[j]);
    }}
    return 0;
}}
''')
                self.assertEqual(output, "")

    def test_mpr_location_read_selects_each_ba_location(self):
        source = (firmware.INCLUDE / "native_mpr_probe.h").read_text()

        def extract(signature):
            start = source.index(signature)
            brace = source.index("{", start)
            depth = 0
            for index in range(brace, len(source)):
                if source[index] == "{": depth += 1
                elif source[index] == "}":
                    depth -= 1
                    if depth == 0: return source[start:index + 1]
            raise AssertionError(f"unterminated function: {signature}")

        helper = extract("static void nb_mpr_lane_location_read(")
        self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#define DFII_COMMAND_CAS 4u
#define DFII_COMMAND_CS 1u
#define DFII_COMMAND_RDDATA 32u
static unsigned address, bank, command, issued, delays, reads;
static void sdram_dfii_pi0_address_write(unsigned value) { address=value; }
static void sdram_dfii_pi0_baddress_write(unsigned value) { bank=value; }
static void command_p0(int value) { command=(unsigned)value; ++issued; }
static void cdelay(int value) { assert(value==100); ++delays; }
static uint64_t sdram_dfii_pi0_rddata_read(void) { return ++reads; }
static uint64_t sdram_dfii_pi1_rddata_read(void) { return ++reads; }
static uint64_t sdram_dfii_pi2_rddata_read(void) { return ++reads; }
static uint64_t sdram_dfii_pi3_rddata_read(void) { return ++reads; }
''' + helper + r'''
int main(void) {
    for (unsigned location=0; location<3; ++location) {
        uint64_t words[4]={0};
        nb_mpr_lane_location_read(words,location);
        assert(address==0 && bank==location);
        assert(command==(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA));
        assert(issued==location+1 && delays==location+1 && reads==4*(location+1));
        for (unsigned phase=0; phase<4; ++phase)
            assert(words[phase]==4*location+phase+1);
    }
    return 0;
}
''')

    def test_mpr_2d_scan_sweeps_gate_and_rx_then_restores_or_asserts_reset(self):
        source = (firmware.INCLUDE / "native_mpr_probe.h").read_text()

        def extract(signature):
            start = source.index(signature)
            brace = source.index("{", start)
            depth = 0
            for index in range(brace, len(source)):
                if source[index] == "{": depth += 1
                elif source[index] == "}":
                    depth -= 1
                    if depth == 0: return source[start:index + 1]
            raise AssertionError(f"unterminated function: {signature}")

        functions = "\n".join(extract(signature) for signature in (
            "static int nb_mpr_lane_word_matches(",
            "static uint64_t nb_mpr_lane_samples(",
            "static unsigned nb_mpr_lane_fifo_reads(",
            "static void nb_mpr_mrs(",
            "static void nb_mpr_lane_location_read(",
            "static void nb_mpr_lane_sweep_read(",
            "static unsigned nb_mpr_valid_count(",
            "static void nb_mpr_lane_gate_width(",
            "static unsigned nb_mpr_2d_candidate(",
            "static int nb_mpr_2d_scan(",
        ))
        output = self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define SDRAM_PHY_DELAYS 512
#define USNATIVE_LANE_MASK 15u
#define NB_MPR_ENABLE_MR3 4u
#define NB_MPR_NORMAL_MR3 0u
#define DFII_COMMAND_RAS 1u
#define DFII_COMMAND_CAS 2u
#define DFII_COMMAND_WE 4u
#define DFII_COMMAND_CS 8u
#define DFII_COMMAND_RDDATA 16u
static unsigned selector=10, selected_lane, address;
static unsigned taps[4]={10,20,30,40}, gate=9, override=3;
static unsigned gates[4]={5,6,7,8}, widths[4]={1,1,1,1};
static unsigned vtc=1, fifo[4], valid_count, reads, mr3, resets, bisc_only;
static unsigned trace_arms, fail_restore, suppress_data;
static const unsigned usnative_dq_taps[32]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
    16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};
static const unsigned good_gate[4]={2,3,1,4};
static const unsigned good_rx[4]={32,64,96,128};
static uint64_t rddata[4];
static void cdelay(int n) {(void)n;}
static int native_tap_wait(void) {return 1;}
static unsigned ddrphy_dly_sel_read(void) {return selector;}
static void ddrphy_dly_sel_write(unsigned x) {selector=x; for(selected_lane=0;selected_lane<4;selected_lane++) if(x&(1u<<selected_lane)) break;}
static void ddrphy_tap_select_write(unsigned x) {(void)x;}
static unsigned ddrphy_tap_rx_count_read(void) {return taps[selected_lane];}
static int nb_delay(unsigned lane,unsigned value,int tx) {
    (void)tx;
    if (fail_restore && reads>=310 && lane==1 && value==20) return 0;
    taps[lane]=value; return 1;
}
static unsigned ddrphy_en_vtc_read(void) {return vtc;}
static void ddrphy_en_vtc_write(unsigned x) {vtc=x;}
static unsigned ddrphy_gate_delay_read(void) {return gate;}
static void ddrphy_gate_delay_write(unsigned x) {gate=x;}
static unsigned ddrphy_gate_override_read(void) {return override;}
static void ddrphy_gate_override_write(unsigned x) {override=x;}
static unsigned ddrphy_gate_delay0_read(void) {return gates[0];}
static unsigned ddrphy_gate_delay1_read(void) {return gates[1];}
static unsigned ddrphy_gate_delay2_read(void) {return gates[2];}
static unsigned ddrphy_gate_delay3_read(void) {return gates[3];}
static unsigned ddrphy_gate_width0_read(void) {return widths[0];}
static unsigned ddrphy_gate_width1_read(void) {return widths[1];}
static unsigned ddrphy_gate_width2_read(void) {return widths[2];}
static unsigned ddrphy_gate_width3_read(void) {return widths[3];}
static void ddrphy_gate_delay0_write(unsigned x) {gates[0]=x;}
static void ddrphy_gate_delay1_write(unsigned x) {gates[1]=x;}
static void ddrphy_gate_delay2_write(unsigned x) {gates[2]=x;}
static void ddrphy_gate_delay3_write(unsigned x) {gates[3]=x;}
static void ddrphy_gate_width0_write(unsigned x) {widths[0]=x;}
static void ddrphy_gate_width1_write(unsigned x) {widths[1]=x;}
static void ddrphy_gate_width2_write(unsigned x) {widths[2]=x;}
static void ddrphy_gate_width3_write(unsigned x) {widths[3]=x;}
static void ddrphy_debug_clear_write(unsigned x) {(void)x; for(unsigned i=0;i<4;i++) fifo[i]=0;}
static void ddrphy_rx_trace_arm_write(unsigned x) {
    assert(x==1 && mr3==NB_MPR_ENABLE_MR3); ++trace_arms;
    for(unsigned i=0;i<4;i++) assert(gates[i]==good_gate[i] && taps[i]==good_rx[i]);
}
static void ddrphy_bisc_only_write(unsigned x) {bisc_only=x;}
static void ddrphy_rst_write(unsigned x) {resets=x;}
static unsigned native_dfi_activity_rddata_valid_cycles_read(void) {return valid_count;}
static unsigned ddrphy_fifo_reads0_read(void) {return fifo[0];}
static unsigned ddrphy_fifo_reads1_read(void) {return fifo[1];}
static unsigned ddrphy_fifo_reads2_read(void) {return fifo[2];}
static unsigned ddrphy_fifo_reads3_read(void) {return fifo[3];}
static void sdram_dfii_pi0_address_write(unsigned x) {address=x;}
static void sdram_dfii_pi0_baddress_write(unsigned x) {(void)x;}
static void sdram_dfii_pi0_command_issue_write(unsigned x) {(void)x;}
static uint64_t sdram_dfii_pi0_rddata_read(void) {return rddata[0];}
static uint64_t sdram_dfii_pi1_rddata_read(void) {return rddata[1];}
static uint64_t sdram_dfii_pi2_rddata_read(void) {return rddata[2];}
static uint64_t sdram_dfii_pi3_rddata_read(void) {return rddata[3];}
static void command_p0(int cmd) {
    if ((cmd&(1|2|4|8))==(1|2|4|8)) {mr3=address; return;}
    if (!(cmd&DFII_COMMAND_RDDATA)) return;
    ++reads; ++valid_count;
    for(unsigned lane=0;lane<4;lane++) {
        static const unsigned boot_rx[4]={10,20,30,40};
        /* The first 128 reads establish the unmodified four-lane baseline. */
        unsigned rx_ok=(reads<=224) ? taps[lane]==boot_rx[lane] : taps[lane]==good_rx[lane];
        unsigned ok=(gates[lane]==good_gate[lane]) && rx_ok;
        if(ok) ++fifo[lane];
        if(ok && !suppress_data) for(unsigned p=0;p<4;p++) for(unsigned dq=lane*8;dq<lane*8+8;dq++)
            rddata[p] |= 1ull<<(dq+32);
    }
}
static unsigned nb_mpr_valid_count(void);
'''+functions+r'''
static void reset_state(void) {
    selector=10; selected_lane=0; address=0;
    taps[0]=10; taps[1]=20; taps[2]=30; taps[3]=40;
    gate=9; override=3; gates[0]=5; gates[1]=6; gates[2]=7; gates[3]=8;
    widths[0]=1; widths[1]=1; widths[2]=1; widths[3]=1;
    vtc=1; valid_count=reads=mr3=resets=bisc_only=trace_arms=fail_restore=suppress_data=0;
    for(unsigned i=0;i<4;i++) fifo[i]=0;
    for(unsigned p=0;p<4;p++) rddata[p]=0;
}
int main(void) {
    reset_state();
    assert(nb_mpr_2d_scan());
    assert(reads==438 && valid_count==438 && trace_arms==1);
    assert(mr3==NB_MPR_NORMAL_MR3 && resets==0 && bisc_only==0);
    assert(selector==10 && vtc==1 && gate==9 && override==3);
    assert(taps[0]==10 && taps[1]==20 && taps[2]==30 && taps[3]==40);
    assert(gates[0]==5 && gates[1]==6 && gates[2]==7 && gates[3]==8);
    assert(widths[0]==1 && widths[1]==1 && widths[2]==1 && widths[3]==1);
    reset_state(); suppress_data=1;
    assert(nb_mpr_2d_scan());
    assert(reads==438 && resets==0 && mr3==NB_MPR_NORMAL_MR3);
    reset_state(); fail_restore=1;
    assert(!nb_mpr_2d_scan());
    assert(resets==1 && bisc_only==1);
    return 0;
}
''')
        self.assertEqual(output.count("DDR4 MPR 2D gate: lane="), 96)
        self.assertEqual(output.count("DDR4 MPR 2D baseline: lane="), 12)
        # The deliberate restoration failure aborts the third scan early.
        self.assertEqual(output.count("DDR4 MPR 2D RX: lane="), 170)
        self.assertEqual(output.count("DDR4 MPR 2D repeat: index="), 20)
        self.assertIn("DDR4 MPR 2D scan complete: diagnostic-only FAIL", output)
        self.assertIn("DDR4 MPR 2D scan RESTORE FAILURE", output)
        self.assertIn("per-DQ accepted-pop trace is still required before qualification", output)

    def test_mpr_rx_tap_scan_selects_lanes_rearms_trace_and_restores_on_tap_error(self):
        source = (firmware.INCLUDE / "native_mpr_probe.h").read_text()

        def extract(signature):
            start = source.index(signature)
            brace = source.index("{", start)
            depth = 0
            for index in range(brace, len(source)):
                if source[index] == "{": depth += 1
                elif source[index] == "}":
                    depth -= 1
                    if depth == 0: return source[start:index + 1]
            raise AssertionError(f"unterminated function: {signature}")

        functions = "\n".join(extract(signature) for signature in (
            "static int nb_mpr_lane_word_matches(",
            "static unsigned nb_mpr_lane_fifo_reads(",
            "static void nb_mpr_mrs(",
            "static void nb_mpr_lane_location_read(",
            "static unsigned nb_mpr_valid_count(",
            "static void nb_mpr_lane_sweep_read(",
            "static void nb_mpr_lane_gate_width(",
            "static int nb_mpr_rx_tap_scan(",
        ))
        output = self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define SDRAM_PHY_DELAYS 512
#define USNATIVE_LANE_MASK 15u
#define NB_MPR_ENABLE_MR3 4u
#define NB_MPR_NORMAL_MR3 0u
#define DFII_COMMAND_RAS 1u
#define DFII_COMMAND_CAS 2u
#define DFII_COMMAND_WE 4u
#define DFII_COMMAND_CS 8u
#define DFII_COMMAND_RDDATA 16u
static unsigned selector=10, selected_lane, tap_select, address, taps[4]={120,120,120,120};
static unsigned gate=4, override=3, gates[4]={4,5,6,7}, widths[4]={1,1,1,1};
static unsigned vtc, fifo[4], valid_count, trace_arms, reads, mr3, fail_tap_once=1;
static const unsigned usnative_dq_taps[32]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
    16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};
static unsigned targets[4]={16,32,48,64};
static uint64_t rddata[4];
static void cdelay(int n) {(void)n;}
static int native_tap_wait(void) {return 1;}
static unsigned ddrphy_dly_sel_read(void) {return selector;}
static void ddrphy_dly_sel_write(unsigned x) {selector=x; for(selected_lane=0;selected_lane<4;selected_lane++) if(x&(1u<<selected_lane)) break;}
static void ddrphy_tap_select_write(unsigned x) {tap_select=x; (void)tap_select;}
static unsigned ddrphy_tap_rx_count_read(void) {return taps[selected_lane];}
static int nb_delay(unsigned lane,unsigned value,int tx) {
    (void)tx;
    if (fail_tap_once && lane==2 && value==32) {fail_tap_once=0; return 0;}
    taps[lane]=value; return 1;
}
static unsigned ddrphy_en_vtc_read(void) {return vtc;}
static void ddrphy_en_vtc_write(unsigned x) {vtc=x;}
static unsigned ddrphy_gate_delay_read(void) {return gate;}
static void ddrphy_gate_delay_write(unsigned x) {gate=x;}
static unsigned ddrphy_gate_override_read(void) {return override;}
static void ddrphy_gate_override_write(unsigned x) {override=x;}
static unsigned ddrphy_gate_delay0_read(void) {return gates[0];}
static unsigned ddrphy_gate_delay1_read(void) {return gates[1];}
static unsigned ddrphy_gate_delay2_read(void) {return gates[2];}
static unsigned ddrphy_gate_delay3_read(void) {return gates[3];}
static unsigned ddrphy_gate_width0_read(void) {return widths[0];}
static unsigned ddrphy_gate_width1_read(void) {return widths[1];}
static unsigned ddrphy_gate_width2_read(void) {return widths[2];}
static unsigned ddrphy_gate_width3_read(void) {return widths[3];}
static void ddrphy_gate_delay0_write(unsigned x) {gates[0]=x;}
static void ddrphy_gate_delay1_write(unsigned x) {gates[1]=x;}
static void ddrphy_gate_delay2_write(unsigned x) {gates[2]=x;}
static void ddrphy_gate_delay3_write(unsigned x) {gates[3]=x;}
static void ddrphy_gate_width0_write(unsigned x) {widths[0]=x;}
static void ddrphy_gate_width1_write(unsigned x) {widths[1]=x;}
static void ddrphy_gate_width2_write(unsigned x) {widths[2]=x;}
static void ddrphy_gate_width3_write(unsigned x) {widths[3]=x;}
static void ddrphy_debug_clear_write(unsigned x) {(void)x;}
static void ddrphy_rx_trace_arm_write(unsigned x) {
    assert(x==1); ++trace_arms;
    for(unsigned i=0;i<4;i++) assert(taps[i]==targets[i]);
    assert(mr3==4); /* trace belongs to the MPR RX scan, not the gate sweep */
}
static unsigned native_dfi_activity_rddata_valid_cycles_read(void) {return valid_count;}
static unsigned ddrphy_fifo_reads0_read(void) {return fifo[0];}
static unsigned ddrphy_fifo_reads1_read(void) {return fifo[1];}
static unsigned ddrphy_fifo_reads2_read(void) {return fifo[2];}
static unsigned ddrphy_fifo_reads3_read(void) {return fifo[3];}
static void sdram_dfii_pi0_address_write(unsigned x) {address=x;}
static void sdram_dfii_pi0_baddress_write(unsigned x) {(void)x;}
static void sdram_dfii_pi0_command_issue_write(unsigned x) {(void)x;}
static uint64_t sdram_dfii_pi0_rddata_read(void) {return rddata[0];}
static uint64_t sdram_dfii_pi1_rddata_read(void) {return rddata[1];}
static uint64_t sdram_dfii_pi2_rddata_read(void) {return rddata[2];}
static uint64_t sdram_dfii_pi3_rddata_read(void) {return rddata[3];}
static void command_p0(int cmd) {
    if ((cmd&(1|2|4|8))==(1|2|4|8)) mr3=address;
    if (cmd&DFII_COMMAND_RDDATA) {
        ++reads; ++valid_count;
        for(unsigned lane=0;lane<4;lane++) if(taps[lane]==targets[lane]) ++fifo[lane];
        for(unsigned p=0;p<4;p++) rddata[p]=0xffffffff00000000ull;
    }
}
static unsigned nb_mpr_valid_count(void);
'''+functions+r'''
int main(void) {
    assert(nb_mpr_rx_tap_scan());
    assert(reads==25 && trace_arms==1);
    for(unsigned lane=0;lane<4;lane++) assert(taps[lane]==120);
    assert(gate==4 && override==3);
    assert(gates[0]==4 && gates[1]==5 && gates[2]==6 && gates[3]==7);
    assert(widths[0]==1 && widths[1]==1 && widths[2]==1 && widths[3]==1);
    assert(selector==10 && vtc==0 && mr3==0);
    return 0;
}
''')
        self.assertEqual(output.count("DDR4 MPR RX candidate:"), 23)
        self.assertEqual(output.count("DDR4 MPR RX repeat:"), 2)
        self.assertIn("DDR4 MPR RX scan complete: diagnostic-only FAIL", output)

    def test_mpr_read_is_bounded_and_restores_normal_mr3_on_pass_or_mismatch(self):
        source = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define SDRAM_PHY_DDR4
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_DFI_DATABITS 64
#define SDRAM_PHY_PHASES 4
#define SDRAM_PHY_XDR 2
#define SDRAM_PHY_DELAYS 512
#define SDRAM_PHY_BITSLIPS 8
#define SDRAM_PHY_CMD_LATENCY 5
#define CONFIG_CLOCK_FREQUENCY 200000000
#define SDRAM_PHY_CL 11
#define SDRAM_PHY_CWL 9
#define CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC
#define CONFIG_SDRAM_USNATIVE_MPR_RX_TRACE
#define CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET 0
#define DFII_COMMAND_RAS 0x08
#define DFII_COMMAND_CAS 0x04
#define DFII_COMMAND_WE  0x02
#define DFII_COMMAND_CS  0x01
#define DFII_COMMAND_RDDATA 0x20
static unsigned address, bank, command, reads, mrs_count, mr3_values[4];
static unsigned gate, lane_gate[4], gate_seen, lane0_seen;
static unsigned trace_arms, trace_arm_gate, trace_arm_read, trace_arm_lane0;
static int corrupt;
static uint64_t rddata[4];
static void sdram_dfii_pi0_address_write(unsigned v) {address=v;}
static void sdram_dfii_pi0_baddress_write(unsigned v) {bank=v;}
static void sdram_dfii_pi0_command_issue_write(unsigned v) {(void)v;}
static uint64_t sdram_dfii_pi0_rddata_read(void) {return rddata[0];}
static uint64_t sdram_dfii_pi1_rddata_read(void) {return rddata[1];}
static uint64_t sdram_dfii_pi2_rddata_read(void) {return rddata[2];}
static uint64_t sdram_dfii_pi3_rddata_read(void) {return rddata[3];}
static unsigned ddrphy_gate_delay_read(void) {return gate;}
static void ddrphy_gate_delay_write(unsigned v) {gate=v;}
static unsigned ddrphy_gate_delay0_read(void) {return lane_gate[0];}
static unsigned ddrphy_gate_delay1_read(void) {return lane_gate[1];}
static unsigned ddrphy_gate_delay2_read(void) {return lane_gate[2];}
static unsigned ddrphy_gate_delay3_read(void) {return lane_gate[3];}
static void ddrphy_gate_delay0_write(unsigned v) {lane_gate[0]=v; if(v<=5) lane0_seen|=1u<<v;}
static void ddrphy_gate_delay1_write(unsigned v) {lane_gate[1]=v; if(v<=5) gate_seen|=1u<<v;}
static void ddrphy_gate_delay2_write(unsigned v) {lane_gate[2]=v; if(v<=5) gate_seen|=1u<<v;}
static void ddrphy_gate_delay3_write(unsigned v) {lane_gate[3]=v; if(v<=5) gate_seen|=1u<<v;}
static void ddrphy_rx_trace_arm_write(unsigned v)
{
    assert(v==1);
    ++trace_arms;
    trace_arm_gate=gate;
    trace_arm_read=reads;
    trace_arm_lane0=lane_gate[0];
}
static void command_p0(int cmd)
{
    command=(unsigned)cmd;
    if ((cmd & (DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS)) ==
        (DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS)) {
        assert(bank==3);
        assert(mrs_count < 4);
        mr3_values[mrs_count++]=address;
    } else if (cmd & DFII_COMMAND_RDDATA) {
        assert(bank==0 && address==0);
        ++reads;
        for (unsigned phase=0; phase<4; ++phase)
            rddata[phase]=(gate < 2) ? 0 : 0xffffffff00000000ull;
        if (corrupt && reads>=5) rddata[2]=0;
    }
}
static void cdelay(int cycles) {assert(cycles==100);}
#include "native_mpr_probe.h"

static void reset_state(void)
{
    address=bank=command=reads=mrs_count=gate_seen=lane0_seen=0;
    trace_arms=trace_arm_gate=trace_arm_read=trace_arm_lane0=0;
    gate=9;
    lane_gate[0]=7; lane_gate[1]=8; lane_gate[2]=9; lane_gate[3]=10;
    for (unsigned i=0; i<4; ++i) mr3_values[i]=0;
    corrupt=0;
}

int main(void)
{
    reset_state();
    assert(nb_mpr_diagnostic());
    assert(reads==63 && mrs_count==4);
    assert(trace_arms==1 && trace_arm_gate==2 && trace_arm_read==32 && trace_arm_lane0==2);
    assert(mr3_values[0]==0x0004 && mr3_values[1]==0x0000);
    assert(mr3_values[2]==0x0004 && mr3_values[3]==0x0000);
    assert(gate_seen==0x3f);
    assert(lane0_seen==0x3f);
    assert(gate==9 && lane_gate[0]==7 && lane_gate[1]==8 &&
        lane_gate[2]==9 && lane_gate[3]==10);

    reset_state(); corrupt=1;
    assert(!nb_mpr_diagnostic());
    assert(reads==63 && mrs_count==4);
    assert(trace_arms==1 && trace_arm_gate==2 && trace_arm_read==32 && trace_arm_lane0==2);
    assert(mr3_values[0]==0x0004 && mr3_values[1]==0x0000);
    assert(mr3_values[2]==0x0004 && mr3_values[3]==0x0000);
    assert(gate_seen==0x3f);
    assert(lane0_seen==0x3f);
    assert(gate==9 && lane_gate[0]==7 && lane_gate[1]==8 &&
        lane_gate[2]==9 && lane_gate[3]==10);
    return 0;
}
'''
        self.compile_run(source)

    def test_mpr_lane0_gate_offsets_sweep_legal_taps_and_restore(self):
        # The primary C harness above already validates the full MPR transaction.
        # Exercise the traced gate-2 offset at both edges. Other sweep taps
        # must remain the global tap so a capture compares a single change.
        for offset in (-2, 2):
            lane0_seen = sum({1 << (gate + (offset if gate == 2 else 0))
                for gate in range(6)})
            reads = 6 * 9
            trace_read = 2 * 9 + 5
            trace_lane0 = 2 + offset
            harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define SDRAM_PHY_DDR4
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_DFI_DATABITS 64
#define SDRAM_PHY_PHASES 4
#define SDRAM_PHY_XDR 2
#define SDRAM_PHY_DELAYS 512
#define SDRAM_PHY_BITSLIPS 8
#define SDRAM_PHY_CMD_LATENCY 5
#define CONFIG_CLOCK_FREQUENCY 200000000
#define SDRAM_PHY_CL 11
#define SDRAM_PHY_CWL 9
#define CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC
#define CONFIG_SDRAM_USNATIVE_MPR_RX_TRACE
#define CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET __OFFSET__
#define DFII_COMMAND_RAS 0x08
#define DFII_COMMAND_CAS 0x04
#define DFII_COMMAND_WE  0x02
#define DFII_COMMAND_CS  0x01
#define DFII_COMMAND_RDDATA 0x20
static unsigned address, bank, reads, gate, lanes[4], lane0_seen, trace_arms, trace_gate, trace_read, trace_lane0;
static uint64_t rddata[4];
static void sdram_dfii_pi0_address_write(unsigned v) {address=v;}
static void sdram_dfii_pi0_baddress_write(unsigned v) {bank=v;}
static void sdram_dfii_pi0_command_issue_write(unsigned v) {(void)v;}
static uint64_t sdram_dfii_pi0_rddata_read(void) {return rddata[0];}
static uint64_t sdram_dfii_pi1_rddata_read(void) {return rddata[1];}
static uint64_t sdram_dfii_pi2_rddata_read(void) {return rddata[2];}
static uint64_t sdram_dfii_pi3_rddata_read(void) {return rddata[3];}
static unsigned ddrphy_gate_delay_read(void) {return gate;}
static void ddrphy_gate_delay_write(unsigned v) {gate=v;}
static unsigned ddrphy_gate_delay0_read(void) {return lanes[0];}
static unsigned ddrphy_gate_delay1_read(void) {return lanes[1];}
static unsigned ddrphy_gate_delay2_read(void) {return lanes[2];}
static unsigned ddrphy_gate_delay3_read(void) {return lanes[3];}
static void ddrphy_gate_delay0_write(unsigned v) {lanes[0]=v; if(v<=5) lane0_seen|=1u<<v;}
static void ddrphy_gate_delay1_write(unsigned v) {lanes[1]=v;}
static void ddrphy_gate_delay2_write(unsigned v) {lanes[2]=v;}
static void ddrphy_gate_delay3_write(unsigned v) {lanes[3]=v;}
static void ddrphy_rx_trace_arm_write(unsigned v) {assert(v==1); ++trace_arms; trace_gate=gate; trace_read=reads; trace_lane0=lanes[0];}
static void command_p0(int cmd) {
    if (cmd & DFII_COMMAND_RDDATA) {
        assert(bank==0 && address==0); ++reads;
        for (unsigned i=0; i<4; ++i) rddata[i]=0xffffffff00000000ull;
    }
}
static void cdelay(int cycles) {assert(cycles==100);}
#define EXPECTED_LANE0_SEEN __LANE0_MASK__
#define EXPECTED_READS __READ_COUNT__
#define EXPECTED_TRACE_READ __TRACE_READ_COUNT__
#define EXPECTED_TRACE_LANE0 __TRACE_LANE0__
#include "native_mpr_probe.h"
int main(void) {
    gate=9; lanes[0]=7; lanes[1]=8; lanes[2]=9; lanes[3]=10;
    (void)nb_mpr_gate_sweep();
    assert(reads==EXPECTED_READS);
    assert(lane0_seen==EXPECTED_LANE0_SEEN);
    assert(trace_arms==1 && trace_gate==2 && trace_read==EXPECTED_TRACE_READ && trace_lane0==EXPECTED_TRACE_LANE0);
    assert(gate==9 && lanes[0]==7 && lanes[1]==8 && lanes[2]==9 && lanes[3]==10);
    return 0;
}
'''
            harness = harness.replace("__OFFSET__", str(offset))
            harness = harness.replace("__LANE0_MASK__", hex(lane0_seen))
            harness = harness.replace("__READ_COUNT__", str(reads))
            harness = harness.replace("__TRACE_READ_COUNT__", str(trace_read))
            harness = harness.replace("__TRACE_LANE0__", str(trace_lane0))
            self.compile_run(harness)


if __name__ == "__main__":
    unittest.main()
