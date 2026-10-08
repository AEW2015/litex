#
# This file is part of LiteX.
#
# SPDX-License-Identifier: BSD-2-Clause

"""Execute production initialization code with injected training/CSR failures."""

import unittest

from test.software import test_usnative as firmware


def function(path, signature):
    source = path.read_text()
    start = source.index(signature)
    return source[start:source.index("\n}\n", start) + 3]


@unittest.skipUnless(firmware.CC, "Host C compiler required")
class TestUSNativeRetry(unittest.TestCase):
    compile_run = firmware.TestUSNativeFirmware.compile_run

    def x64_profile(self):
        source = firmware.TestUSNativeFirmware().profile(1600)
        source = source.split("static int selected=-1;")[0]
        source = source.replace('#define SDRAM_PHY_DATABITS 16',
                                '#define SDRAM_PHY_DATABITS 64')
        source = source.replace('#define SDRAM_PHY_DFI_DATABITS 32',
                                '#define SDRAM_PHY_DFI_DATABITS 64')
        source = source.replace('#define SDRAM_PHY_PHASES 4',
                                '#define SDRAM_PHY_PHASES 8')
        source = source.replace('#include "profile.h"',
                                '#include <stdint.h>\n#include "profile.h"')
        return source

    def test_x64_1to8_read_probe_selects_all_four_native_command_phases(self):
        body = function(firmware.INCLUDE / "native_burst_calibration.h",
                        "static void nb_issue_read(")
        phase_io = "\n".join(
            f"static void sdram_dfii_pi{i}_address_write(unsigned v) {{ address[{i}]=v; }}\n"
            f"static void sdram_dfii_pi{i}_baddress_write(unsigned v) {{ bank[{i}]=v; }}\n"
            f"static void command_p{i}(unsigned v) {{ selected={i}; command=v; }}"
            for i in range(4))
        self.compile_run(self.x64_profile() + r'''
#define DFII_COMMAND_CAS 1
#define DFII_COMMAND_CS 2
#define DFII_COMMAND_RDDATA 4
static unsigned selected, command;
static unsigned address[4], bank[4];
static unsigned nb_read_phase;
''' + phase_io + "\n" + body + r'''
int main(void) {
    for (unsigned phase=0; phase<4; ++phase) {
        nb_read_phase=phase; selected=99; command=0;
        nb_issue_read(0x1234);
        if (selected!=phase || command!=(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA)) return 1;
        if (address[phase]!=0x1234 || bank[phase]!=0) return 2;
    }
    return 0;
}
''')

    def test_x64_stage3_coarse_seed_phase_gate_and_abort_paths(self):
        header = (firmware.INCLUDE / "native_burst_calibration.h").read_text()
        read_probe = function(firmware.INCLUDE / "native_burst_calibration.h",
                              "static void nb_issue_read(")
        calibrate = function(firmware.INCLUDE / "native_burst_calibration.h",
                             "static unsigned nb_calibrate(")
        phase_io = "\n".join(
            f"static void sdram_dfii_pi{i}_address_write(unsigned v) {{ (void)v; }}\n"
            f"static void sdram_dfii_pi{i}_baddress_write(unsigned v) {{ (void)v; }}\n"
            f"static void command_p{i}(unsigned v) {{ (void)v; selected_phase={i}; }}"
            for i in range(4))
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "native_rx_seed.h"
#define CONFIG_SDRAM_USNATIVE_MPR_DIAGNOSTIC
#define DFII_COMMAND_CAS 1
#define DFII_COMMAND_CS 2
#define DFII_COMMAND_RDDATA 4
#define USNATIVE_CONTROL_MASK 255
#define USNATIVE_GATE_DELAY 3
#define USNATIVE_DEBUG(...) do {} while(0)
#define USNATIVE_SNAPSHOT() do {} while(0)
static unsigned selected_phase, nb_read_phase=USNATIVE_RDPHASE;
static unsigned nb_seed_rx[USNATIVE_LANES], nb_seed_tx[USNATIVE_LANES];
static unsigned scenario, current_ck, current_gate=USNATIVE_GATE_DELAY, boots, checks;
static unsigned mpr_calls, fail_boot_at;
static unsigned current_rx[USNATIVE_LANES], current_tx[USNATIVE_LANES];
static unsigned last_direction, stage, fail_code, centers;
struct nb_window { unsigned first,last,center,short_window; };
struct nb_result { struct nb_window ck,rx[USNATIVE_LANES],dq[USNATIVE_LANES];
                   unsigned gate,phase; };
static void cdelay(unsigned v) {(void)v;}
static void nb_issue_read(unsigned address);
static int nb_boot(unsigned ck) {
    ++boots; current_ck=ck; current_gate=USNATIVE_GATE_DELAY;
    return boots!=fail_boot_at;
}
static int nb_mpr_x64_probe(void) { ++mpr_calls; return 0; }
static int nb_delay(unsigned lane,unsigned tap,int transmit) {
    if (transmit) current_tx[lane]=tap; else current_rx[lane]=tap;
    last_direction=!!transmit; return 1;
}
static void nb_set_all_gates(unsigned gate) { current_gate=gate; }
static unsigned nb_check(unsigned count,unsigned lanes) {
    ++checks;
    nb_issue_read(0);
    if (count==64) {
        unsigned lane=0; while (!(lanes&(1u<<lane))) ++lane;
        if (scenario==2 && lane==7) return 64;
        unsigned tap=last_direction?current_tx[lane]:current_rx[lane];
        unsigned low=last_direction?64:40, high=last_direction?80:56;
        return tap>=low && tap<=high ? 0 : 64;
    }
    if (count==256)
        return scenario!=1 && selected_phase==2 && current_gate==2 &&
            current_ck>=48 && current_ck<=80 ? 0 : 1;
    return 0;
}
static unsigned nb_fail(unsigned code) { fail_code=code; return code; }
static int nb_center(unsigned lane,int tx,struct nb_window *w) {
    (void)lane;(void)tx;++centers;w->first=32;w->last=64;w->center=48;w->short_window=0;return 1;
}
static int nb_center_tx(unsigned lane,struct nb_window *rx,struct nb_window *tx) {
    (void)lane;(void)rx;++centers;tx->first=32;tx->last=64;tx->center=48;tx->short_window=0;return 1;
}
static void ddrphy_training_error_write(unsigned v) {(void)v;}
static void ddrphy_training_stage_write(unsigned v) {stage=v;}
static void ddrphy_gate_override_write(unsigned v) {(void)v;}
static void ddrphy_en_vtc_write(unsigned v) {(void)v;}
static unsigned ddrphy_ready_read(void) {return 1;}
static unsigned ddrphy_vtc_rdy_read(void) {return USNATIVE_CONTROL_MASK;}
''' + phase_io + "\n" + read_probe + "\n" + calibrate + r'''
int main(void) {
    struct nb_result result;
    scenario=0; centers=boots=checks=mpr_calls=0; fail_boot_at=0; stage=fail_code=0;
    assert(nb_calibrate(&result)==0);
    assert(stage==5 && result.phase==2 && result.gate==2);
    assert(result.ck.first==48 && result.ck.last==80 && result.ck.center==64);
    assert(nb_read_phase==2 && selected_phase==2);
    assert(nb_seed_rx[7]==48 && nb_seed_tx[7]==72 && centers==16);
    /* MPR reports no signature, but every guarded array check and center
     * calculation still runs after the mandatory fresh boot. */
    assert(mpr_calls==1 && boots==71 && checks==599);

    scenario=1; centers=boots=checks=mpr_calls=0; fail_boot_at=0; stage=fail_code=0;
    assert(nb_calibrate(&result)==6);
    assert(stage==3 && fail_code==6 && centers==0);
    assert(mpr_calls==1 && boots==208 && checks==1580);

    scenario=2; centers=boots=checks=mpr_calls=0; fail_boot_at=0; stage=fail_code=0;
    assert(nb_calibrate(&result)==0);
    assert(stage==5 && nb_seed_rx[7]==USNATIVE_BOOT_RX_DELAY);
    assert(nb_seed_tx[7]==USNATIVE_BOOT_TX_DELAY && centers==16);
    assert(mpr_calls==1 && boots==71 && checks==599);

    /* The required post-MPR boot fails: no array probes/centers can run. */
    scenario=0; centers=boots=checks=mpr_calls=0; fail_boot_at=2; stage=fail_code=0;
    assert(nb_calibrate(&result)==1);
    assert(mpr_calls==1 && boots==2 && checks==0 && centers==0);
    assert(stage==3 && fail_code==1);
    return 0;
}
'''
        self.compile_run(self.x64_profile() + harness)
        self.assertIn('USNATIVE_REDUCED_X64_1TO8', header)

    def test_ck_failure_reasons_and_unchanged_acceptance(self):
        body = function(firmware.INCLUDE / "native_burst_calibration.h", "static unsigned nb_calibrate(")
        self.compile_run(r'''
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
static char messages[4096];
static int capture(const char *format, ...) {
 va_list args; va_start(args,format);
 int n=vsnprintf(messages+strlen(messages),sizeof(messages)-strlen(messages),format,args);
 va_end(args); return n;
}
#define printf capture
#define USNATIVE_DEBUG(...) do {} while(0)
#define USNATIVE_SNAPSHOT() do {} while(0)
#define USNATIVE_CONTROL_MASK 255
#define USNATIVE_GATE_DELAY 3
#define USNATIVE_RDPHASE 2
#define USNATIVE_LANES 2
#define USNATIVE_LANE_MASK 3
#define USNATIVE_BOOT_RX_DELAY 40
#define USNATIVE_BOOT_TX_DELAY 64
struct nb_window {unsigned first,last,center,short_window;};
struct nb_result {struct nb_window ck,rx[2],dq[2];unsigned gate,phase;};
static unsigned scenario,boots,checks,stage;
static unsigned nb_seed_rx[2],nb_seed_tx[2];
static void nb_set_all_gates(unsigned gate) {(void)gate;}
static int nb_boot(unsigned tap) {
 (void)tap; ++boots;
 return !((scenario==1 && boots==3) || (scenario==3 && boots==18));
}
static unsigned nb_check(unsigned count,unsigned lanes) {
 (void)count;(void)lanes;++checks;
 if(scenario==2 && checks<=17) return checks<=4?0:1;
 return scenario==4 && checks==18?7:0;
}
static unsigned nb_fail(unsigned code) {return code;}
static int nb_center(unsigned lane,int tx,struct nb_window *w) {(void)lane;(void)tx;(void)w;return 1;}
static int nb_center_tx(unsigned lane,struct nb_window *rx,struct nb_window *tx) {(void)rx;return nb_center(lane,1,tx);}
static void ddrphy_training_error_write(unsigned v) {(void)v;}
static void ddrphy_training_stage_write(unsigned v) {stage=v;}
static void ddrphy_gate_override_write(unsigned v) {(void)v;}
static void ddrphy_en_vtc_write(unsigned v) {(void)v;}
static unsigned ddrphy_ready_read(void) {return 1;}
static unsigned ddrphy_vtc_rdy_read(void) {return 255;}
static void cdelay(unsigned v) {(void)v;}
''' + body + r'''
int main(void) {
 struct nb_result result;
 const char *reason[]={"","CK initialization failed: tap=16",
   "Native stage3 CK_NO_WINDOW: scan=[0..128] step=8 phase=2 gate=3 clean_samples=4 last=24 required=5; no passing window found (center confirmation not attempted)",
   "center initialization failed: window=[0..128] center=64",
   "center burst failed: window=[0..128] center=64 errors=7"};
 for(scenario=1;scenario<=4;++scenario) {
  boots=checks=0; messages[0]=0;
  assert(nb_calibrate(&result)==(scenario==1?1:6));
  if(!strstr(messages,reason[scenario])) {
   fprintf(stderr,"scenario=%u expected=%s messages=%s\n",scenario,reason[scenario],messages);
   return 1;
  }
  assert(stage==3);
 }
 scenario=0;boots=checks=0;assert(!nb_calibrate(&result));assert(stage==5);
 assert(result.ck.first==0 && result.ck.last==128 && result.ck.center==64);
 return 0;
}
''')

    def test_full_retry_bisc_and_final_memory_admission(self):
        from test.software.test_usnative_mapping import DESCRIPTOR
        reject = function(firmware.INCLUDE / "init.h", "static void usnative_mapping_reject(")
        init = function(firmware.INCLUDE / "init.h", "static int sdram_usnative_init(")
        bisc = function(firmware.INCLUDE / "init.h", "int sdram_usnative_bisc(")
        full = function(firmware.ROOT / "litex/soc/software/liblitedram/sdram.c", "int sdram_init(void)")
        source = r'''
#include <assert.h>
#include <stdio.h>
#define CONFIG_SDRAM_USNATIVE
#define CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION
#define SDRAM_PHY_DATABITS 16
#define USNATIVE_LANES 2
#define USNATIVE_LANE_MASK 3
#define CSR_DDRCTRL_BASE 1
#define MAIN_RAM_BASE 0x40000000ul
#define MAIN_RAM_BASE_VA MAIN_RAM_BASE
#define MEMTEST_DATA_SIZE 64
#define USNATIVE_RDPHASE 2
#define SDRAM_PHY_USNATIVE_ABI_MAJOR 1
#define SDRAM_PHY_USNATIVE_ABI_MINOR 0
#define SDRAM_PHY_USNATIVE_CONFIG_ID 0x12345678u
#define SDRAM_PHY_WRPHASE 3
#define USNATIVE_DEBUG(...) do {} while(0)
#define USNATIVE_SNAPSHOT() do {} while(0)
#define USNATIVE_CONTROL_MASK 255
#define false 0
struct nb_window {unsigned first,last,center,short_window;};
struct nb_result {struct nb_window ck,rx[2],dq[2];};
static unsigned admission=1,stage,error,bisc_only,reset,owner;
static unsigned fail_cal=1,fail_memory,calibrations,memtests,bisc_calls,bisc_ok=1;
static unsigned refine_error,refine_calls,dma_width=256;
static unsigned mapping_ok=1;
static int usnative_mapping_verified;
static unsigned dfii_control=99;
static void sdram_dfii_control_write(unsigned v) {dfii_control=v;owner=1;}
static void dma_bench_software_ready_write(unsigned v) {admission=v;}
static unsigned dma_bench_data_width_read(void) {return dma_width;}
static void sdram_software_control_on(void) {assert(mapping_ok);owner=1;}
static void sdram_software_control_off(void) {assert(mapping_ok);owner=0;}
static unsigned sdram_get_freq(void) {return 3200000000u;}
static void cdelay(unsigned v) {(void)v;}
static void ddrphy_training_stage_write(unsigned v) {assert(mapping_ok);stage=v;}
static void ddrphy_training_error_write(unsigned v) {assert(mapping_ok);error=v;}
static unsigned ddrphy_training_error_read(void) {assert(mapping_ok);return error;}
static unsigned ddrphy_training_stage_read(void) {assert(mapping_ok);return stage;}
static void ddrctrl_init_done_write(unsigned v) {(void)v;}
static void ddrctrl_init_error_write(unsigned v) {(void)v;}
static unsigned ddrphy_dly_rdy_read(void) {assert(mapping_ok);return 255;}
static unsigned ddrphy_vtc_rdy_read(void) {assert(mapping_ok);return 255;}
static unsigned nb_fail(unsigned v) {assert(mapping_ok);error=v;bisc_only=reset=owner=1;return v;}
static unsigned nb_calibrate(struct nb_result *r) {
 (void)r;assert(!admission);++calibrations;
 if(fail_cal)return nb_fail(6);
 bisc_only=reset=0;stage=5;return 0;
}
static unsigned nd_calibrate(unsigned lanes,unsigned *centers,const unsigned *coarse) {
 (void)lanes;(void)centers;(void)coarse;assert(!admission);return 0;
}
static unsigned nd_dma_refine(unsigned *centers) {
 (void)centers;assert(admission);++refine_calls;
 return refine_error?nb_fail(refine_error):0;
}
static int nb_bisc(void) {++bisc_calls;bisc_only=1;reset=0;return bisc_ok;}
static int memtest(unsigned *p,unsigned size) {
 (void)p;(void)size;assert(!admission && !owner);++memtests;return !fail_memory;
}
static void memspeed(unsigned *p,unsigned size,int w,int rnd) {(void)p;(void)size;(void)w;(void)rnd;}
'''
        source += DESCRIPTOR + '\n#include "mapping.h"\n' + reject
        main = r'''
int main(void) {
 mapping_ok=0;
 for(unsigned mismatch=0;mismatch<3;++mismatch) {
  version=mismatch==0?0x20000:0x10000;
  identity=mismatch==1?0:0x12345678;
  caps=mismatch==2?0:1;
  admission=1;dfii_control=99;
  assert(!sdram_init() && !admission && dfii_control==0 && !calibrations && !memtests);
  admission=1;dfii_control=99;
  assert(!sdram_usnative_bisc() && !admission && dfii_control==0 && !bisc_calls);
 }
 mapping_ok=1;version=0x10000;identity=0x12345678;caps=1;
 assert(!sdram_init() && !admission && bisc_only && reset && calibrations==1 && !memtests);
 fail_cal=0;
 assert(sdram_init() && admission && !bisc_only && calibrations==2 && memtests==1);
 assert(sdram_usnative_bisc() && !admission && bisc_only && owner && stage==2 && bisc_calls==1);
 assert(sdram_init() && admission && calibrations==3 && memtests==2);
 fail_memory=1;assert(!sdram_init() && !admission && error==20 && reset);
 fail_memory=0;assert(sdram_init() && admission && calibrations==5 && memtests==4);
 bisc_ok=0;assert(!sdram_usnative_bisc() && !admission && reset && error==1);
 assert(sdram_init() && admission && calibrations==6);
#ifdef CONFIG_SDRAM_USNATIVE_DMA_CALIBRATION
 refine_error=17;assert(!sdram_init() && !admission && error==17);
 assert(refine_calls==6);
 unsigned prior_calibrations=calibrations;
 dma_width=128;refine_error=0;
 assert(!sdram_init() && !admission && error==21 && reset);
 assert(calibrations==prior_calibrations && refine_calls==6);
#endif
 /* A previously accepted boot must not leave permission for stale PHY writes. */
 mapping_ok=0;identity=0;admission=1;dfii_control=99;
 unsigned previous_calibrations=calibrations, previous_memtests=memtests;
 assert(!sdram_init() && !admission && dfii_control==0);
 assert(calibrations==previous_calibrations && memtests==previous_memtests);
 admission=1;dfii_control=99;
 assert(!sdram_usnative_bisc() && !admission && dfii_control==0);
 return 0;
}
'''
        for debug, dma in ((False, False), (True, False), (False, True), (True, True)):
            with self.subTest(debug=debug, dma=dma):
                config = ("#define CONFIG_SDRAM_USNATIVE_DEBUG\n" if debug else "")
                config += ("#define CONFIG_SDRAM_USNATIVE_DMA_CALIBRATION\n" if dma else "")
                self.compile_run(config + source + init + bisc + full + main)

        self.compile_run("#define SDRAM_TEST_DISABLE\n" + source + init + full + r'''
int main(void) {
 fail_cal=0;assert(sdram_init());assert(!admission && !memtests);
 return 0;
}
''')


    def test_lane_window_failures_keep_minimum_width(self):
        body = function(firmware.INCLUDE / "native_burst_calibration.h", "static int nb_center(")
        self.compile_run(r'''
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
static char messages[4096];
static int capture(const char *format, ...) {
 va_list args;va_start(args,format);
 int n=vsnprintf(messages+strlen(messages),sizeof(messages)-strlen(messages),format,args);
 va_end(args);return n;
}
#define printf capture
#define USNATIVE_DEBUG(...) do {} while(0)
#define USNATIVE_TX_SCAN_END 96
struct nb_window {unsigned first,last,center,short_window;};
static unsigned scenario,tap,programs;
static int nb_delay(unsigned lane,unsigned value,int tx) {
 assert(lane==1 && tx);tap=value;++programs;
 return !((scenario==1 && value==40) || (scenario==3 && programs==34));
}
static unsigned nb_check(unsigned count,unsigned lanes) {
 assert(lanes==2);
 if(count==256)return scenario==4?2:0;
 return tap<56 || tap>(scenario==2?84:88);
}
''' + body + r'''
int main(void) {
 const char *reasons[]={"","TX delay failed: lane=1 tap=40",
  "TX window too short: lane=1 samples=15 required=17",
  "TX center delay failed: lane=1 center=72",
  "TX center burst failed: lane=1 center=72 errors=2"};
 struct nb_window w;
 for(scenario=1;scenario<=4;++scenario) {
  programs=0;messages[0]=0;
  assert(!nb_center(1,1,&w));assert(strstr(messages,reasons[scenario]));
 }
 scenario=0;programs=0;assert(nb_center(1,1,&w));
 assert(w.first==56 && w.last==88 && w.center==72);
 return 0;
}
''')


    def test_3200_tx_retry_requires_two_wide_windows(self):
        body = function(firmware.INCLUDE / "native_burst_calibration.h", "static int nb_center_tx(")
        source = r'''
#include <assert.h>
#include <stdio.h>
#define USNATIVE_BOOT_TX_DELAY 72
struct nb_window {unsigned first,last,center,short_window;};
static unsigned scenario,scans,programs,rx_value;
static int nb_center(unsigned lane,int tx,struct nb_window *w) {
 assert(lane==0 && tx);++scans;
 w->short_window=0;
 if(scans==1) {w->short_window=scenario!=1;return 0;}
 if(scenario==2 || (scenario==6 && scans==2)) {w->short_window=1;return 0;}
 w->first=scans%2?60:56;w->last=92;w->center=76;
 if(scenario==3 && scans%2)w->first=64; /* intersection is only 28 taps */
 return 1;
}
static int nb_delay(unsigned lane,unsigned tap,int tx) {
 assert(lane==0);++programs;if(!tx)rx_value=tap;return scenario!=4;
}
static unsigned nb_check(unsigned count,unsigned lanes) {assert(count==256 && lanes==1);return 0;}
'''
        main = r'''
int main(void) {
 struct nb_window rx={24,72,48,0},tx;
#ifdef EXPECT_RETRY
 assert(nb_center_tx(0,&rx,&tx));assert(scans==3 && rx.center==52 && rx_value==52);
 assert(tx.first==60 && tx.last==92 && tx.center==76);
 for(scenario=1;scenario<=4;++scenario) {
  scans=programs=0;rx.center=48;
  assert(!nb_center_tx(0,&rx,&tx));assert(rx.center==48);
  assert(scans<=9);if(scenario==1)assert(programs==0);
 }
 scenario=6;scans=programs=0;rx.center=44;
 assert(nb_center_tx(0,&rx,&tx));assert(rx.center==52 && scans==4);
 scenario=0;scans=programs=0;rx.center=48;rx.first=44;rx.last=52;
 assert(!nb_center_tx(0,&rx,&tx));assert(scans==1 && programs==0);
#else
 assert(!nb_center_tx(0,&rx,&tx));assert(scans==1 && programs==0);
#endif
 return 0;
}
'''
        self.compile_run("#define CONFIG_CLOCK_FREQUENCY 400000000\n#define EXPECT_RETRY\n" + source + body + main)
        self.compile_run("#define CONFIG_CLOCK_FREQUENCY 333333333\n" + source + body + main)
