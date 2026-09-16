#
# This file is part of LiteX.
#
# SPDX-License-Identifier: BSD-2-Clause

"""Host C tests for native profile selection and bounded firmware CSR polling."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
INCLUDE = ROOT / "litex/soc/software/liblitedram/usnative"
BIOS_INCLUDE = ROOT / "litex/soc/software/bios"
CC = shutil.which(os.environ.get("CC", "gcc"))

@unittest.skipUnless(CC, "Host C compiler required")
class TestUSNativeFirmware(unittest.TestCase):
    def compile_run(self, code, success=True):
        with tempfile.TemporaryDirectory(prefix="usnative_bios_") as temporary:
            directory = Path(temporary)
            source = directory / "test.c"
            binary = directory / ("test.exe" if os.name == "nt" else "test")
            source.write_text(code)
            result = subprocess.run([CC, "-std=c99", "-Werror=implicit-function-declaration",
                "-I", str(INCLUDE), "-I", str(BIOS_INCLUDE), str(source), "-o", str(binary)],
                capture_output=True, text=True, timeout=60)
            if not success:
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("#error", result.stderr)
                return
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def profile(self, rate=2400, extra="", check=""):
        clock, cl, cwl, rd, wr = {2400: (300000000,17,12,2,3), 2667: (333333333,19,14,0,1),
            2933: (366666666,21,16,2,3), 3200: (400000000,24,16,3,3)}[rate]
        return f'''
#define SDRAM_PHY_DDR4
#define SDRAM_PHY_DATABITS 16
#define SDRAM_PHY_DFI_DATABITS 32
#define SDRAM_PHY_PHASES 4
#define SDRAM_PHY_XDR 2
#define SDRAM_PHY_DELAYS 512
#define SDRAM_PHY_BITSLIPS 8
#define SDRAM_PHY_CMD_LATENCY 5
#define MAIN_RAM_BASE 0x40000000
#define MAIN_RAM_SIZE 0x01200000
#define CSR_DDRPHY_RIU_BUSY_ADDR 1
#define CSR_DDRPHY_RIU_VALID_ADDR 1
#define CSR_DDRPHY_TAP_STATUS_VALID_ADDR 1
#define CONFIG_CLOCK_FREQUENCY {clock}
#define SDRAM_PHY_CL {cl}
#define SDRAM_PHY_CWL {cwl}
#define SDRAM_PHY_RDPHASE {rd}
#define SDRAM_PHY_WRPHASE {wr}
{extra}
#include "profile.h"
static int selected=-1;
static void command_p0(unsigned value) {{ (void)value; selected=0; }}
static void command_p1(unsigned value) {{ (void)value; selected=1; }}
static void command_p2(unsigned value) {{ (void)value; selected=2; }}
static void command_p3(unsigned value) {{ (void)value; selected=3; }}
int main(void) {{
 USNATIVE_RD_COMMAND(1); if(selected!={rd}) return 1;
 USNATIVE_WR_COMMAND(1);
 {check}
 return selected!={wr};
}}
'''

    def test_default_profiles_without_dma_or_debug_csrs(self):
        for rate in (2400, 2667):
            with self.subTest(rate=rate):
                self.compile_run(self.profile(rate, check="USNATIVE_DEBUG(unknown_debug_function()); USNATIVE_SNAPSHOT();"))

    def test_high_rates_require_overclock_opt_in(self):
        for rate in (2933, 3200):
            with self.subTest(rate=rate):
                self.compile_run(self.profile(rate), False)
                self.compile_run(self.profile(rate, extra="#define CONFIG_SDRAM_USNATIVE_OVERCLOCK"))

    def test_reject_wrong_phase(self):
        self.compile_run(self.profile(extra="#undef SDRAM_PHY_RDPHASE\n#define SDRAM_PHY_RDPHASE 1"), False)

    def test_reject_wrong_data_width(self):
        self.compile_run(self.profile(extra="#undef SDRAM_PHY_DATABITS\n#define SDRAM_PHY_DATABITS 32"), False)

    def test_dma_opt_in_requires_engine(self):
        self.compile_run(self.profile(extra="#define CONFIG_SDRAM_USNATIVE_DMA_CALIBRATION"), False)

    def test_dma_benchmark_reports_selected_hardware_mode(self):
        self.compile_run(r'''
#include <string.h>
#include "native_dma_mode.h"
int main(void) {
 return NATIVE_DMA_BANK_GROUP_INTERLEAVED != 0 ||
     strcmp(NATIVE_DMA_MODE_NAME, "standard-native-port");
}
''')
        self.compile_run(r'''
#include <string.h>
#define CONFIG_SDRAM_NATIVE_DMA_BANK_GROUP_INTERLEAVING
#include "native_dma_mode.h"
int main(void) {
 return NATIVE_DMA_BANK_GROUP_INTERLEAVED != 1 ||
     strcmp(NATIVE_DMA_MODE_NAME, "paired-bank-group-interleaved");
}
''')

    def test_reject_insufficient_scratch_ram(self):
        self.compile_run(self.profile(extra="#undef MAIN_RAM_SIZE\n#define MAIN_RAM_SIZE 0x01000000"), False)

    def test_dma_completion_timeout_and_integrity(self):
        self.compile_run(r'''
#include <assert.h>
#define USNATIVE_DMA_POLL_LIMIT 8
static unsigned done_after, polls, busy, fault, errors, starts, readonly;
static void flush_cpu_dcache(void) {} static void flush_l2_cache(void) {}
static void cdelay(unsigned value) {(void)value;}
static unsigned dma_bench_busy_read(void) {return busy;}
static unsigned dma_bench_fault_read(void) {return fault;}
static void dma_bench_base_write(unsigned value) {assert(value==0x01000000);}
static void dma_bench_length_write(unsigned value) {assert(value==0x04000000);}
static void dma_bench_random_write(unsigned value) {(void)value;}
static void dma_bench_read_only_write(unsigned value) {readonly=value;}
static void dma_bench_timeout_write(unsigned value) {assert(value>0);}
static void dma_bench_start_write(unsigned value) {assert(value==1);++starts;}
static unsigned dma_bench_done_read(void) {return ++polls>=done_after;}
static unsigned dma_bench_read_beats_read(void) {return 0x200000;}
static unsigned dma_bench_write_beats_read(void) {return readonly?0:0x200000;}
static unsigned dma_bench_errors_read(void) {return errors;}
#include "native_dma_io.h"
int main(void) {
 done_after=3; assert(nd_dma_check(0,0)==0 && polls==3 && starts==1);
 polls=0; done_after=99; assert(nd_dma_check(0,0)==0xffffffffu && polls==8);
 polls=0; done_after=1; errors=17; assert(nd_dma_check(1,1)==17 && readonly==1);
 busy=1; unsigned old=starts; assert(nd_dma_check(0,0)==0xffffffffu && starts==old);
 busy=0; fault=3; assert(nd_dma_check(0,0)==0xffffffffu && starts==old);
 return 0;
}
''')

    def test_riu_timeout_stale_response_and_tap_freshness(self):
        self.compile_run(r'''
#include <assert.h>
static unsigned busy, error, valid=1, starts, polls, tap_valid=1;
static void cdelay(unsigned value) {(void)value;}
static unsigned ddrphy_riu_busy_read(void) {++polls;return busy;}
static unsigned ddrphy_riu_error_read(void) {return error;}
static unsigned ddrphy_riu_valid_read(void) {return valid;}
static unsigned ddrphy_riu_rdata_read(void) {return 0x1234;}
static void ddrphy_riu_nibble_write(unsigned v) {(void)v;}
static void ddrphy_riu_address_write(unsigned v) {(void)v;}
static void ddrphy_riu_wdata_write(unsigned v) {(void)v;}
static void ddrphy_riu_write_write(unsigned v) {(void)v;++starts;}
static void ddrphy_riu_read_write(unsigned v) {(void)v;++starts;}
static unsigned ddrphy_tap_status_valid_read(void) {return tap_valid;}
#include "native_status_io.h"
int main(void) {
 unsigned result=0;
 assert(native_riu_access(1,3,0,0,&result) && result==0x1234);
 busy=1; polls=0; unsigned old=starts;
 assert(!native_riu_access(1,3,0,0,&result) && polls==10000 && starts==old);
 busy=0;valid=0;result=0xbeef;
 assert(!native_riu_access(1,3,0,0,&result) && result==0xbeef);
 valid=1;error=1;assert(!native_riu_access(1,3,0,1,&result));
 assert(!native_riu_access(8,3,0,0,&result));
 assert(native_tap_wait());tap_valid=0;assert(!native_tap_wait());
 return 0;
}
''')

    def test_sdram_init_failure_ownership_and_normal_handoff(self):
        source = (ROOT / "litex/soc/software/liblitedram/sdram.c").read_text()
        start = source.index("int sdram_init(void) {")
        function = source[start:source.index("\n}\n", start) + 3]
        self.compile_run(r"""
#include <assert.h>
#include <stdio.h>
#define CONFIG_SDRAM_USNATIVE_XEM8320
#define CSR_DDRCTRL_BASE 1
#define MAIN_RAM_BASE 0x40000000ul
#define MAIN_RAM_BASE_VA MAIN_RAM_BASE
#define MEMTEST_DATA_SIZE 64
#define false 0
static unsigned calibration_ok, memory_ok=1, handoffs, tests, speeds, failed, done, status_error;
static int sdram_usnative_init(void) {return calibration_ok;}
static unsigned ddrphy_training_error_read(void) {return 7;}
static unsigned nb_fail(unsigned code) {failed=code;return code;}
static void sdram_software_control_off(void) {++handoffs;}
static void ddrctrl_init_done_write(unsigned value) {done=value;}
static void ddrctrl_init_error_write(unsigned value) {status_error=value;}
static int memtest(unsigned *p, unsigned size) {(void)p;(void)size;++tests;return memory_ok;}
static void memspeed(unsigned *p,unsigned size,int write,int random) {(void)p;(void)size;(void)write;(void)random;++speeds;}
""" + function + r"""
int main(void) {
 assert(!sdram_init() && failed==7 && !handoffs && !tests && done && status_error);
 calibration_ok=1;failed=0;memory_ok=0;
 assert(!sdram_init() && failed==20 && handoffs==1 && tests==1 && !speeds && done && status_error);
 failed=0;memory_ok=1;
 assert(sdram_init() && !failed && handoffs==2 && tests==2 && speeds==1 && done && !status_error);
 return 0;
}
""")

    def test_deskew_seeds_memory_with_debug_disabled(self):
        source = (INCLUDE / "native_bit_calibration.h").read_text()
        start = source.index("static unsigned nd_calibrate(")
        function = source[start:]
        self.compile_run(r"""
#include <assert.h>
struct nd_score {unsigned errors[16];};
static unsigned seeded, writes, stage;
#define USNATIVE_DEBUG(...) do {} while (0)
#define USNATIVE_SNAPSHOT() do {} while (0)
static unsigned nd_check(struct nd_score *score, unsigned readonly) {
 if(readonly) assert(seeded); else {seeded=1; ++writes;}
 for(unsigned i=0;i<16;++i) score->errors[i]=0;
 return 0;
}
static void ddrphy_training_stage_write(unsigned value) {stage=value;}
static void sdram_software_control_off(void) {}
static int nc_set_rx(unsigned lane, unsigned tap) {(void)lane;(void)tap;return 1;}
static unsigned nb_fail(unsigned error) {return error;}
static int nd_program(unsigned *centers,unsigned lanes,int offset) {
 (void)centers;(void)lanes;(void)offset; return 1;
}
""" + function + r"""
int main(void) {
 unsigned centers[16];
 assert(nd_calibrate(3,centers)==0);
 assert(writes==5 && stage==5);
 return 0;
}
""")

    def test_ddr_rate_does_not_overflow_signed_int(self):
        source = (ROOT / "litex/soc/software/liblitedram/sdram.c").read_text()
        start = source.index("unsigned int sdram_get_freq(void) {")
        function = source[start:source.index("\n}\n", start) + 3]
        for clock in (300000000, 333333333, 366666666, 400000000):
            self.compile_run("#define SDRAM_PHY_XDR 2\n#define SDRAM_PHY_PHASES 4\n"
                + f"#define CONFIG_CLOCK_FREQUENCY {clock}\n" + function
                + f"int main(void) {{return sdram_get_freq() != {clock * 8}u;}}")
