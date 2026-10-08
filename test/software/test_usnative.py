#
# This file is part of LiteX.
#
# SPDX-License-Identifier: BSD-2-Clause

"""Host C tests for native profile selection and bounded firmware CSR polling."""

import os
import shutil
import tempfile
import unittest
import subprocess

from pathlib import Path

# Firmware harness --------------------------------------------------------------------------------

ROOT         = Path(__file__).resolve().parents[2]
INCLUDE      = ROOT / "litex/soc/software/liblitedram/usnative"
BIOS_INCLUDE = ROOT / "litex/soc/software/bios"
CC           = shutil.which(os.environ.get("CC", "gcc"))


@unittest.skipUnless(CC, "Host C compiler required")
class TestUSNativeFirmware(unittest.TestCase):
    def compile_run(self, code, success=True, extra_flags=()):
        with tempfile.TemporaryDirectory(prefix="usnative_bios_") as temporary:
            directory = Path(temporary)
            source = directory / "test.c"
            binary = directory / ("test.exe" if os.name == "nt" else "test")
            source.write_text(code)
            result = subprocess.run(
                [CC, "-std=c99", "-Werror=implicit-function-declaration", *extra_flags,
                 "-I", str(INCLUDE), "-I", str(BIOS_INCLUDE), str(source), "-o", str(binary)],
                capture_output=True, text=True, timeout=60,
            )
            if not success:
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("#error", result.stderr)
                return
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            return result.stdout

    def profile(self, rate=2400, extra="", check=""):
        clock, cl, cwl, rd, wr = {
            1600: (100000000, 11, 9, 0, 2),
            2400: (300000000, 17, 12, 2, 3),
            2667: (333333333, 19, 14, 0, 1),
            2933: (366666666, 21, 16, 2, 3),
            3200: (400000000, 24, 16, 3, 3),
        }[rate]
        operating_rd = 2 if rate == 3200 else rd
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
 USNATIVE_RD_COMMAND(1); if(selected!={operating_rd}) return 1;
 if(USNATIVE_RDPHASE!={operating_rd}) return 2;
 if(USNATIVE_BOOT_TX_DELAY!={72 if rate == 3200 else 88}) return 3;
 if(USNATIVE_BOOT_RX_DELAY!={48 if rate == 3200 else 32}) return 4;
 if(USNATIVE_TX_SCAN_END!=176) return 5;
 USNATIVE_WR_COMMAND(1);
 {check}
 return selected!={wr};
}}
'''

    def profile_2667_dual_slot_1to8(self, clock, *, dual_slot=True):
        source = self.profile(2667).replace(
            "#define SDRAM_PHY_DATABITS 16", "#define SDRAM_PHY_DATABITS 32")
        source = source.replace("#define SDRAM_PHY_DFI_DATABITS 32",
                                "#define SDRAM_PHY_DFI_DATABITS 64")
        source = source.replace("#define SDRAM_PHY_PHASES 4", "#define SDRAM_PHY_PHASES 8")
        source = source.replace("#define CONFIG_CLOCK_FREQUENCY 333333333",
                                f"#define CONFIG_CLOCK_FREQUENCY {clock}")
        config = "#include <stdint.h>\n"
        if dual_slot:
            config = "#define CONFIG_SDRAM_USNATIVE_DUAL_SLOT\n" + config
        source = source.replace('#include "profile.h"', config + '#include "profile.h"')
        source = source.replace("if(USNATIVE_TX_SCAN_END!=176)",
                                "if(USNATIVE_TX_SCAN_END!=240)")
        return source

    def profile_high_rate_dual_slot_1to8(self, rate, clock, *, dual_slot=True):
        source = self.profile(rate, extra="#define CONFIG_SDRAM_USNATIVE_OVERCLOCK")
        source = source.replace("#define SDRAM_PHY_DATABITS 16", "#define SDRAM_PHY_DATABITS 32")
        source = source.replace("#define SDRAM_PHY_DFI_DATABITS 32", "#define SDRAM_PHY_DFI_DATABITS 64")
        source = source.replace("#define SDRAM_PHY_PHASES 4", "#define SDRAM_PHY_PHASES 8")
        source = source.replace("#define CONFIG_CLOCK_FREQUENCY " + str({2933: 366666666, 3200: 400000000}[rate]),
                                f"#define CONFIG_CLOCK_FREQUENCY {clock}")
        source = source.replace('#include "profile.h"',
            ("#define CONFIG_SDRAM_USNATIVE_DUAL_SLOT\n" if dual_slot else "") +
            '#include "profile.h"')
        source = source.replace("if(USNATIVE_TX_SCAN_END!=176)",
                                "if(USNATIVE_TX_SCAN_END!=240)")
        source = source.replace("USNATIVE_WR_COMMAND(1);",
            "USNATIVE_WR_COMMAND(1);\n"
            " if(sizeof(usnative_dfi_word_t)!=8 || !USNATIVE_DUAL_SLOT_FULL_WIDTH) return 6;\n"
            " if(USNATIVE_LANES!=4 || USNATIVE_LANE_MASK!=15) return 7;\n"
            " if(USNATIVE_RDPHASE!=2) return 8;\n"
            f" if(USNATIVE_GATE_DELAY!={4 if rate == 2933 else 5}) return 9;\n")
        return source

    def profile_low_rate_dual_slot_1to8(self, clock, cl, cwl, rd, wr):
        source = self.profile(2400)
        source = source.replace("#define SDRAM_PHY_DATABITS 16", "#define SDRAM_PHY_DATABITS 32")
        source = source.replace("#define SDRAM_PHY_DFI_DATABITS 32", "#define SDRAM_PHY_DFI_DATABITS 64")
        source = source.replace("#define SDRAM_PHY_PHASES 4", "#define SDRAM_PHY_PHASES 8")
        source = source.replace("#define SDRAM_PHY_CL 17", f"#define SDRAM_PHY_CL {cl}")
        source = source.replace("#define SDRAM_PHY_CWL 12", f"#define SDRAM_PHY_CWL {cwl}")
        source = source.replace("#define SDRAM_PHY_RDPHASE 2", f"#define SDRAM_PHY_RDPHASE {rd}")
        source = source.replace("#define SDRAM_PHY_WRPHASE 3", f"#define SDRAM_PHY_WRPHASE {wr}")
        source = source.replace("#define CONFIG_CLOCK_FREQUENCY 300000000",
                                f"#define CONFIG_CLOCK_FREQUENCY {clock}")
        source = source.replace('#include "profile.h"',
            "#define CONFIG_SDRAM_USNATIVE_DUAL_SLOT\n" + '#include "profile.h"')
        source = source.replace("if(USNATIVE_TX_SCAN_END!=176)",
                                "if(USNATIVE_TX_SCAN_END!=240)")
        source = source.replace("if(selected!=2) return 1;", f"if(selected!={rd}) return 1;")
        source = source.replace(f"if(USNATIVE_RDPHASE!=2) return 2;",
                                f"if(USNATIVE_RDPHASE!={rd}) return 2;")
        source = source.replace("return selected!=3;", f"return selected!={wr};")
        source = source.replace("USNATIVE_WR_COMMAND(1);",
            "USNATIVE_WR_COMMAND(1);\n"
            " if(sizeof(usnative_dfi_word_t)!=8 || !USNATIVE_DUAL_SLOT_FULL_WIDTH) return 6;\n"
            " if(USNATIVE_LANES!=4 || USNATIVE_LANE_MASK!=15) return 7;\n"
            f" if(USNATIVE_RDPHASE!={rd} || USNATIVE_GATE_DELAY!=3) return 8;\n")
        return source

    def profile_reduced_x64_1to8(self, clock, cl, cwl, rd, wr):
        source = self.profile(2400)
        source = source.replace("#define SDRAM_PHY_DATABITS 16", "#define SDRAM_PHY_DATABITS 64")
        source = source.replace("#define SDRAM_PHY_DFI_DATABITS 32", "#define SDRAM_PHY_DFI_DATABITS 64")
        source = source.replace("#define SDRAM_PHY_PHASES 4", "#define SDRAM_PHY_PHASES 8")
        source = source.replace("#define SDRAM_PHY_CL 17", f"#define SDRAM_PHY_CL {cl}")
        source = source.replace("#define SDRAM_PHY_CWL 12", f"#define SDRAM_PHY_CWL {cwl}")
        source = source.replace("#define SDRAM_PHY_RDPHASE 2", f"#define SDRAM_PHY_RDPHASE {rd}")
        source = source.replace("#define SDRAM_PHY_WRPHASE 3", f"#define SDRAM_PHY_WRPHASE {wr}")
        source = source.replace("#define CONFIG_CLOCK_FREQUENCY 300000000", f"#define CONFIG_CLOCK_FREQUENCY {clock}")
        source = source.replace('#include "profile.h"', '#include <stdint.h>\n#include "profile.h"')
        main_start = source.index("int main(void) {")
        return source[:main_start] + f'''int main(void) {{
 USNATIVE_RD_COMMAND(1);
 if(selected!={rd} || USNATIVE_RDPHASE!={rd}) return 1;
 if(sizeof(usnative_dfi_word_t)!=8 || !USNATIVE_REDUCED_X64_1TO8 || USNATIVE_DUAL_SLOT_FULL_WIDTH) return 2;
 if(USNATIVE_LANES!=8 || USNATIVE_LANE_MASK!=255) return 3;
 USNATIVE_WR_COMMAND(1);
 return selected!={wr};
}}
'''

    def test_reduced_x64_1to8_profiles_accept_only_expected_geometry(self):
        for profile in ((100000000, 11, 9, 0, 2), (150000000, 17, 12, 2, 3)):
            with self.subTest(profile=profile):
                self.compile_run(self.profile_reduced_x64_1to8(*profile))
        source = self.profile_reduced_x64_1to8(100000000, 11, 9, 0, 2)
        self.compile_run(source.replace("#define SDRAM_PHY_DFI_DATABITS 64",
            "#define SDRAM_PHY_DFI_DATABITS 128"), False)
        self.compile_run(source.replace("#define SDRAM_PHY_PHASES 8",
            "#define SDRAM_PHY_PHASES 4"), False)
        self.compile_run(source.replace("#define SDRAM_PHY_DFI_DATABITS 64",
            "#define SDRAM_PHY_DFI_DATABITS 128"), False)
        self.compile_run(source.replace('#include <stdint.h>\n#include "profile.h"',
            "#define CONFIG_SDRAM_USNATIVE_DUAL_SLOT\n#include <stdint.h>\n#include \"profile.h\""), False)

    def test_cpu_word_dq_mapping_covers_both_x64_dwords_without_undefined_shifts(self):
        self.compile_run(r'''
#define SDRAM_PHY_DATABITS 64
#include <assert.h>
#include "native_data_map.h"
int main(void) {
    assert(USNATIVE_BL8_CPU_WORDS==16u);
    assert(usnative_cpu_word_dq_mask(0, 0)==1u);
    assert(usnative_cpu_word_dq_mask(31, 0)==0x80000000u);
    assert(usnative_cpu_word_dq_mask(32, 0)==0u);
    assert(usnative_cpu_word_dq_mask(63, 0)==0u);
    assert(usnative_cpu_word_dq_mask(0, 1)==0u);
    assert(usnative_cpu_word_dq_mask(31, 1)==0u);
    assert(usnative_cpu_word_dq_mask(32, 1)==1u);
    assert(usnative_cpu_word_dq_mask(63, 1)==0x80000000u);
    assert(usnative_cpu_word_dq_mask(64, 1)==0u);
    return 0;
}
''')

    def test_x64_bit_calibration_error_callback_tracks_dq_half_and_address(self):
        source = (INCLUDE / "native_bit_calibration.h").read_text()
        struct_start = source.index("struct nd_score {")
        struct = source[struct_start:source.index("\n};", struct_start) + 3]
        function_start = source.index("static int nd_error(")
        function = source[function_start:source.index("\n}", function_start) + 3]
        data_map_source = (INCLUDE / "native_data_map.h").read_text()
        data_map_start = data_map_source.index("static inline uint32_t usnative_cpu_word_dq_mask(")
        data_map = data_map_source[data_map_start:data_map_source.index("\n}", data_map_start) + 3]
        self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#define SDRAM_PHY_DATABITS 64
''' + data_map + struct + function + r'''
int main(void) {
    struct nd_score score;
    memset(&score, 0, sizeof(score));
    nd_error(0x41000000u, 1u, 0u, &score);
    assert(score.errors[0]==1 && score.errors[32]==0);
    memset(&score, 0, sizeof(score));
    nd_error(0x41000004u, 1u, 0u, &score);
    assert(score.errors[0]==0 && score.errors[32]==1);
    memset(&score, 0, sizeof(score));
    nd_error(0x41000000u, 0x80000000u, 0u, &score);
    assert(score.errors[31]==1 && score.errors[63]==0);
    memset(&score, 0, sizeof(score));
    nd_error(0x41000004u, 0x80000000u, 0u, &score);
    assert(score.errors[31]==0 && score.errors[63]==1);
    memset(&score, 0, sizeof(score));
    nd_error(0x41000000u, 1u, 0u, &score); /* phase 0, low 32-bit half */
    nd_error(0x41000004u, 1u, 0u, &score); /* phase 0, high 32-bit half */
    nd_error(0x41000038u, 1u, 0u, &score); /* phase 7, low 32-bit half */
    nd_error(0x4100003cu, 1u, 0u, &score); /* phase 7, high 32-bit half */
    assert(score.word_errors_by_phase[0]==2);
    for (unsigned phase=1; phase<7; ++phase) assert(score.word_errors_by_phase[phase]==0);
    assert(score.word_errors_by_phase[7]==2);
    return 0;
}
''')

    def test_x32_bit_calibration_error_callback_keeps_32bit_phase_bins(self):
        source = (INCLUDE / "native_bit_calibration.h").read_text()
        struct_start = source.index("struct nd_score {")
        struct = source[struct_start:source.index("\n};", struct_start) + 3]
        function_start = source.index("static int nd_error(")
        function = source[function_start:source.index("\n}", function_start) + 3]
        data_map_source = (INCLUDE / "native_data_map.h").read_text()
        data_map_start = data_map_source.index("static inline uint32_t usnative_cpu_word_dq_mask(")
        data_map = data_map_source[data_map_start:data_map_source.index("\n}", data_map_start) + 3]
        self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#define SDRAM_PHY_DATABITS 32
''' + data_map + struct + function + r'''
int main(void) {
    struct nd_score score;
    memset(&score, 0, sizeof(score));
    nd_error(0x41000000u, 1u, 0u, &score);
    nd_error(0x41000004u, 1u, 0u, &score);
    nd_error(0x4100001cu, 1u, 0u, &score);
    assert(score.word_errors_by_phase[0]==1);
    assert(score.word_errors_by_phase[1]==1);
    assert(score.word_errors_by_phase[7]==1);
    return 0;
}
''')

    def test_x64_boot_calibration_programs_each_of_eight_gate_lanes(self):
        source = (INCLUDE / "native_burst_calibration.h").read_text()
        start = source.index("static void nb_set_gate(")
        function = source[start:source.index("\n}\n", start) + 3]
        declarations = "\n".join(
            f"static unsigned delay{i}, width{i}; static void ddrphy_gate_delay{i}_write(unsigned v) {{delay{i}=v;}} "
            f"static void ddrphy_gate_width{i}_write(unsigned v) {{width{i}=v;}}"
            for i in range(8))
        self.compile_run(r'''
#include <assert.h>
#define SDRAM_PHY_DATABITS 64
''' + declarations + "\n" + function + r'''
int main(void) {
    for (unsigned lane=0; lane<8; ++lane) nb_set_gate(lane, 40+lane);
    assert(delay0==40 && delay1==41 && delay2==42 && delay3==43);
    assert(delay4==44 && delay5==45 && delay6==46 && delay7==47);
    assert(width0==1 && width1==1 && width2==1 && width3==1);
    assert(width4==1 && width5==1 && width6==1 && width7==1);
    return 0;
}
''')

    def profile_low_rate_1to4(self, clock, cl, cwl, rd, wr):
        source = self.profile(2400)
        source = source.replace("#define SDRAM_PHY_DATABITS 16", "#define SDRAM_PHY_DATABITS 32")
        source = source.replace("#define SDRAM_PHY_DFI_DATABITS 32",
                                "#define SDRAM_PHY_DFI_DATABITS 64")
        source = source.replace("#define SDRAM_PHY_CL 17", f"#define SDRAM_PHY_CL {cl}")
        source = source.replace("#define SDRAM_PHY_CWL 12", f"#define SDRAM_PHY_CWL {cwl}")
        source = source.replace("#define SDRAM_PHY_RDPHASE 2", f"#define SDRAM_PHY_RDPHASE {rd}")
        source = source.replace("#define SDRAM_PHY_WRPHASE 3", f"#define SDRAM_PHY_WRPHASE {wr}")
        source = source.replace("#define CONFIG_CLOCK_FREQUENCY 300000000",
                                f"#define CONFIG_CLOCK_FREQUENCY {clock}")
        source = source.replace("if(USNATIVE_TX_SCAN_END!=176)",
                                "if(USNATIVE_TX_SCAN_END!=240)")
        source = source.replace("if(selected!=2) return 1;",
                                f"if(selected!={rd}) return 1;")
        source = source.replace("if(USNATIVE_RDPHASE!=2) return 2;",
                                f"if(USNATIVE_RDPHASE!={rd}) return 2;")
        source = source.replace("return selected!=3;", f"return selected!={wr};")
        return source

    def test_low_rate_1to4_profiles_match_generated_parameters(self):
        profiles = ((200000000, 11, 9, 0, 2),
                    (233333333, 13, 10, 2, 1),
                    (266666667, 15, 11, 0, 0))
        for clock, cl, cwl, rd, wr in profiles:
            with self.subTest(clock=clock, cl=cl, cwl=cwl, rd=rd, wr=wr):
                self.compile_run(self.profile_low_rate_1to4(
                    clock, cl, cwl, rd, wr))

    def test_low_rate_1to4_profiles_reject_timing_or_geometry_mismatches(self):
        source = self.profile_low_rate_1to4(200000000, 11, 9, 0, 2)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_CWL 9", "#define SDRAM_PHY_CWL 10"), False)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_RDPHASE 0", "#define SDRAM_PHY_RDPHASE 1"), False)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_WRPHASE 2", "#define SDRAM_PHY_WRPHASE 3"), False)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_DFI_DATABITS 64", "#define SDRAM_PHY_DFI_DATABITS 32"), False)

    def test_low_rate_dual_slot_1to8_profiles_match_generated_parameters(self):
        for clock, cl, cwl, rd, wr in ((100000000, 11, 9, 0, 2),
                                       (116666666, 13, 10, 2, 1),
                                       (133333333, 15, 11, 0, 0)):
            with self.subTest(clock=clock, cl=cl, cwl=cwl, rd=rd, wr=wr):
                self.compile_run(self.profile_low_rate_dual_slot_1to8(
                    clock, cl, cwl, rd, wr))

    def test_low_rate_dual_slot_1to8_profiles_reject_timing_or_geometry_mismatches(self):
        source = self.profile_low_rate_dual_slot_1to8(100000000, 11, 9, 0, 2)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_CWL 9", "#define SDRAM_PHY_CWL 10"), False)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_RDPHASE 0", "#define SDRAM_PHY_RDPHASE 1"), False)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_WRPHASE 2", "#define SDRAM_PHY_WRPHASE 3"), False)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_DFI_DATABITS 64", "#define SDRAM_PHY_DFI_DATABITS 32"), False)

    def test_high_rate_dual_slot_1to8_profiles_accept_nominal_clock_roundings(self):
        for rate, clocks in ((2933, (183333333, 183333334)), (3200, (200000000,))):
            for clock in clocks:
                with self.subTest(rate=rate, clock=clock):
                    self.compile_run(self.profile_high_rate_dual_slot_1to8(rate, clock))

    def test_high_rate_dual_slot_1to8_profiles_reject_wrong_geometry_or_phase(self):
        source = self.profile_high_rate_dual_slot_1to8(2933, 183333333)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_RDPHASE 2", "#define SDRAM_PHY_RDPHASE 0"), False)
        self.compile_run(self.profile_high_rate_dual_slot_1to8(
            2933, 183333333, dual_slot=False), False)
        self.compile_run(self.profile_high_rate_dual_slot_1to8(
            2933, 183333333).replace(
                "#define CONFIG_SDRAM_USNATIVE_OVERCLOCK", ""), False)

    def test_2667_dual_slot_1to8_accepts_both_half_rate_roundings(self):
        for clock in (166666666, 166666667):
            with self.subTest(clock=clock):
                source = self.profile_2667_dual_slot_1to8(clock)
                source = source.replace(
                    "USNATIVE_WR_COMMAND(1);",
                    "USNATIVE_WR_COMMAND(1);\n"
                    " if(sizeof(usnative_dfi_word_t)!=8 || !USNATIVE_DUAL_SLOT_FULL_WIDTH) return 6;\n"
                    " if(USNATIVE_LANES!=4 || USNATIVE_LANE_MASK!=15) return 7;\n"
                    " if(USNATIVE_RDPHASE!=0) return 8;\n")
                self.compile_run(source)

    def test_2667_dual_slot_1to8_rejects_wrong_phase_and_single_slot_geometry(self):
        source = self.profile_2667_dual_slot_1to8(166666667)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_RDPHASE 0", "#define SDRAM_PHY_RDPHASE 2"), False)
        self.compile_run(source.replace(
            "#define SDRAM_PHY_WRPHASE 1", "#define SDRAM_PHY_WRPHASE 3"), False)
        source = self.profile_2667_dual_slot_1to8(166666667, dual_slot=False)
        self.compile_run(source, False)

    def test_default_profiles_without_dma_or_debug_csrs(self):
        for rate in (2400, 2667):
            with self.subTest(rate=rate):
                self.compile_run(self.profile(rate, check="USNATIVE_DEBUG(unknown_debug_function()); USNATIVE_SNAPSHOT();"))

    def test_1to8_x32_profile_accepts_32bit_dfi_phases(self):
        source = self.profile(2400).replace(
            "#define SDRAM_PHY_DATABITS 16", "#define SDRAM_PHY_DATABITS 32")
        source = source.replace("#define SDRAM_PHY_PHASES 4", "#define SDRAM_PHY_PHASES 8")
        source = source.replace("#define CONFIG_CLOCK_FREQUENCY 300000000",
                                "#define CONFIG_CLOCK_FREQUENCY 150000000")
        source = source.replace("#include \"profile.h\"", "#include <stdint.h>\n#include \"profile.h\"")
        source = source.replace("if(USNATIVE_TX_SCAN_END!=176)",
                                "if(USNATIVE_TX_SCAN_END!=240)")
        source = source.replace(
            "static int selected=-1;",
            "static int selected=-1;\n" + "\n".join(
                f"static void command_p{i}(unsigned value) {{ (void)value; selected={i}; }}"
                for i in range(4, 8)))
        self.compile_run(source.replace(
            "return selected!=3;",
            "if (sizeof(usnative_dfi_word_t)!=4 || SDRAM_PHY_PHASES!=8) return 5;\n"
            "if (USNATIVE_LANES!=4 || USNATIVE_LANE_MASK!=15) return 6;\n"
            "if (USNATIVE_TX_SCAN_END!=240) return 7;\n"
            "return selected!=3;"))

    def test_x32_1to4_uses_extended_tx_scan_bound(self):
        source = self.profile(2400).replace(
            "#define SDRAM_PHY_DATABITS 16", "#define SDRAM_PHY_DATABITS 32")
        source = source.replace("#define SDRAM_PHY_DFI_DATABITS 32", "#define SDRAM_PHY_DFI_DATABITS 64")
        source = source.replace("if(USNATIVE_TX_SCAN_END!=176)",
                                "if(USNATIVE_TX_SCAN_END!=240)")
        source = source.replace("return selected!=3;",
            "if (USNATIVE_TX_SCAN_END!=240) return 5;\nreturn selected!=3;")
        self.compile_run(source)

    def test_1to8_rejects_64bit_dfi_phase_geometry(self):
        source = self.profile(2400).replace(
            "#define SDRAM_PHY_DATABITS 16", "#define SDRAM_PHY_DATABITS 32")
        source = source.replace("#define SDRAM_PHY_DFI_DATABITS 32", "#define SDRAM_PHY_DFI_DATABITS 64")
        source = source.replace("#define SDRAM_PHY_PHASES 4", "#define SDRAM_PHY_PHASES 8")
        source = source.replace("#define CONFIG_CLOCK_FREQUENCY 300000000",
                                "#define CONFIG_CLOCK_FREQUENCY 150000000")
        source = source.replace("#include \"profile.h\"", "#include <stdint.h>\n#include \"profile.h\"")
        self.compile_run(source, False)

    def test_dual_slot_1to8_accepts_opt_in_64bit_dfi_phases(self):
        source = self.profile(2400).replace(
            "#define SDRAM_PHY_DATABITS 16", "#define SDRAM_PHY_DATABITS 32")
        source = source.replace("#define SDRAM_PHY_DFI_DATABITS 32", "#define SDRAM_PHY_DFI_DATABITS 64")
        source = source.replace("#define SDRAM_PHY_PHASES 4", "#define SDRAM_PHY_PHASES 8")
        source = source.replace("#define CONFIG_CLOCK_FREQUENCY 300000000",
                                "#define CONFIG_CLOCK_FREQUENCY 150000000")
        source = source.replace("#include \"profile.h\"",
            "#define CONFIG_SDRAM_USNATIVE_DUAL_SLOT\n#include <stdint.h>\n#include \"profile.h\"")
        source = source.replace(
            "if(USNATIVE_TX_SCAN_END!=176)",
            "if(sizeof(usnative_dfi_word_t)!=8 || !USNATIVE_DUAL_SLOT_FULL_WIDTH) return 5;\n"
            "if(USNATIVE_LANES!=4 || USNATIVE_LANE_MASK!=15) return 6;\n"
            "if(USNATIVE_TX_SCAN_END!=240)")
        self.compile_run(source)

    def test_ck_probe_64bit_diagnostics_print_high_and_low_halves_explicitly(self):
        calibration = (INCLUDE / "native_burst_calibration.h").read_text()
        ck_probe = calibration[calibration.index('"CK_PROBE ready='):calibration.index("#else\n            USNATIVE_DEBUG(\"CK_PROBE ready=", calibration.index('"CK_PROBE ready='))]
        self.assertNotIn("%016llx", ck_probe)
        self.assertIn("expected-p0=%08x%08x", ck_probe)
        self.assertIn("(unsigned)(q[0]>>32), (unsigned)q[0]", ck_probe)
        self.assertIn("p0=%08x%08x", ck_probe)
        self.assertIn("(unsigned)(r0>>32), (unsigned)r0", ck_probe)
        output = self.compile_run(r'''
#include <stdio.h>
#include <stdint.h>
int main(void) {
    uint64_t value = UINT64_C(0x89abcdef12345678);
    printf("value=%08x%08x\n", (unsigned)(value >> 32), (unsigned)value);
    return 0;
}
''')
        self.assertEqual(output, "value=89abcdef12345678\n")

    def test_dual_slot_8x64_direct_probe_repeats_one_bl8_into_both_slots(self):
        calibration = (INCLUDE / "native_burst_calibration.h").read_text()
        self.assertIn("if (lane==0 && tap==USNATIVE_BOOT_RX_DELAY) nb_seed_probe_kind=1;", calibration)
        self.assertIn("if (lane==0 && tap==USNATIVE_BOOT_TX_DELAY) nb_seed_probe_kind=2;", calibration)
        start = calibration.index("static unsigned nb_check(")
        check = calibration[start:calibration.index("\n}\n", start) + 3]
        phase_io = "\n".join(
            f"static void sdram_dfii_pi{i}_wrdata_write(uint64_t v) "
            f"{{ wr[{i}]=v; if (!written[{i}]) {{ first_wr[{i}]=v; written[{i}]=1; }} }}\n"
            f"static uint64_t sdram_dfii_pi{i}_rddata_read(void) "
            f"{{ ++rd_reads[{i}]; return wr[{i}]; }}"
            for i in range(8))
        address_io = "\n".join(
            f"static void sdram_dfii_pi{i}_address_write(unsigned v) {{ (void)v; }}\n"
            f"static void sdram_dfii_pi{i}_baddress_write(unsigned v) {{ (void)v; }}\n"
            f"static void command_p{i}(unsigned v) {{ (void)v; }}"
            for i in range(4))
        self.compile_run(r'''
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define SDRAM_PHY_DDR4
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_DFI_DATABITS 64
#define SDRAM_PHY_PHASES 8
#define SDRAM_PHY_XDR 2
#define SDRAM_PHY_DELAYS 512
#define SDRAM_PHY_BITSLIPS 8
#define SDRAM_PHY_CMD_LATENCY 5
#define MAIN_RAM_BASE 0x40000000
#define MAIN_RAM_SIZE 0x01200000
#define CSR_DDRPHY_RIU_BUSY_ADDR 1
#define CSR_DDRPHY_RIU_VALID_ADDR 1
#define CSR_DDRPHY_TAP_STATUS_VALID_ADDR 1
#define CONFIG_CLOCK_FREQUENCY 150000000
#define SDRAM_PHY_CL 17
#define SDRAM_PHY_CWL 12
#define SDRAM_PHY_RDPHASE 2
#define SDRAM_PHY_WRPHASE 3
#define CONFIG_SDRAM_USNATIVE_DUAL_SLOT
#define CONFIG_SDRAM_USNATIVE_DEBUG
#define CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC
#define CONFIG_SDRAM_USNATIVE_WRITE_MONITOR
#include "profile.h"
#include "native_burst_pattern.h"
#define DFII_COMMAND_RAS 1
#define DFII_COMMAND_CAS 2
#define DFII_COMMAND_WE 4
#define DFII_COMMAND_CS 8
#define DFII_COMMAND_RDDATA 16
#define DFII_COMMAND_WRDATA 32
#define CSR_DDRPHY_FIFO_EMPTY_ADDR 77
static char debug_log[4096];
static void debug_capture(const char *format, ...) {
 va_list args; va_start(args, format);
 vsnprintf(debug_log+strlen(debug_log), sizeof(debug_log)-strlen(debug_log), format, args);
 va_end(args);
}
#define printf debug_capture
#define USNATIVE_DEBUG(...) debug_capture(__VA_ARGS__)
static uint64_t wr[8], first_wr[8];
static unsigned rd_reads[8], written[8], nb_probe_epoch;
static unsigned nb_seed_probe_kind;
static unsigned nb_dfi_diag_active=1;
static unsigned nb_read_phase=USNATIVE_RDPHASE;
static void cdelay(unsigned v) {(void)v;}
static void ddrphy_probe_fast_clear_write(unsigned v) {(void)v;}
static unsigned ddrphy_probe_fast_command_read(void) {return 0x12;}
static unsigned ddrphy_probe_fast_address_read(void) {return 0x3456;}
static unsigned ddrphy_probe_fast_wren_read(void) {return 7;}
static unsigned ddrphy_probe_fast_oe_read(void) {return 8;}
static unsigned ddrphy_probe_dqs_drive_read(void) {return 9;}
static unsigned ddrphy_probe_not_ready_read(void) {return 0;}
static unsigned ddrphy_probe_word0_read(void) {return 0x101;}
static unsigned ddrphy_probe_word1_read(void) {return 0x202;}
static unsigned ddrphy_probe_word2_read(void) {return 0x303;}
static unsigned ddrphy_probe_word3_read(void) {return 0x404;}
static unsigned ddrphy_probe_word4_read(void) {return 0x505;}
static unsigned ddrphy_probe_word5_read(void) {return 0x606;}
static void nb_issue_read(unsigned address) {(void)address;}
static unsigned ddrphy_ready_read(void) {return 1;}
static unsigned ddrphy_debug_read(void) {return 0;}
static unsigned csr_read_simple(unsigned address) {(void)address; return 0;}
static unsigned ddrphy_tap_rx_count_read(void) {return 0;}
static unsigned ddrphy_tap_tx_count_read(void) {return 0;}
''' + address_io + "\n" + phase_io + r'''
''' + check + r'''
int main(void) {
 char expected_pair[32];
 if (!USNATIVE_DUAL_SLOT_FULL_WIDTH || sizeof(usnative_dfi_word_t)!=8) return 1;
 nb_seed_probe_kind=1;
 if (nb_check(64, USNATIVE_LANE_MASK)!=0) return 2;
 if (!strstr(debug_log, "NATIVE_SEED_PROBE dir=RX lane=0 tap=0 rd_cmd_phase=2 wr_cmd_phase=3 ready=1 fifo=00000000 status=00000000")) return 7;
 uint64_t expected=usnative_burst_phase_pattern(0, 0x9e3779c1u);
 snprintf(expected_pair, sizeof(expected_pair), "%08x:%08x",
  (unsigned)(expected>>32), (unsigned)expected);
 if (!strstr(debug_log, "NATIVE_SEED_PROBE expected=") || !strstr(debug_log, " observed=")) return 8;
 if (!strstr(debug_log, "NATIVE_DFI_TX rdphase=2 wrphase=3 wr_cmd_phase=3 address=040 chunks=")) return 12;
 if (!strstr(debug_log, "NATIVE_DFI_RX rd_cmd_phase=2 errors=")) return 13;
 if (!strstr(debug_log, "NATIVE_DFI_FAST_TX phase3_cmd=12 address=03456 wren=7 oe=8 dqs=9 not_ready=0 chunks=00000101,00000202,00000303,00000404,00000505,00000606")) return 14;
 if (!strstr(debug_log, expected_pair)) return 11;
 nb_seed_probe_kind=2;
 if (nb_check(64, USNATIVE_LANE_MASK)!=0) return 9;
 if (!strstr(debug_log, "NATIVE_SEED_PROBE dir=TX lane=0 tap=0 rd_cmd_phase=2 wr_cmd_phase=3 ready=1 fifo=00000000 status=00000000")) return 10;
 for (unsigned phase=0; phase<4; ++phase) {
  if (first_wr[phase]!=usnative_burst_phase_pattern(phase, 0x9e3779b9u)) return 3;
  if (first_wr[phase+4]!=first_wr[phase]) return 5;
  if (!rd_reads[phase]) return 4;
 }
 for (unsigned phase=4; phase<8; ++phase) {
  if (rd_reads[phase]) return 6;
 }
 return 0;
}
''')

    def test_optional_dfi_diagnostic_restores_nominal_phase_and_boot(self):
        calibration = (INCLUDE / "native_burst_calibration.h").read_text()
        signature = "static int nb_dfi_diagnostic(void)"
        start = calibration.index(signature)
        diagnostic = calibration[start:calibration.index("\n}\n", start) + 3]
        output = self.compile_run(r'''
#include <assert.h>
#include <stdio.h>
#define CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_PHASES 8
#define USNATIVE_DUAL_SLOT_FULL_WIDTH 1
#define USNATIVE_RDPHASE 2
#define USNATIVE_WRPHASE 0
#define USNATIVE_LANES 4
#define USNATIVE_LANE_MASK 15
#define USNATIVE_BOOT_RX_DELAY 32
#define USNATIVE_BOOT_TX_DELAY 88
#define USNATIVE_DEBUG(...) do {} while (0)
static unsigned nb_read_phase=USNATIVE_RDPHASE;
static unsigned nb_dfi_diag_active;
static unsigned nb_seed_rx[USNATIVE_LANES], nb_seed_tx[USNATIVE_LANES];
static unsigned boots, checks, written_phase=99;
static int nb_boot(unsigned ck) {
    assert(ck==0);
    assert(nb_read_phase==USNATIVE_RDPHASE || nb_read_phase==2);
    ++boots;
    return 1;
}
static unsigned nb_check(unsigned count, unsigned lanes) {
    assert(count==1 && lanes==USNATIVE_LANE_MASK);
    assert(nb_dfi_diag_active==1);
    ++checks;
    return 0;
}
static void ddrphy_rdphase_write(unsigned phase) {
    assert(phase<4);
    written_phase=phase;
}
static void cdelay(unsigned cycles) {(void)cycles;}
''' + diagnostic + r'''
int main(void) {
    nb_read_phase=USNATIVE_RDPHASE;
    assert(nb_dfi_diagnostic());
    assert(boots==2 && checks==4);
    assert(nb_read_phase==USNATIVE_RDPHASE && written_phase==USNATIVE_RDPHASE);
    assert(nb_dfi_diag_active==0);
    for (unsigned lane=0; lane<USNATIVE_LANES; ++lane) {
        assert(nb_seed_rx[lane]==USNATIVE_BOOT_RX_DELAY);
        assert(nb_seed_tx[lane]==USNATIVE_BOOT_TX_DELAY);
    }
    return 0;
}
''')

    def test_optional_dfi_diagnostic_accepts_1600_1to4_profile(self):
        self.compile_run(r'''
#include <assert.h>
#include <stdio.h>
#define SDRAM_PHY_DDR4
#define CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC
#define CONFIG_SDRAM_USNATIVE_DEBUG
#define CONFIG_SDRAM_USNATIVE_WRITE_MONITOR
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_DFI_DATABITS 64
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
#define CONFIG_CLOCK_FREQUENCY 200000000
#define SDRAM_PHY_CL 11
#define SDRAM_PHY_CWL 9
#define SDRAM_PHY_RDPHASE 0
#define SDRAM_PHY_WRPHASE 2
#include <stdint.h>
#include "profile.h"
int main(void) {
    if (USNATIVE_RDPHASE != 0 || SDRAM_PHY_WRPHASE != 2) return 1;
    if (USNATIVE_DUAL_SLOT_FULL_WIDTH) return 2;
    return 0;
}
''')

    def test_1600_1to4_dfi_diagnostic_cycles_all_phase_pairs_and_restores(self):
        calibration = (INCLUDE / "native_burst_calibration.h").read_text()
        signature = "static int nb_dfi_diagnostic(void)"
        start = calibration.index(signature)
        diagnostic = calibration[start:calibration.index("\n}\n", start) + 3]
        self.compile_run(r'''
#include <assert.h>
#include <stdio.h>
#define CONFIG_SDRAM_USNATIVE_DFI_DIAGNOSTIC
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_DFI_DATABITS 64
#define SDRAM_PHY_PHASES 4
#define CONFIG_CLOCK_FREQUENCY 200000000
#define SDRAM_PHY_CL 11
#define SDRAM_PHY_CWL 9
#define SDRAM_PHY_RDPHASE 0
#define SDRAM_PHY_WRPHASE 2
#define USNATIVE_DUAL_SLOT_FULL_WIDTH 0
#define USNATIVE_RDPHASE 0
#define USNATIVE_LANES 4
#define USNATIVE_LANE_MASK 15
#define USNATIVE_BOOT_RX_DELAY 32
#define USNATIVE_BOOT_TX_DELAY 88
#define USNATIVE_DEBUG(...) do {} while (0)
static unsigned nb_read_phase=USNATIVE_RDPHASE;
static unsigned nb_write_phase=SDRAM_PHY_WRPHASE;
static unsigned nb_dfi_diag_active;
static unsigned nb_seed_rx[USNATIVE_LANES], nb_seed_tx[USNATIVE_LANES];
static unsigned boots, checks, written_phase=99, written_write_phase=99;
static unsigned seen[4][4];
static int nb_boot(unsigned ck) { assert(ck==0); ++boots; return 1; }
static unsigned nb_check(unsigned count, unsigned lanes) {
    assert(count==1 && lanes==USNATIVE_LANE_MASK && nb_dfi_diag_active);
    assert(nb_read_phase < 4 && nb_write_phase < 4);
    assert(written_phase==nb_read_phase && written_write_phase==nb_write_phase);
    ++seen[nb_write_phase][nb_read_phase];
    ++checks;
    return 0;
}
static void ddrphy_rdphase_write(unsigned phase) { assert(phase<4); written_phase=phase; }
static void ddrphy_wrphase_write(unsigned phase) { assert(phase<4); written_write_phase=phase; }
static void cdelay(unsigned cycles) {(void)cycles;}
''' + diagnostic + r'''
int main(void) {
    assert(nb_dfi_diagnostic());
    assert(boots==2 && checks==16);
    for (unsigned wr=0; wr<4; ++wr)
        for (unsigned rd=0; rd<4; ++rd)
            assert(seen[wr][rd]==1);
    assert(nb_read_phase==0 && written_phase==0 && nb_dfi_diag_active==0);
    assert(nb_write_phase==2 && written_write_phase==2);
    return 0;
}
''')

    def test_fixed_row_diagnostic_issues_two_reads_per_phase_pair(self):
        calibration = (INCLUDE / "native_burst_calibration.h").read_text()

        def extract_function(signature):
            start = calibration.index(signature)
            brace = calibration.index("{", start)
            depth = 0
            for index in range(brace, len(calibration)):
                if calibration[index] == "{":
                    depth += 1
                elif calibration[index] == "}":
                    depth -= 1
                    if depth == 0:
                        return calibration[start:index + 1]
            raise AssertionError(f"unterminated function: {signature}")

        capture = extract_function("static unsigned nb_fixed_row_capture(")
        diagnostic = extract_function("static unsigned nb_fixed_row_diagnostic(")
        output = self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define DFII_COMMAND_RAS 1u
#define DFII_COMMAND_CAS 2u
#define DFII_COMMAND_WE 4u
#define DFII_COMMAND_CS 8u
#define DFII_COMMAND_RDDATA 16u
#define DFII_COMMAND_WRDATA 32u
#define CSR_DDRPHY_FIFO_EMPTY_ADDR 77u
#define NB_FIXED_ROW_SETTLE_CYCLES 1000u
static unsigned nb_write_phase, nb_read_phase;
static unsigned write_phase_reg, read_phase_reg;
static unsigned precharges, activates, writes, reads, corrupt_phase=99;
static unsigned trace_arms, trace_arm_pending;
static unsigned rddata_valid_cycles;
static const uint64_t expected[4]={
    0x96a53cc35aa5c33cu, 0x1e4b78d2a569c30fu,
    0xf00f69969669f00fu, 0x8731c6e25a0d4bf8u
};
static uint64_t wrdata[4], captured[4];
static void command_p0(unsigned command) {
    if (command==(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS)) ++precharges;
    else if (command==(DFII_COMMAND_RAS|DFII_COMMAND_CS)) ++activates;
    else assert(0);
}
static void ddrphy_wrphase_write(unsigned phase) { write_phase_reg=phase; }
static void ddrphy_rdphase_write(unsigned phase) { read_phase_reg=phase; }
static unsigned ddrphy_ready_read(void) { return 1; }
static unsigned ddrphy_debug_read(void) { return 0x1234; }
static unsigned csr_read_simple(unsigned address) { assert(address==77); return 0x0f; }
static void sdram_dfii_pi0_address_write(unsigned value) { (void)value; }
static void sdram_dfii_pi0_baddress_write(unsigned value) { assert(value==0); }
static void sdram_dfii_pi0_wrdata_write(uint64_t value) { wrdata[0]=value; }
static void sdram_dfii_pi1_wrdata_write(uint64_t value) { wrdata[1]=value; }
static void sdram_dfii_pi2_wrdata_write(uint64_t value) { wrdata[2]=value; }
static void sdram_dfii_pi3_wrdata_write(uint64_t value) { wrdata[3]=value; }
static uint64_t sdram_dfii_pi0_rddata_read(void) { return captured[0]; }
static uint64_t sdram_dfii_pi1_rddata_read(void) { return captured[1]; }
static uint64_t sdram_dfii_pi2_rddata_read(void) { return captured[2]; }
static uint64_t sdram_dfii_pi3_rddata_read(void) { return captured[3]; }
static unsigned native_dfi_activity_rddata_valid_cycles_read(void) { return rddata_valid_cycles; }
static void ddrphy_rx_trace_arm_write(unsigned value) {
    assert(value==1 && !trace_arm_pending);
    ++trace_arms;
    trace_arm_pending=1;
}
static void nb_issue_write(unsigned address) {
    assert(address==0 && write_phase_reg==nb_write_phase);
    for (unsigned phase=0; phase<4; ++phase) assert(wrdata[phase]==expected[phase]);
    ++writes;
}
static void nb_issue_read(unsigned address) {
    assert(address==0 && read_phase_reg==nb_read_phase);
    if (nb_write_phase==2 && nb_read_phase==0 && reads==16)
        assert(trace_arms==1 && !trace_arm_pending);
    for (unsigned phase=0; phase<4; ++phase) captured[phase]=expected[phase];
    if (corrupt_phase<4) captured[corrupt_phase]^=1;
    rddata_valid_cycles += 3;
    ++reads;
}
static void cdelay(unsigned cycles) {
    assert(cycles==100 || cycles==1000);
    if (trace_arm_pending) {
        assert(cycles>=100);
        trace_arm_pending=0;
    }
}
''' + capture + diagnostic + r'''
int main(void) {
    assert(nb_fixed_row_diagnostic()==0);
    assert(writes==16 && reads==32 && activates==16 && precharges==32);
    assert(trace_arms==1 && !trace_arm_pending);
    for (unsigned phase=0; phase<4; ++phase) assert(wrdata[phase]==expected[phase]);
    writes=reads=activates=precharges=0;
    trace_arms=0;
    corrupt_phase=2;
    assert(nb_fixed_row_diagnostic()==32);
    assert(writes==16 && reads==32 && activates==16 && precharges==32);
    assert(trace_arms==1 && !trace_arm_pending);
    return 0;
}
''')
        self.assertEqual(output.count("valid_delta=3"), 128)
        self.assertEqual(output.count("DDR4 fixed-row diagnostic:"), 2)
        self.assertIn("DDR4 fixed-row read: wr_cmd_phase=0 rd_cmd_phase=0 valid_delta=3", output)

    def test_tagged_read_diagnostic_associates_two_distinct_bursts_in_order(self):
        calibration = (INCLUDE / "native_burst_calibration.h").read_text()

        def extract_function(signature):
            start = calibration.index(signature)
            brace = calibration.index("{", start)
            depth = 0
            for index in range(brace, len(calibration)):
                if calibration[index] == "{":
                    depth += 1
                elif calibration[index] == "}":
                    depth -= 1
                    if depth == 0:
                        return calibration[start:index + 1]
            raise AssertionError(f"unterminated function: {signature}")

        capture = extract_function("static unsigned nb_tagged_read_capture(")
        diagnostic = extract_function("static unsigned nb_tagged_read_diagnostic(")
        output = self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define CONFIG_SDRAM_USNATIVE_MPR_GATE_WIDTH2
#define CONFIG_SDRAM_USNATIVE_MPR_LANE0_GATE_OFFSET -1
#define NB_TAGGED_READ_SETTLE_CYCLES 1000u
#define CSR_DDRPHY_FIFO_EMPTY_ADDR 77u
#define DFII_COMMAND_RAS 1u
#define DFII_COMMAND_CAS 2u
#define DFII_COMMAND_WE 4u
#define DFII_COMMAND_CS 8u
#define DFII_COMMAND_RDDATA 16u
#define DFII_COMMAND_WRDATA 32u
static unsigned nb_write_phase=3, nb_read_phase=1;
static unsigned write_phase_reg, read_phase_reg;
static unsigned write_count, read_count, command_count, trace_arms, armed;
static unsigned corrupt_second;
static unsigned global_gate=3, lane_gate[4]={4,5,6,7};
static unsigned lane_width[4]={1,1,1,1};
static uint64_t wrdata[4], captured[4];
static const uint64_t tag_a[4]={
    0x13579bdf2468ace0ull, 0x0f1e2d3c4b5a6978ull,
    0x55aa33cc699696f0ull, 0x89abcdef01234567ull
};
static const uint64_t tag_b[4]={
    0xcafef00d5a5aa5a5ull, 0xdeadbeef87654321ull,
    0xa55ac33c9669690full, 0x76543210fedcba98ull
};
static void command_p0(unsigned command) {
    if (command_count==0 || command_count==2)
        assert(command==(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS));
    else assert(command==(DFII_COMMAND_RAS|DFII_COMMAND_CS));
    ++command_count;
}
static void ddrphy_wrphase_write(unsigned phase) { write_phase_reg=phase; }
static void ddrphy_rdphase_write(unsigned phase) { read_phase_reg=phase; }
static unsigned ddrphy_gate_delay_read(void) { return global_gate; }
static unsigned ddrphy_gate_delay0_read(void) { return lane_gate[0]; }
static unsigned ddrphy_gate_delay1_read(void) { return lane_gate[1]; }
static unsigned ddrphy_gate_delay2_read(void) { return lane_gate[2]; }
static unsigned ddrphy_gate_delay3_read(void) { return lane_gate[3]; }
static void ddrphy_gate_delay_write(unsigned value) { global_gate=value; }
static void ddrphy_gate_delay0_write(unsigned value) { lane_gate[0]=value; }
static void ddrphy_gate_delay1_write(unsigned value) { lane_gate[1]=value; }
static void ddrphy_gate_delay2_write(unsigned value) { lane_gate[2]=value; }
static void ddrphy_gate_delay3_write(unsigned value) { lane_gate[3]=value; }
static unsigned ddrphy_gate_width0_read(void) { return lane_width[0]; }
static unsigned ddrphy_gate_width1_read(void) { return lane_width[1]; }
static unsigned ddrphy_gate_width2_read(void) { return lane_width[2]; }
static unsigned ddrphy_gate_width3_read(void) { return lane_width[3]; }
static void ddrphy_gate_width0_write(unsigned value) { lane_width[0]=value; }
static void ddrphy_gate_width1_write(unsigned value) { lane_width[1]=value; }
static void ddrphy_gate_width2_write(unsigned value) { lane_width[2]=value; }
static void ddrphy_gate_width3_write(unsigned value) { lane_width[3]=value; }
static void ddrphy_rx_trace_arm_write(unsigned value) {
    assert(value==1 && read_count==1 && !armed);
    ++trace_arms; armed=1;
}
static unsigned csr_read_simple(unsigned address) { assert(address==77); return 0; }
static unsigned native_dfi_activity_rddata_valid_cycles_read(void) { return read_count; }
static void sdram_dfii_pi0_address_write(unsigned value) { (void)value; }
static void sdram_dfii_pi0_baddress_write(unsigned value) { assert(value==0); }
static void sdram_dfii_pi0_wrdata_write(uint64_t value) { wrdata[0]=value; }
static void sdram_dfii_pi1_wrdata_write(uint64_t value) { wrdata[1]=value; }
static void sdram_dfii_pi2_wrdata_write(uint64_t value) { wrdata[2]=value; }
static void sdram_dfii_pi3_wrdata_write(uint64_t value) { wrdata[3]=value; }
static uint64_t sdram_dfii_pi0_rddata_read(void) { return captured[0]; }
static uint64_t sdram_dfii_pi1_rddata_read(void) { return captured[1]; }
static uint64_t sdram_dfii_pi2_rddata_read(void) { return captured[2]; }
static uint64_t sdram_dfii_pi3_rddata_read(void) { return captured[3]; }
static void nb_issue_write(unsigned address) {
    assert(write_phase_reg==2 && address==(write_count ? 8u : 0u));
    const uint64_t *expected=write_count ? tag_b : tag_a;
    for (unsigned phase=0; phase<4; ++phase) assert(wrdata[phase]==expected[phase]);
    ++write_count;
}
static void nb_issue_read(unsigned address) {
    assert(read_phase_reg==0 && address==(read_count ? 8u : 0u));
    assert(global_gate==2 && lane_gate[0]==1 && lane_gate[1]==2 &&
        lane_gate[2]==2 && lane_gate[3]==2);
    for (unsigned lane=0; lane<4; ++lane) assert(lane_width[lane]==2);
    if (read_count==1) assert(armed);
    const uint64_t *expected=read_count ? tag_b : tag_a;
    for (unsigned phase=0; phase<4; ++phase) captured[phase]=expected[phase];
    if (read_count==1 && corrupt_second) captured[2]^=1;
    ++read_count;
}
static void cdelay(unsigned cycles) {
    assert(cycles==100 || cycles==1000);
    if (armed && read_count==2) armed=0;
}
''' + capture + diagnostic + r'''
int main(void) {
    assert(nb_tagged_read_diagnostic()==0);
    assert(write_count==2 && read_count==2 && trace_arms==1);
    assert(command_count==3 && write_phase_reg==3 && read_phase_reg==1);
    assert(global_gate==3 && lane_gate[0]==4 && lane_gate[1]==5 &&
        lane_gate[2]==6 && lane_gate[3]==7);
    for (unsigned lane=0; lane<4; ++lane) assert(lane_width[lane]==1);
    write_count=read_count=command_count=trace_arms=0;
    corrupt_second=1;
    assert(nb_tagged_read_diagnostic()==1);
    assert(write_count==2 && read_count==2 && trace_arms==1);
    assert(command_count==3 && write_phase_reg==3 && read_phase_reg==1);
    assert(global_gate==3 && lane_gate[0]==4 && lane_gate[1]==5 &&
        lane_gate[2]==6 && lane_gate[3]==7);
    for (unsigned lane=0; lane<4; ++lane) assert(lane_width[lane]==1);
    return 0;
}
''')
        self.assertEqual(output.count("DDR4 tagged read: burst="), 4)
        self.assertIn("DDR4 tagged read: burst=0 column=0", output)
        self.assertIn("DDR4 tagged read: burst=1 column=8", output)
        self.assertEqual(output.count("valid_delta=1"), 4)
        self.assertIn("DDR4 tagged two-burst diagnostic complete: errors=1", output)

    def test_mpr_lane_gate_sweep_scores_each_byte_and_repeats_selected_vector(self):
        firmware = (INCLUDE / "native_mpr_probe.h").read_text()

        def extract_function(signature):
            start = firmware.index(signature)
            brace = firmware.index("{", start)
            depth = 0
            for index in range(brace, len(firmware)):
                if firmware[index] == "{":
                    depth += 1
                elif firmware[index] == "}":
                    depth -= 1
                    if depth == 0:
                        return firmware[start:index + 1]
            raise AssertionError(f"unterminated function: {signature}")

        helpers = "\n".join(extract_function(signature) for signature in (
            "static int nb_mpr_lane_word_matches(",
            "static unsigned nb_mpr_lane_fifo_reads(",
            "static void nb_mpr_mrs(",
            "static void nb_mpr_lane_location_read(",
            "static void nb_mpr_lane_sweep_read(",
            "static unsigned nb_mpr_valid_count(",
            "static int nb_mpr_lane_by_lane_gate_sweep(",
        ))
        output = self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define USNATIVE_LANE_MASK 0x0fu
#define NB_MPR_ENABLE_MR3 4u
#define NB_MPR_NORMAL_MR3 0u
#define NB_MPR_LANE_GATE_FIRST 1u
#define NB_MPR_LANE_GATE_LAST 3u
#define DFII_COMMAND_RAS 1u
#define DFII_COMMAND_CAS 2u
#define DFII_COMMAND_WE 4u
#define DFII_COMMAND_CS 8u
#define DFII_COMMAND_RDDATA 16u
static unsigned global_gate=4, override=3;
static unsigned gate[4]={4,5,6,7}, width[4]={1,1,1,1};
static unsigned fifo_reads[4], valid_count, mr3, mr3_address, trace_arms, trace_pending;
static unsigned best[4]={1,2,3,1};
static uint64_t words[4];
static void cdelay(unsigned cycles) {
    assert(cycles==100);
    if (trace_pending) trace_pending=0;
}
static void ddrphy_gate_delay_write(unsigned value) { global_gate=value; }
static unsigned ddrphy_gate_delay_read(void) { return global_gate; }
static void ddrphy_gate_override_write(unsigned value) { override=value; }
static unsigned ddrphy_gate_override_read(void) { return override; }
#define GATE_ACCESSORS(n) \
static void ddrphy_gate_delay##n##_write(unsigned value) { gate[n]=value; } \
static unsigned ddrphy_gate_delay##n##_read(void) { return gate[n]; } \
static void ddrphy_gate_width##n##_write(unsigned value) { width[n]=value; } \
static unsigned ddrphy_gate_width##n##_read(void) { return width[n]; } \
static unsigned ddrphy_fifo_reads##n##_read(void) { return fifo_reads[n]; }
GATE_ACCESSORS(0)
GATE_ACCESSORS(1)
GATE_ACCESSORS(2)
GATE_ACCESSORS(3)
static void ddrphy_debug_clear_write(unsigned value) {
    assert(value==1); for (unsigned lane=0; lane<4; ++lane) fifo_reads[lane]=0;
}
static unsigned native_dfi_activity_rddata_valid_cycles_read(void) { return valid_count; }
static void ddrphy_rx_trace_arm_write(unsigned value) {
    assert(value==1 && !trace_arms); ++trace_arms; trace_pending=1;
}
static void sdram_dfii_pi0_address_write(unsigned value) { mr3_address=value; }
static void sdram_dfii_pi0_baddress_write(unsigned value) { (void)value; }
static void command_p0(unsigned command) {
    if (command==(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS)) {
        mr3=mr3_address;
    } else if (command==(DFII_COMMAND_CAS|DFII_COMMAND_CS|DFII_COMMAND_RDDATA)) {
        assert(!trace_pending);
        ++valid_count;
        for (unsigned phase=0; phase<4; ++phase) words[phase]=0;
        for (unsigned lane=0; lane<4; ++lane) {
            const unsigned good=gate[lane]==best[lane];
            if (good) ++fifo_reads[lane];
            for (unsigned phase=0; phase<4; ++phase) {
                for (unsigned bit=lane*8; bit<lane*8+8; ++bit) {
                    if (good) words[phase]|=(uint64_t)1u<<(bit+32);
                    else words[phase]&=~((uint64_t)1u<<(bit+32));
                }
            }
        }
    }
}
static uint64_t sdram_dfii_pi0_rddata_read(void) { return words[0]; }
static uint64_t sdram_dfii_pi1_rddata_read(void) { return words[1]; }
static uint64_t sdram_dfii_pi2_rddata_read(void) { return words[2]; }
static uint64_t sdram_dfii_pi3_rddata_read(void) { return words[3]; }
''' + helpers + r'''
int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    uint64_t polarity_one[4], phase_alternating[4]={0,0,0,0};
    for (unsigned phase=0; phase<4; ++phase) {
        polarity_one[phase]=(uint64_t)0xffu<<8;
        if (phase & 1u) {
            phase_alternating[phase]|=(uint64_t)0xffu;
        } else {
            phase_alternating[phase]|=(uint64_t)0xffu<<32;
        }
    }
    assert(nb_mpr_lane_word_matches(polarity_one, 1));
    assert(!nb_mpr_lane_word_matches(phase_alternating, 0));
    assert(nb_mpr_lane_by_lane_gate_sweep()==0);
    assert(global_gate==4 && override==3);
    assert(gate[0]==4 && gate[1]==5 && gate[2]==6 && gate[3]==7);
    assert(width[0]==1 && width[1]==1 && width[2]==1 && width[3]==1);
    assert(mr3==0);
    assert(trace_arms==1);
    return 0;
}
''')
        self.assertEqual(output.count("DDR4 MPR lane gate: lane="), 12)
        self.assertIn("best vector: 1,2,3,1 score=3,3,3,3", output)
        self.assertEqual(output.count("result=DIAGNOSTIC_PASS"), 2)
        self.assertIn("DDR4 MPR lane gate sweep complete: diagnostic-only PASS", output)

    def test_1to8_rx_seed_fallback_uses_median_then_scan_midpoint(self):
        self.compile_run(r'''
#include "native_rx_seed.h"
int main(void) {
    if (usnative_coarse_seed_select(48,0,64,64)!=48) return 1;
    if (usnative_coarse_seed_select(48,32,17,64)!=32) return 2;
    if (usnative_coarse_seed_select(48,88,0,64)!=88) return 3;
    unsigned r5[4]={88,48,48,80};
    if (usnative_rx_seed_fallback(r5,4,0,0,96,8)!=64) return 4;
    if (usnative_rx_seed_fallback(r5,4,1,0,96,8)!=48) return 5;
    unsigned centered[4]={48,48,48,48};
    if (usnative_rx_seed_fallback(centered,4,1,0,96,8)!=56) return 6;
    unsigned r3[4]={32,48,56,80};
    if (usnative_rx_seed_fallback(r3,4,0,0,96,8)!=52) return 7;
    if (usnative_tx_seed_select(88,32,0)!=88) return 8;
    if (usnative_tx_seed_select(88,72,3)!=72) return 9;
    unsigned r8[8]={8,16,24,32,40,48,56,64};
    if (usnative_rx_seed_fallback(r8,8,0,0,96,8)!=36) return 10;
    if (usnative_rx_seed_fallback(r8,8,1,0,96,8)!=48) return 11;
    if (usnative_rx_seed_fallback(r8,9,0,0,96,8)!=0) return 12;
    return 0;
}
''')

    def test_joint_seed_selection_prefers_errors_then_nearest_stable_pair(self):
        self.compile_run(r'''
#include "native_rx_seed.h"
#include <assert.h>
int main(void) {
    struct usnative_joint_seed best={0,0,0,0};
    usnative_joint_seed_consider(&best,40,80,12,48,88);
    usnative_joint_seed_consider(&best,48,88,12,48,88);
    assert(best.valid && best.rx==48 && best.tx==88); /* equal error, closer */
    struct usnative_joint_seed tie={0,0,0,0};
    usnative_joint_seed_consider(&tie,56,88,12,48,88);
    usnative_joint_seed_consider(&tie,40,88,12,48,88);
    assert(tie.rx==40 && tie.tx==88); /* equal distance: low RX wins */
    usnative_joint_seed_consider(&best,0,32,11,48,88);
    assert(best.rx==0 && best.tx==32 && best.errors==11); /* score dominates */
    return 0;
}
''')

    def test_dq_eye_tap_updates_yield_to_refresh_in_bounded_chunks(self):
        """Exercise the real tap helper against CSR mocks with refresh monitoring."""
        if not CC:
            self.skipTest("Host C compiler required")
        with tempfile.TemporaryDirectory(prefix="usnative_eye_") as temporary:
            directory = Path(temporary)
            (directory / "generated").mkdir()
            (directory / "libbase").mkdir()
            (directory / "generated/sdram_phy.h").write_text("/* mock generated PHY */\n")
            (directory / "libbase/memtest.h").write_text(
                "#define MEMTEST_DATA_SIZE 4096\n")
            (directory / "libbase/lfsr.h").write_text(
                "static inline unsigned lfsr(unsigned n, unsigned x) "
                "{ (void)n; return x + 1; }\n")
            (directory / "system.h").write_text(r'''
#include <stddef.h>
static inline void cdelay(unsigned n) {(void)n;}
static inline void flush_cpu_dcache(void) {}
static inline void flush_l2_cache(void) {}
static inline void flush_cpu_dcache_range(void *p, size_t n) {(void)p;(void)n;}
static inline void invd_cpu_dcache_range(void *p, size_t n) {(void)p;(void)n;}
''')
            source = directory / "test.c"
            source.write_text(r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define SDRAM_PHY_DATABITS 32
#define SDRAM_PHY_DELAYS 512
#define SDRAM_PHY_USNATIVE_ABI_MAJOR 1
#define SDRAM_PHY_USNATIVE_ABI_MINOR 0
#define SDRAM_PHY_USNATIVE_CONFIG_ID 0x1234u
#define SDRAM_PHY_USNATIVE_REQUIRED_CAPS 1u
#define SDRAM_PHY_USNATIVE_LANE_COUNT 4
#define SDRAM_PHY_USNATIVE_DQ_TAPS_COUNT 32
#define SDRAM_PHY_USNATIVE_DQS_TAPS_COUNT 4
#define SDRAM_PHY_USNATIVE_DM_TAPS_COUNT 0
#define SDRAM_PHY_USNATIVE_CK_TAPS_COUNT 1
#define SDRAM_PHY_USNATIVE_TAP_COUNT 41
#define SDRAM_PHY_USNATIVE_CONTROL_COUNT 4
#define SDRAM_PHY_USNATIVE_DATA_CONTROLS_COUNT 1
#define SDRAM_PHY_USNATIVE_DQ_TAPS {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}
#define SDRAM_PHY_USNATIVE_DQS_TAPS {32,33,34,35}
#define SDRAM_PHY_USNATIVE_DM_TAPS {40}
#define SDRAM_PHY_USNATIVE_CK_TAPS {36}
#define SDRAM_PHY_USNATIVE_DATA_CONTROLS {0}
#define CSR_NATIVE_DFI_ACTIVITY_REFRESH_COMMANDS_ADDR 1
#define CSR_DDRPHY_ABI_VERSION_ADDR 1
#define CSR_DDRPHY_ABI_CONFIG_ID_ADDR 1
#define CSR_DDRPHY_ABI_CAPABILITIES_ADDR 1
#define MAIN_RAM_BASE_VA 0x40000000u
#define MAIN_RAM_SIZE 0x02000000u
#define DFII_CONTROL_SEL 1u
#define DFII_CONTROL_CKE 2u
#define DFII_CONTROL_ODT 4u
#define DFII_CONTROL_RESET_N 8u

static unsigned control=DFII_CONTROL_SEL, vtc=1, selected, dly_sel;
static unsigned refresh_count, auto_refresh=1, sw_inc_count, max_chunk;
static unsigned inject_double, tx_inc_calls, vtc_shift_once, vtc_off_shift_once;
static unsigned late_rx_shift_once, vtc_rx_reads;
static unsigned rx[41], tx[41], current_chunk;
static unsigned ddrphy_abi_version_read(void) {return 0x00010000u;}
static unsigned ddrphy_abi_config_id_read(void) {return 0x1234u;}
static unsigned ddrphy_abi_capabilities_read(void) {return 1u;}
static unsigned ddrphy_riu_busy_read(void) {return 0;}
static void ddrphy_riu_nibble_write(unsigned x) {(void)x;}
static void ddrphy_riu_address_write(unsigned x) {(void)x;}
static void ddrphy_riu_wdata_write(unsigned x) {(void)x;}
static void ddrphy_riu_write_write(unsigned x) {(void)x;}
static void ddrphy_riu_read_write(unsigned x) {(void)x;}
static unsigned ddrphy_riu_error_read(void) {return 0;}
static unsigned ddrphy_riu_valid_read(void) {return 1;}
static unsigned ddrphy_riu_rdata_read(void) {return 0;}
static unsigned ddrphy_tap_select_read(void) {return selected;}
static void ddrphy_tap_select_write(unsigned x) {selected=x;}
static unsigned ddrphy_tap_status_valid_read(void) {return 1;}
static unsigned ddrphy_tap_allowed_read(void) {return !(control&DFII_CONTROL_SEL) && !vtc;}
static unsigned ddrphy_tap_rx_count_read(void) {
    if (vtc && late_rx_shift_once && ++vtc_rx_reads==2) {
        rx[selected]-=2;
        late_rx_shift_once=0;
    }
    return rx[selected];
}
static unsigned ddrphy_tap_tx_count_read(void) {return tx[selected];}
static void ddrphy_tap_rx_rst_write(unsigned x) {if(x) rx[selected]=0;}
static void ddrphy_tap_tx_rst_write(unsigned x) {if(x) tx[selected]=0;}
static void ddrphy_tap_rx_inc_write(unsigned x) {if(x) {++rx[selected];++current_chunk;if(current_chunk>max_chunk)max_chunk=current_chunk;}}
static void ddrphy_tap_tx_inc_write(unsigned x) {if(x) {++tx_inc_calls;++tx[selected];if(inject_double){++tx[selected];inject_double=0;}++current_chunk;if(current_chunk>max_chunk)max_chunk=current_chunk;}}
static unsigned ddrphy_en_vtc_read(void) {return vtc;}
static void ddrphy_en_vtc_write(unsigned x) {vtc=x;if(x) vtc_rx_reads=0;if(!x && vtc_off_shift_once){++rx[selected];vtc_off_shift_once=0;}if(x && vtc_shift_once){++rx[selected];vtc_shift_once=0;}}
static unsigned ddrphy_dly_sel_read(void) {return dly_sel;}
static void ddrphy_dly_sel_write(unsigned x) {dly_sel=x;}
static unsigned ddrphy_ready_read(void) {return 1;}
static unsigned ddrphy_vtc_rdy_read(void) {return 15;}
static unsigned sdram_dfii_control_read(void) {return control;}
static void sdram_dfii_control_write(unsigned x) {
    control=x;
    if (x & DFII_CONTROL_SEL) {current_chunk=0;if(auto_refresh) ++refresh_count;}
}
static unsigned native_dfi_activity_refresh_commands_read(void) {return refresh_count;}
static void sdram_invalidate_dma(void) {}
#include "native_dq_eye.h"
int main(void) {
    rx[0]=5; tx[0]=18;
    if (!ndq_eye_set_tap(0,21,0)) return 1;
    if (rx[0]!=21 || control!=DFII_CONTROL_SEL || vtc!=1) return 2;
    if (max_chunk>NDQ_EYE_TAP_CHUNK || refresh_count!=4) return 3;
    if (!ndq_eye_set_tap(0,3,1)) return 4;
    if (tx[0]!=3 || control!=DFII_CONTROL_SEL || vtc!=1) return 5;
    if (max_chunk>NDQ_EYE_TAP_CHUNK || refresh_count!=5) return 6;
    vtc_shift_once=1;
    if (!ndq_eye_set_tap(0,0,0) || rx[0]!=1 || control!=DFII_CONTROL_SEL || vtc!=1) return 7;
    vtc_off_shift_once=1;
    if (!ndq_eye_set_tap(0,4,0) || rx[0]!=4 || control!=DFII_CONTROL_SEL || vtc!=1) return 8;
    unsigned observed=0;
    late_rx_shift_once=1;
    if (!ndq_eye_set_tap_stable(0,8,0,&observed)) return 13;
    if (rx[0]!=8 || observed!=8 || late_rx_shift_once) return 14;
    unsigned calls=tx_inc_calls;
    inject_double=1;
    if (ndq_eye_set_tap(0,7,1)) return 9;
    if (tx[0]!=5 || tx_inc_calls!=calls+1 || control!=DFII_CONTROL_SEL || vtc!=1) return 10;
    auto_refresh=0;
    if (ndq_eye_set_tap(0,2,1)) return 11;
    if (control!=DFII_CONTROL_SEL || vtc!=1) return 12;
    auto_refresh=1;
    unsigned saved_rx[32], saved_tx[32];
    if (!ndq_eye_snapshot(saved_rx,saved_tx)) return 15;
    ++rx[0]; ++tx[0];
    if (!ndq_eye_restore(saved_rx,saved_tx,selected,dly_sel,vtc)) return 16;
    return 0;
}
''')
            binary = directory / ("eye_test.exe" if os.name == "nt" else "eye_test")
            result = subprocess.run(
                [CC, "-std=c99", "-Werror=implicit-function-declaration",
                 "-I", str(directory), "-I", str(INCLUDE), str(source), "-o", str(binary)],
                capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(result.returncode, 0,
                f"command={result.args!r} stdout={result.stdout!r} stderr={result.stderr!r}")
            result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("DQWIN_TAP_FAIL,dq=0,dir=tx,target=2,phase=refresh_timeout", result.stdout)
            self.assertIn("DQWIN_TAP_FAIL,dq=0,dir=tx,target=7,phase=increment_mismatch", result.stdout)
            self.assertIn("refresh_before=", result.stdout)

    def test_1to8_two_edge_burst_patterns_are_distinct(self):
        self.compile_run(r'''
#include <stdint.h>
#include "native_burst_pattern.h"
int main(void) {
    uint64_t previous=0;
    for (unsigned phase=0; phase<4; ++phase) {
        uint64_t value=usnative_burst_phase_pattern(phase, 0);
        if ((uint32_t)value == (uint32_t)(value >> 32)) return 1;
        if (phase && value == previous) return 2;
        if (value == usnative_burst_phase_pattern(phase, 1)) return 3;
        previous=value;
    }
    return 0;
}
''')

    def test_reduced_x64_1to8_patterns_fill_all_eight_64bit_phases(self):
        self.compile_run(r'''
#include <stdint.h>
#include <assert.h>
#include "native_burst_pattern.h"
int main(void) {
    uint64_t words[8];
    usnative_x64_1to8_patterns(words, 0);
    for (unsigned phase=0; phase<8; ++phase) {
        assert((uint32_t)words[phase] != 0);
        assert((uint32_t)(words[phase] >> 32) != 0);
        for (unsigned prior=0; prior<phase; ++prior) assert(words[phase] != words[prior]);
        assert(words[phase] != usnative_x64_phase_pattern(phase, 1));
    }
    return 0;
}
''')

    def test_native_dma_dq_mask_formatter_keeps_all_64_bits(self):
        source = (BIOS_INCLUDE / "native_dma.h").read_text()
        start = source.index("static void native_dma_print_dq_error_mask(")
        function = source[start:source.index("\n}", start) + 3]
        output = self.compile_run(r'''
#include <stdint.h>
#include <stdio.h>
''' + function + r'''
int main(void) {
    native_dma_print_dq_error_mask(0x8000000180000001ull);
    return 0;
}
''')
        self.assertIn("DMA first-error DQ mask=8000000180000001", output)

    def test_reduced_x64_rejects_unsupported_optional_dma_calibration(self):
        source = self.profile_reduced_x64_1to8(100000000, 11, 9, 0, 2)
        source = source.replace('#include <stdint.h>\n#include "profile.h"',
            "#define CONFIG_SDRAM_USNATIVE_DMA_CALIBRATION\n"
            "#define CONFIG_SDRAM_NATIVE_DMA_TEST\n"
            "#define CSR_DMA_BENCH_START_ADDR 1\n"
            "#define CSR_DMA_BENCH_DQ_ERROR_MASK_ADDR 1\n"
            "#define CSR_DMA_BENCH_DATA_WIDTH_ADDR 1\n"
            "#include <stdint.h>\n#include \"profile.h\"")
        self.compile_run(source, False)

    def test_high_rates_require_overclock_opt_in(self):
        for rate in (2933, 3200):
            with self.subTest(rate=rate):
                self.compile_run(self.profile(rate), False)
                self.compile_run(self.profile(rate, extra="#define CONFIG_SDRAM_USNATIVE_OVERCLOCK"))

    def test_fpga_clock_over_spec_marker_is_independent_of_edy_overclock_opt_in(self):
        marker = "#define CONFIG_SDRAM_USNATIVE_FPGA_NATIVE_CLOCK_OVER_SPEC"
        # 2400 MT/s remains on the nominal EDY4016A profile despite exceeding
        # the AES-KU40 -1 native PLL clock rating.
        self.compile_run(self.profile(2400, extra=marker))
        # The FPGA marker must not satisfy the separate >2400 memory overclock
        # guard in profile.h.
        self.compile_run(self.profile(2933, extra=marker), False)

    def test_3200_operating_phase_is_independent_of_debug(self):
        for debug in (False, True):
            with self.subTest(debug=debug):
                extra = "#define CONFIG_SDRAM_USNATIVE_OVERCLOCK\n"
                if debug:
                    extra += "#define CONFIG_SDRAM_USNATIVE_DEBUG\n"
                self.compile_run(self.profile(3200, extra=extra))

    def test_reject_wrong_phase(self):
        self.compile_run(self.profile(extra="#undef SDRAM_PHY_RDPHASE\n#define SDRAM_PHY_RDPHASE 1"), False)

    def test_reject_wrong_data_width(self):
        self.compile_run(self.profile(extra="#undef SDRAM_PHY_DATABITS\n#define SDRAM_PHY_DATABITS 32"), False)

    def test_dma_opt_in_requires_engine(self):
        self.compile_run(self.profile(extra="#define CONFIG_SDRAM_USNATIVE_DMA_CALIBRATION"), False)

    def test_dma_opt_in_profile_is_independent_of_debug(self):
        dma = r'''
#undef MAIN_RAM_SIZE
#define MAIN_RAM_SIZE 0x08000000
#define CONFIG_SDRAM_USNATIVE_DMA_CALIBRATION
#define CONFIG_SDRAM_NATIVE_DMA_TEST
#define CONFIG_SDRAM_NATIVE_DMA_BANK_GROUP_INTERLEAVING
#define CSR_DMA_BENCH_START_ADDR 1
#define CSR_DMA_BENCH_DQ_ERROR_MASK_ADDR 1
#define CSR_DMA_BENCH_DATA_WIDTH_ADDR 1
'''
        self.compile_run(self.profile(extra=dma,
            check='USNATIVE_DEBUG("debug disabled\\n");'))
        self.compile_run(self.profile(extra="#include <stdio.h>\n#define CONFIG_SDRAM_USNATIVE_DEBUG\n" + dma,
            check='USNATIVE_DEBUG("debug enabled\\n");'))
        self.compile_run(self.profile(extra=dma.replace(
            "#define CONFIG_SDRAM_NATIVE_DMA_BANK_GROUP_INTERLEAVING\n", "")))

    def test_dma_refinement_compiles_with_debug_on_and_off(self):
        source = r'''
#include <stdint.h>
#include <stdio.h>
#define __USNATIVE_DMA_IO_H
static unsigned nd_program(unsigned *centers, unsigned lanes, int offset)
    {(void)centers;(void)lanes;(void)offset;return 1;}
static unsigned nb_fail(unsigned error) {return error;}
static unsigned nd_dma_check(unsigned random, unsigned readonly)
    {(void)random;(void)readonly;return 0;}
static uint32_t dma_bench_dq_error_mask_read(void) {return 0;}
static void ddrphy_training_stage_write(unsigned stage) {(void)stage;}
#define USNATIVE_SNAPSHOT() do {} while (0)
#ifdef TEST_DEBUG
#define USNATIVE_DEBUG(...) printf(__VA_ARGS__)
#else
#define USNATIVE_DEBUG(...) do {} while (0)
#endif
#include "native_dma_calibration.h"
int main(void) {unsigned centers[16]={0};return nd_dma_refine(centers);}
'''
        flags = ("-Wall", "-Wformat=2", "-Werror=format",
                 "-Werror=unused-variable", "-Werror=unused-but-set-variable")
        self.compile_run(source, extra_flags=flags)
        self.compile_run("#define TEST_DEBUG\n#define CONFIG_SDRAM_USNATIVE_DEBUG\n" + source,
            extra_flags=flags)

    def test_dma_benchmark_reports_selected_hardware_mode(self):
        self.compile_run(r'''
#include <string.h>
#include "native_dma_mode.h"
int main(void) {
 return NATIVE_DMA_BANK_GROUP_INTERLEAVED != 0 ||
     strcmp(NATIVE_DMA_MODE_NAME, "standard-native-port");
}
''')

    def test_dma_completion_deadline_tracks_hardware_cycles(self):
        for frequency, expected_us in ((125_000_000, 17_000_000),
                                       (250_000_000, 9_000_000),
                                       (400_000_000, 6_000_000)):
            self.compile_run(f'''
#define CONFIG_CLOCK_FREQUENCY {frequency}u
#include "native_dma_timeout.h"
int main(void) {{ return native_dma_completion_timeout_us(0, 0) != {expected_us}u; }}
''')

    def test_dma_admission_for_native_and_component_phys(self):
        self.compile_run(r'''
#include <assert.h>
#define CONFIG_SDRAM_USNATIVE
static unsigned ready, stage, error, bist_only;
static unsigned ddrphy_ready_read(void) { return ready; }
static unsigned ddrphy_training_stage_read(void) { return stage; }
static unsigned ddrphy_training_error_read(void) { return error; }
static unsigned ddrphy_bisc_only_read(void) { return bist_only; }
#include "native_dma_admission.h"
int main(void) {
 assert(!native_dma_admission_ready());
 ready=1; stage=5; assert(native_dma_admission_ready());
 error=1; assert(!native_dma_admission_ready());
 error=0; bist_only=1; assert(!native_dma_admission_ready());
 return 0;
}
''')
        self.compile_run(r'''
#include <assert.h>
#define CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION
static unsigned admitted;
static unsigned dma_bench_software_ready_read(void) { return admitted; }
#include "native_dma_admission.h"
int main(void) {
 assert(!native_dma_admission_ready());
 admitted=1; assert(native_dma_admission_ready());
 return 0;
}
''')
        self.compile_run(r'''
#include "native_dma_admission.h"
int main(void) { return native_dma_admission_ready(); }
''', False)
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
#define SDRAM_PHY_USNATIVE_CONTROL_COUNT 8
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
#define CONFIG_SDRAM_USNATIVE
#define CSR_DDRCTRL_BASE 1
#define MAIN_RAM_BASE 0x40000000ul
#define MAIN_RAM_BASE_VA MAIN_RAM_BASE
#define MEMTEST_DATA_SIZE 64
#define false 0
static unsigned calibration_ok, memory_ok=1, handoffs, tests, speeds, failed, done, status_error;
static int usnative_mapping_verified=1;
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

    def test_component_phy_dma_admission_tracks_full_initialization(self):
        source = (ROOT / "litex/soc/software/liblitedram/sdram.c").read_text()
        start = source.index("int sdram_init(void) {")
        function = source[start:source.index("\n}\n", start) + 3]
        self.compile_run(r"""
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION
#define SDRAM_PHY_USPDDRPHY
#define CSR_DDRCTRL_BASE 1
#define CSR_DDRPHY_RST_ADDR 1
#define CSR_DDRPHY_WDLY_DQS_INC_COUNT_ADDR 1
#define SDRAM_PHY_WRITE_LEVELING_CAPABLE
#define SDRAM_PHY_MODULES 2
#define write_rst_dqs_delay 7
#define MAIN_RAM_BASE 0x40000000ul
#define MAIN_RAM_BASE_VA MAIN_RAM_BASE
#define MEMTEST_DATA_SIZE 64
#define false 0
static unsigned memory_ok=1, dqs_ok=1, leveling_ok=1, admission=9, admission_writes, tests, initialized, events;
static char order[64];
static void dma_bench_software_ready_write(unsigned value) {admission=value;++admission_writes;}
static void ddrctrl_init_done_write(unsigned value) {(void)value;}
static void ddrctrl_init_error_write(unsigned value) {(void)value;}
static void sdram_write_leveling_rst_cmd_delay(unsigned value) {(void)value;}
static void sdram_write_leveling_rst_dat_delay(unsigned module, unsigned show) {(void)module;(void)show;}
static void sdram_software_control_on(void) {order[events++]='O';}
static void sdram_software_control_off(void) {}
static int sdram_leveling(void) {return leveling_ok;}
static int selected;
static void sdram_select(int module, int dq) {assert(dq==0);selected=module;}
static void sdram_deselect(int module, int dq) {assert(module==selected && dq==0);}
static int write_rst_dqs_delay_checked(int module) {
 assert(module==selected); order[events++]=module?'1':'0'; return dqs_ok;
}
static void ddrphy_rst_write(unsigned value) {order[events++]=value?'R':'r';}
static void cdelay(unsigned value) {(void)value;order[events++]='D';}
static void init_sequence(void) {++initialized;order[events++]='J';}
static int memtest(unsigned *p, unsigned size) {(void)p;(void)size;++tests;return memory_ok;}
static void memspeed(unsigned *p,unsigned size,int write,int random) {(void)p;(void)size;(void)write;(void)random;}
""" + function + r"""
int main(void) {
 assert(sdram_init() && admission==1 && admission_writes==2 && tests==1 && initialized==1);
 assert(events>=8 && !memcmp(order, "O01RDrDJ", 8));
 memory_ok=0;
 assert(!sdram_init() && admission==0 && admission_writes==3 && tests==2 && initialized==2);
 memory_ok=1; dqs_ok=0;
 assert(!sdram_init() && admission==0 && admission_writes==4 && tests==2 && initialized==2);
 dqs_ok=1; leveling_ok=0;
 assert(!sdram_init() && admission==0 && admission_writes==5 && tests==2 && initialized==3);
 return 0;
}
""")

    def test_write_leveling_failure_stops_later_training(self):
        source = (ROOT / "litex/soc/software/liblitedram/sdram.c").read_text()
        start = source.index("int sdram_leveling(void) {")
        function = source[start:source.index("\n}\n", start) + 3]
        self.compile_run(r"""
#include <assert.h>
#include <stdio.h>
#define SDRAM_PHY_USPDDRPHY
#define SDRAM_PHY_MODULES 1
#define DQ_COUNT 1
#define SDRAM_PHY_WRITE_LEVELING_CAPABLE
#define SDRAM_PHY_WRITE_LATENCY_CALIBRATION_CAPABLE
#define SDRAM_PHY_WRITE_DQ_DQS_TRAINING_CAPABLE
#define SDRAM_PHY_READ_LEVELING_CAPABLE
static unsigned write_ok, write_calls, latency_calls, write_dq_calls, read_calls, released;
static void write_rst_delay(int module) {(void)module;}
static void read_rst_dq_delay(int module) {(void)module;}
static void sdram_leveling_action(int module, int dq, void (*action)(int)) {(void)dq;action(module);}
static void sdram_software_control_on(void) {}
static void sdram_software_control_off(void) {released++;}
static int sdram_write_leveling(void) {write_calls++;return write_ok;}
static void sdram_write_latency_calibration(void) {latency_calls++;}
static void sdram_write_dq_dqs_training(void) {write_dq_calls++;}
static void sdram_read_leveling(void) {read_calls++;}
""" + function + r"""
int main(void) {
 assert(!sdram_leveling());
 assert(write_calls==1 && !latency_calls && !write_dq_calls && !read_calls);
 assert(released==1);
 write_ok=1;
 assert(sdram_leveling());
 assert(write_calls==2 && latency_calls==1 && write_dq_calls==1 && read_calls==1);
 assert(released==2);
 return 0;
}
""")

    def test_component_phy_debug_maps_only_standard_training_details(self):
        source = (ROOT / "litex/soc/software/liblitedram/sdram.c").read_text()
        self.assertIn("#ifdef CONFIG_SDRAM_PHY_DEBUG", source)
        block = source.split("#ifdef CONFIG_SDRAM_PHY_DEBUG", 1)[1].split("#endif", 1)[0]
        self.assertIn("#define SDRAM_WRITE_LEVELING_CMD_DELAY_DEBUG", block)
        self.assertIn("#define SDRAM_WRITE_LATENCY_CALIBRATION_DEBUG", block)
        self.assertNotIn("USNATIVE", block)

    def test_component_phy_memory_test_cannot_grant_dma(self):
        source = (ROOT / "litex/soc/software/bios/cmds/cmd_litedram.c").read_text()
        start = source.index("static void sdram_test_handler(")
        function = source[start:source.index("\n}\n", start) + 3]
        self.compile_run(r"""
#include <assert.h>
#define CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION
#define MAIN_RAM_BASE_VA 0x40000000ul
#define MAIN_RAM_SIZE 0x08000000ul
static unsigned ready, memory_ok=1;
static unsigned dma_bench_software_ready_read(void) {return ready;}
static void dma_bench_software_ready_write(unsigned value) {ready=value;}
static int memtest(unsigned *p, unsigned long size) {(void)p;(void)size;return memory_ok;}
""" + function + r"""
int main(void) {
 sdram_test_handler(0, 0); assert(!ready);
 ready=1; sdram_test_handler(0, 0); assert(ready);
 memory_ok=0; sdram_test_handler(0, 0); assert(!ready);
 return 0;
}
""")

    def test_deskew_seeds_memory_with_debug_disabled(self):
        source = (INCLUDE / "native_bit_calibration.h").read_text()
        start = source.index("static unsigned nd_calibrate(")
        function = source[start:]
        self.compile_run(r"""
#include <assert.h>
#include <stdio.h>
#define SDRAM_PHY_DATABITS 16
#define USNATIVE_LANES 2
struct nd_score {
 unsigned errors[16],word_errors_by_phase[8];
 unsigned first_address[4],first_actual[4],first_expected[4],first_count;
};
static unsigned seeded, writes, stage, current_tap[2];
static unsigned scan_first[2]={999,999},scan_last[2];
static unsigned nd_write_probe(void) {return 0;}
#define USNATIVE_DEBUG(...) do {} while (0)
#define USNATIVE_SNAPSHOT() do {} while (0)
static unsigned nd_check(struct nd_score *score, unsigned readonly) {
 if(readonly) assert(seeded); else {seeded=1; ++writes;}
 score->first_count=0;
 for(unsigned phase=0;phase<8;++phase) score->word_errors_by_phase[phase]=0;
 unsigned errors=0;
 for(unsigned i=0;i<16;++i) {
  unsigned lane=i/8, tap=current_tap[lane];
  score->errors[i]=(lane==0 ? tap<30 || tap>50 : tap<80 || tap>100);
  errors+=score->errors[i];
 }
 return errors;
}
static void ddrphy_training_stage_write(unsigned value) {stage=value;}
static void sdram_software_control_off(void) {}
static int nc_set_rx(unsigned lane, unsigned tap) {
 if(tap<scan_first[lane]) scan_first[lane]=tap;
 if(tap>scan_last[lane]) scan_last[lane]=tap;
 current_tap[lane]=tap;
 return 1;
}
static unsigned nb_fail(unsigned error) {return error;}
static int nd_program(unsigned *centers,unsigned lanes,int offset) {
 for(unsigned lane=0;lane<2;++lane)
  if(lanes & (1u<<lane)) current_tap[lane]=(unsigned)((int)centers[lane*8]+offset);
 return 1;
}
""" + function + r"""
int main(void) {
 unsigned centers[16];
 unsigned coarse[2]={40,80};
 assert(nd_calibrate(3,centers,coarse)==0);
 assert(writes==5 && stage==5);
 assert(scan_first[0]==12 && scan_last[0]==68);
 assert(scan_first[1]==52 && scan_last[1]==108);
 assert(centers[0]==40 && centers[8]==90);
 return 0;
}
""")

    def test_per_bit_deskew_scans_every_descriptor_lane(self):
        source = (INCLUDE / "native_bit_calibration.h").read_text()
        start = source.index("static unsigned nd_calibrate(")
        function = source[start:]
        self.compile_run(r"""
#include <assert.h>
#include <stdio.h>
#define SDRAM_PHY_DATABITS 32
#define USNATIVE_LANES 4
struct nd_score {
 unsigned errors[SDRAM_PHY_DATABITS],word_errors_by_phase[8];
 unsigned first_address[4],first_actual[4],first_expected[4],first_count;
};
static unsigned stage, current_tap[USNATIVE_LANES];
static unsigned scan_first[USNATIVE_LANES]={999,999,999,999};
static unsigned scan_last[USNATIVE_LANES];
static unsigned nd_write_probe(void) {return 0;}
#define USNATIVE_DEBUG(...) do {} while (0)
#define USNATIVE_SNAPSHOT() do {} while (0)
static unsigned nd_check(struct nd_score *score, unsigned readonly) {
 unsigned errors=0;
 score->first_count=0;
 for(unsigned phase=0;phase<8;++phase) score->word_errors_by_phase[phase]=0;
 for(unsigned bit=0;bit<SDRAM_PHY_DATABITS;++bit) {
  unsigned lane=bit/8, tap=current_tap[lane];
  score->errors[bit]=readonly && (tap<40+40*lane-10 || tap>40+40*lane+10);
  errors+=score->errors[bit];
 }
 return errors;
}
static void ddrphy_training_stage_write(unsigned value) {stage=value;}
static void sdram_software_control_off(void) {}
static int nc_set_rx(unsigned lane, unsigned tap) {
 if(tap<scan_first[lane]) scan_first[lane]=tap;
 if(tap>scan_last[lane]) scan_last[lane]=tap;
 current_tap[lane]=tap;
 return 1;
}
static unsigned nb_fail(unsigned error) {return error;}
static int nd_program(const unsigned *centers,unsigned lanes,int offset) {
 for(unsigned lane=0;lane<USNATIVE_LANES;++lane)
  if(lanes & (1u<<lane)) current_tap[lane]=(unsigned)((int)centers[lane*8]+offset);
 return 1;
}
""" + function + r"""
int main(void) {
 unsigned centers[SDRAM_PHY_DATABITS];
 unsigned coarse[USNATIVE_LANES]={40,80,120,160};
 assert(nd_calibrate(15,centers,coarse)==0);
 assert(stage==5);
 for(unsigned lane=0;lane<USNATIVE_LANES;++lane) {
  unsigned first=coarse[lane]>28 ? coarse[lane]-28 : 0;
  assert(scan_first[lane]==first);
  assert(scan_last[lane]==first+56);
  for(unsigned bit=lane*8;bit<lane*8+8;++bit) assert(centers[bit]==coarse[lane]);
 }
 return 0;
}
""")

    def test_eye_mask_maps_one_logical_dq_to_cpu_word_lanes(self):
        source = (INCLUDE / "native_dq_eye.h").read_text()
        start = source.index("static unsigned ndq_eye_dq_word_mask(")
        function = source[start:source.index("\n}\n", start) + 3]
        data_map_source = (INCLUDE / "native_data_map.h").read_text()
        data_map_start = data_map_source.index("static inline uint32_t usnative_cpu_word_dq_mask(")
        data_map = data_map_source[data_map_start:data_map_source.index("\n}", data_map_start) + 3]
        for width, low, high in ((16, 1, 1 << 16), (32, 1, 0)):
            with self.subTest(width=width):
                self.compile_run(f"""
#include <assert.h>
#include <stdint.h>
#define SDRAM_PHY_DATABITS {width}
""" + data_map + function + f"""
int main(void) {{
 assert(ndq_eye_dq_word_mask(0,0)=={low | high}u);
 return 0;
}}
""")
        self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#define SDRAM_PHY_DATABITS 64
''' + data_map + function + r'''
int main(void) {
 assert(ndq_eye_dq_word_mask(0,0)==1u);
 assert(ndq_eye_dq_word_mask(32,0)==0u);
 assert(ndq_eye_dq_word_mask(0,1)==0u);
 assert(ndq_eye_dq_word_mask(32,1)==1u);
 return 0;
}
''')

    def test_eye_baseline_guard_rejects_errors_on_unselected_dqs(self):
        source = (INCLUDE / "native_dq_eye.h").read_text()
        start = source.index("static int ndq_eye_guard_pass(")
        function = source[start:source.index("\n}\n", start) + 3]
        self.compile_run(r'''
#include <assert.h>
''' + function + r'''
int main(void) {
    assert(ndq_eye_guard_pass(0, 0));
    assert(!ndq_eye_guard_pass(1, 0));
    assert(!ndq_eye_guard_pass(0, 1));
    assert(!ndq_eye_guard_pass(1, 1));
    return 0;
}
''')

    def test_eye_patterns_cover_complementary_walking_and_aggressor_data(self):
        source = (INCLUDE / "native_dq_eye.h").read_text()
        mask_start = source.index("static unsigned ndq_eye_dq_word_mask(")
        mask = source[mask_start:source.index("\n}\n", mask_start) + 3]
        pattern_start = source.index("enum ndq_eye_pattern {")
        pattern_end = source.index("\n};", pattern_start) + 3
        enum = source[pattern_start:pattern_end]
        name_start = source.index("static const char *ndq_eye_pattern_name(")
        name = source[name_start:source.index("\n}\n", name_start) + 3]
        word_start = source.index("static unsigned ndq_eye_pattern_word(")
        word = source[word_start:source.index("\n}\n", word_start) + 3]
        data_map_source = (INCLUDE / "native_data_map.h").read_text()
        data_map_start = data_map_source.index("static inline uint32_t usnative_cpu_word_dq_mask(")
        data_map = data_map_source[data_map_start:data_map_source.index("\n}", data_map_start) + 3]
        self.compile_run(r'''
#include <assert.h>
#include <string.h>
#define SDRAM_PHY_DATABITS 32
#include <stdint.h>
''' + data_map + r'''
static unsigned lfsr(unsigned width, unsigned state) {
    assert(width == 32);
    return (state << 1) | ((state >> 31) ^ (state >> 21) ^ (state >> 1) ^ state) & 1u;
}
''' + enum + mask + name + word + r'''
int main(void) {
    unsigned state = 0x12345678u;
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_COMPLEMENTARY, 3, 0, 0, &state) == 0x55555555u);
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_COMPLEMENTARY, 3, 1, 0, &state) == 0xaaaaaaaau);
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_COMPLEMENTARY, 3, 0, 1, &state) == 0xaaaaaaaau);
    for (unsigned bit=0; bit<32; ++bit) {
        assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_WALKING_BIT, 3, bit*2, 0, &state) == 0);
        assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_WALKING_BIT, 3, bit*2+1, 0, &state) == 1u << bit);
    }
    unsigned target = 1u << 3;
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_AGGRESSOR, 3, 0, 0, &state) == 0);
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_AGGRESSOR, 3, 1, 0, &state) == target);
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_AGGRESSOR, 3, 2, 0, &state) == ~target);
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_AGGRESSOR, 3, 3, 0, &state) == 0xffffffffu);
    unsigned expected = lfsr(32, 0x12345678u);
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_PRBS32_SEEDED, 3, 0, 0, &state) == expected);
    assert(!strcmp(ndq_eye_pattern_name(NDQ_EYE_PATTERN_AGGRESSOR), "aggressor"));
    return 0;
}
''')

    def test_x64_eye_patterns_attribute_dq0_31_32_63_to_cpu_word_address(self):
        data_map_source = (INCLUDE / "native_data_map.h").read_text()
        data_map_start = data_map_source.index("static inline uint32_t usnative_cpu_word_dq_mask(")
        data_map = data_map_source[data_map_start:data_map_source.index("\n}", data_map_start) + 3]
        source = (INCLUDE / "native_dq_eye.h").read_text()
        mask_start = source.index("static unsigned ndq_eye_dq_word_mask(")
        mask = source[mask_start:source.index("\n}", mask_start) + 3]
        pattern_start = source.index("enum ndq_eye_pattern {")
        pattern_end = source.index("\n};", pattern_start) + 3
        enum = source[pattern_start:pattern_end]
        word_start = source.index("static unsigned ndq_eye_pattern_word(")
        word = source[word_start:source.index("\n}", word_start) + 3]
        self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#define SDRAM_PHY_DATABITS 64
static unsigned lfsr(unsigned width, unsigned state) {(void)width; return state;}
''' + data_map + enum + mask + word + r'''
int main(void) {
    assert(ndq_eye_dq_word_mask(0, 0)==1u);
    assert(ndq_eye_dq_word_mask(31, 0)==0x80000000u);
    assert(ndq_eye_dq_word_mask(32, 0)==0u);
    assert(ndq_eye_dq_word_mask(63, 0)==0u);
    assert(ndq_eye_dq_word_mask(0, 1)==0u);
    assert(ndq_eye_dq_word_mask(31, 1)==0u);
    assert(ndq_eye_dq_word_mask(32, 1)==1u);
    assert(ndq_eye_dq_word_mask(63, 1)==0x80000000u);
    unsigned state=1;
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_AGGRESSOR,32,1,0,&state)==1u);
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_AGGRESSOR,32,3,0,&state)==0xffffffffu);
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_AGGRESSOR,0,2,0,&state)==0xfffffffeu);
    assert(ndq_eye_pattern_word(NDQ_EYE_PATTERN_AGGRESSOR,0,0,0,&state)==0u);
    return 0;
}
''')

    def test_ddr_rate_does_not_overflow_signed_int(self):
        source = (ROOT / "litex/soc/software/liblitedram/sdram.c").read_text()
        start = source.index("unsigned int sdram_get_freq(void) {")
        function = source[start:source.index("\n}\n", start) + 3]
        for clock in (300000000, 333333333, 366666666, 400000000):
            self.compile_run("#define SDRAM_PHY_XDR 2\n#define SDRAM_PHY_PHASES 4\n"
                + f"#define CONFIG_CLOCK_FREQUENCY {clock}\n" + function
                + f"int main(void) {{return sdram_get_freq() != {clock * 8}u;}}")
