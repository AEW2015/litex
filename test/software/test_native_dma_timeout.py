# SPDX-License-Identifier: BSD-2-Clause
"""Bounded C tests for the BIOS DMA deadline and diagnostic quiescence."""
import os
import shlex
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BIOS = ROOT / "litex/soc/software/bios"
EYE = ROOT / "litex/soc/software/liblitedram/usnative/native_dq_eye.h"
CC = shutil.which(os.environ.get("CC", "gcc"))


def compiler_path(path):
    """Pass MSYS paths in native /c/... form when using its MinGW GCC."""
    value = str(path)
    if os.name == "nt" and CC and "msys" in CC.lower():
        drive, tail = os.path.splitdrive(value)
        if drive:
            return "/" + drive[0].lower() + tail.replace("\\", "/")
    return value


@unittest.skipUnless(CC, "Host C compiler required")
class NativeDMATimeoutTest(unittest.TestCase):
    def compile_run(self, code):
        with tempfile.TemporaryDirectory(prefix="native_dma_timeout_") as tmp:
            source = Path(tmp) / "test.c"
            binary = Path(tmp) / ("test.exe" if os.name == "nt" else "test")
            source.write_text(code, encoding="utf-8")
            command = [CC, "-std=c99", "-Wall", "-Werror", "-Werror=implicit-function-declaration",
                      "-I", compiler_path(BIOS), compiler_path(source), "-o", compiler_path(binary)]
            if os.name == "nt" and CC and "msys" in CC.lower():
                bash = str(Path(CC).parents[2] / "usr/bin/bash.exe")
                shell_command = "PATH=/ucrt64/bin:/usr/bin:$PATH " + " ".join(
                    shlex.quote(arg if arg != CC else "gcc") for arg in command)
                result = subprocess.run([bash, "-lc", shell_command], capture_output=True,
                                        text=True, timeout=60)
            else:
                result = subprocess.run(command, capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0,
                             f"compiler returned {result.returncode}: {result.stdout!r} {result.stderr!r}; cmd={command!r}")
            result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_timeout_budget_preserves_normal_and_covers_repeat_read_and_idle(self):
        cases = (
            (125_000_000, "native_dma_completion_timeout_us(0, 0) != 17000000u"),
            (250_000_000, "native_dma_completion_timeout_us(0, 0) != 9000000u"),
            (400_000_000, "native_dma_completion_timeout_us(0, 0) != 6000000u"),
            (150_000_000, "native_dma_completion_timeout_us(1, 0) != 21000000u"),
            (150_000_000, "native_dma_completion_timeout_us(1, 150000000u) != 22000000u"),
            (150_000_000, "native_dma_completion_timeout_us(0, 150000000u) != 14333334u"),
        )
        for frequency, expression in cases:
            with self.subTest(frequency=frequency, expression=expression):
                self.compile_run(f"""
#include <stdint.h>
#define CONFIG_CLOCK_FREQUENCY {frequency}u
#include "native_dma_timeout.h"
int main(void) {{ return {expression}; }}
""")

    def test_timeout_budget_saturates_instead_of_wrapping(self):
        for frequency in (0, 1):
            with self.subTest(frequency=frequency):
                self.compile_run(f"""
#include <stdint.h>
#include <limits.h>
#define CONFIG_CLOCK_FREQUENCY {frequency}u
#include "native_dma_timeout.h"
int main(void) {{ return native_dma_completion_timeout_us(1, UINT32_MAX) != UINT_MAX; }}
""")

    def test_dma_fault_quarantine_revokes_software_admission(self):
        text = (BIOS / "native_dma.h").read_text(encoding="utf-8")
        start = text.index("static void native_dma_quarantine(void)")
        helper = text[start:text.index("\n}\n", start) + 3]
        self.compile_run("""
#include <assert.h>
#define CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION
static unsigned admission = 1;
static unsigned writes;
static void dma_bench_software_ready_write(unsigned value) { admission=value; ++writes; }
""" + helper + """
int main(void) {
    native_dma_quarantine();
    assert(admission == 0 && writes == 1);
    admission = 1;
    native_dma_quarantine();
    assert(admission == 0 && writes == 2);
    return 0;
}
""")

    def test_eye_fifo_guard_checks_mapped_dq_and_allows_nonempty_sideband(self):
        text = EYE.read_text(encoding="utf-8")
        start = text.index("#define NDQ_EYE_FIFO_WORDS")
        helper_start = text.index("static int ndq_eye_wait_dq_fifo_empty(", start)
        helper_end = text.index("\n}\n", helper_start) + 3
        define = text[start:text.index("\n", start) + 1] + "#define NDQ_EYE_FIFO_UNAVAILABLE (-1)\n"
        helper = text[helper_start:helper_end]
        common = """
#include <assert.h>
#include <stdio.h>
#define SDRAM_PHY_DATABITS 3u
#define SDRAM_PHY_USNATIVE_DQ_TAPS_COUNT 3u
#define SDRAM_PHY_USNATIVE_TAP_COUNT 65u
#define CSR_DDRPHY_FIFO_EMPTY_ADDR 0x1000u
#define CSR_DDRPHY_FIFO_EMPTY_SIZE 3u
static unsigned usnative_dq_taps[3] = {2u, 33u, 64u};
static unsigned csr_read_simple(unsigned address) { return state[(address-0x1000u)/4u]; }
static void cdelay(unsigned value) { (void)value; }
"""
        for big_endian, initial_state, fail_index, fail_mask, expected in (
            (False, "{0xfffffff7u, 0xffffffffu, 1u}", 0, 4, (0xfffffff7, 0xffffffff, 1)),
            (True,  "{1u, 0xffffffffu, 0xffffffffu}", 1, 2, (1, 0xffffffff, 0xffffffff)),
        ):
            with self.subTest(big_endian=big_endian):
                endian_define = "#define CONFIG_CSR_ORDERING_BIG\n" if big_endian else ""
                fail_word = fail_index
                self.compile_run(common.replace("static unsigned csr_read_simple", "static unsigned state[3] = "+initial_state+";\nstatic unsigned csr_read_simple") +
                    endian_define + define + helper + f"""
int main(void) {{
    unsigned words[NDQ_EYE_FIFO_WORDS];
    assert(ndq_eye_wait_dq_fifo_empty(words));
    assert(words[0] == {expected[0]}u && words[1] == {expected[1]}u && words[2] == {expected[2]}u);
    state[{fail_word}] &= ~{fail_mask}u;
    assert(!ndq_eye_wait_dq_fifo_empty(words));
    state[{fail_word}] |= {fail_mask}u;
    usnative_dq_taps[0] = SDRAM_PHY_USNATIVE_TAP_COUNT;
    assert(ndq_eye_wait_dq_fifo_empty(words) == NDQ_EYE_FIFO_UNAVAILABLE);
    return 0;
}}
""")

    def test_eye_fifo_guard_requires_status_csr(self):
        text = EYE.read_text(encoding="utf-8")
        start = text.index("#define NDQ_EYE_FIFO_WORDS")
        helper_start = text.index("static int ndq_eye_wait_dq_fifo_empty(", start)
        helper_end = text.index("\n}\n", helper_start) + 3
        define = text[start:text.index("\n", start) + 1] + "#define NDQ_EYE_FIFO_UNAVAILABLE (-1)\n"
        helper = text[helper_start:helper_end]
        self.compile_run("""
#include <assert.h>
#define SDRAM_PHY_DATABITS 1u
#define SDRAM_PHY_USNATIVE_DQ_TAPS_COUNT 1u
#define SDRAM_PHY_USNATIVE_TAP_COUNT 1u
""" + define + helper + """
int main(void) {
    unsigned words[NDQ_EYE_FIFO_WORDS];
    assert(ndq_eye_wait_dq_fifo_empty(words) == NDQ_EYE_FIFO_UNAVAILABLE);
    assert(words[0] == 0u);
    return 0;
}
""")

    def test_eye_fifo_guard_requires_three_consecutive_dq_observations(self):
        text = EYE.read_text(encoding="utf-8")
        start = text.index("#define NDQ_EYE_FIFO_WORDS")
        helper_start = text.index("static int ndq_eye_wait_dq_fifo_empty(", start)
        helper_end = text.index("\n}\n", helper_start) + 3
        define = text[start:text.index("\n", start) + 1] + "#define NDQ_EYE_FIFO_UNAVAILABLE (-1)\n"
        helper = text[helper_start:helper_end]
        self.compile_run("""
#include <assert.h>
#define SDRAM_PHY_DATABITS 1u
#define SDRAM_PHY_USNATIVE_DQ_TAPS_COUNT 1u
#define SDRAM_PHY_USNATIVE_TAP_COUNT 1u
#define CSR_DDRPHY_FIFO_EMPTY_ADDR 0x1000u
#define CSR_DDRPHY_FIFO_EMPTY_SIZE 1u
static unsigned usnative_dq_taps[1] = {0u};
static const unsigned observations[] = {1u, 0u, 1u, 1u, 1u};
static unsigned reads;
static unsigned csr_read_simple(unsigned address) {
    assert(address == CSR_DDRPHY_FIFO_EMPTY_ADDR);
    return reads < sizeof(observations)/sizeof(observations[0]) ? observations[reads++] : 1u;
}
static void cdelay(unsigned value) { (void)value; }
""" + define + helper + """
int main(void) {
    unsigned words[NDQ_EYE_FIFO_WORDS];
    assert(ndq_eye_wait_dq_fifo_empty(words));
    /* The second sample was nonempty, so three fresh stable samples are needed. */
    assert(reads == 5u);
    return 0;
}
""")


if __name__ == "__main__":
    unittest.main()
