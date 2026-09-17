#
# This file is part of LiteX.
# SPDX-License-Identifier: BSD-2-Clause

"""Check diagnostic tap changes preserve ownership and fail closed."""
import unittest
from test.software import test_usnative as firmware

@unittest.skipUnless(firmware.CC, "Host C compiler required")
class TestRXDiagnostic(unittest.TestCase):
    compile_run=firmware.TestUSNativeFirmware.compile_run
    def test_success_refusal_and_rollback(self):
        self.compile_run(r'''
#include <assert.h>
#include <stdio.h>
#define USNATIVE_RX_DIAGNOSTIC_CYCLES() 0u
#define DFII_CONTROL_HARDWARE 1
#define DFII_CONTROL_CKE 2
#define DFII_CONTROL_ODT 4
#define DFII_CONTROL_RESET_N 8
static const unsigned nd_sites[16]={4,2,3,11,5,10,8,9,23,22,16,15,18,21,17,24};
static unsigned scenario,admitted,owner,vtc,tap,selected,resets,busy_reads,waits;
static unsigned dma_bench_busy_read(void) {++busy_reads;return scenario==2 || (scenario==5 && busy_reads==2);}
static unsigned dma_bench_fault_read(void) {return 0;}
static unsigned dma_bench_software_ready_read(void) {return admitted;}
static void dma_bench_software_ready_write(unsigned v) {admitted=v;}
static unsigned ddrphy_ready_read(void) {return 1;}
static unsigned ddrphy_dly_rdy_read(void) {return 255;}
static unsigned ddrphy_vtc_rdy_read(void) {return 255;}
static unsigned ddrphy_en_vtc_read(void) {return vtc;}
static unsigned ddrphy_training_stage_read(void) {return 5;}
static unsigned ddrphy_training_error_read(void) {return 0;}
static unsigned ddrphy_bisc_only_read(void) {return 0;}
static unsigned sdram_dfii_control_read(void) {return owner;}
static void sdram_dfii_control_write(unsigned v) {assert(!admitted);owner=v;}
static void ddrphy_en_vtc_write(unsigned v) {vtc=v;}
static unsigned ddrphy_tap_select_read(void) {return selected;}
static void ddrphy_tap_select_write(unsigned v) {selected=v;}
static unsigned ddrphy_tap_allowed_read(void) {return owner==14 && !vtc && selected==15;}
static unsigned ddrphy_tap_rx_count_read(void) {return tap;}
static void ddrphy_tap_rx_rst_write(unsigned v) {assert(v && owner==14 && !vtc);tap=0;++resets;}
static void ddrphy_tap_rx_inc_write(unsigned v) {assert(v && owner==14 && !vtc);if(scenario!=3 || resets!=1) ++tap;}
static int native_tap_wait(void) {return 1;}
static void cdelay(unsigned v) {(void)v;}
static void flush_cpu_dcache(void) {assert(owner==1);}
static void flush_l2_cache(void) {assert(owner==1);}
static void sdram_software_control_off(void) {owner=vtc=1;}
static int nc_wait_vtc(void) {assert(owner==1 && vtc);return !(scenario==4 && ++waits==1);}
#include "native_rx_diagnostic.h"
int main(void) {
 for(scenario=0;scenario<6;++scenario) {
  admitted=owner=vtc=1;tap=40;selected=4;resets=busy_reads=waits=0;
  int ok=sdram_usnative_rx_delay(scenario==1?16:11,36);
  assert(owner==1 && vtc==1 && selected==4);
  if(scenario==0) assert(ok && admitted && tap==36 && resets==1);
  else if(scenario==1 || scenario==2) assert(!ok && admitted && tap==40 && !resets);
  else if(scenario==5) assert(!ok && !admitted && tap==40 && !resets);
  else assert(!ok && !admitted && tap==40 && resets==2);
 }
 return 0;
}
''')
