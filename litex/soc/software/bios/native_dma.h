/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* BIOS command: native-port DMA test; execution and stack remain in ROM/SRAM.
 * CPU caches must be flushed before DMA, and CPU must not access the test
 * region until completion. Rates use hardware cycles, not UART/CPU timing.
 */
#include <system.h>
#include <libbase/timeout.h>
#include <generated/soc.h>
#include <generated/sdram_phy.h>
#include "native_dma_admission.h"
#include "native_dma_mode.h"
#include "native_dma_timeout.h"
static void native_dma_quarantine(void)
{
#ifdef CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION
    /* Any failed or unterminated DMA invalidates later DMA admission until a
     * complete sdram_init and memory test explicitly re-admits traffic. */
    dma_bench_software_ready_write(0);
#endif
}

static void native_dma_print_dq_error_mask(uint64_t mask)
{
    printf("DMA first-error DQ mask=%08x%08x\n",
        (unsigned)(mask >> 32), (unsigned)mask);
}

static void native_dma_rate(const char *direction,unsigned beats,unsigned cycles)
{
    unsigned bytes_per_beat=dma_bench_data_width_read()/8;
    uint64_t bytes=(uint64_t)beats*bytes_per_beat;
    if(!cycles) {printf("DMA %s: no transfers\n",direction);return;}
    uint64_t bps=bytes*CONFIG_CLOCK_FREQUENCY/cycles;
    uint64_t mib100=bps*100/1048576;
    unsigned peak_bytes_per_cycle=SDRAM_PHY_DATABITS*SDRAM_PHY_XDR*SDRAM_PHY_PHASES/8;
    unsigned utilization=(unsigned)(bytes*10000/((uint64_t)cycles*peak_bytes_per_cycle));
    printf("DMA %s: beats=%u cycles=%u, %u.%03u MB/s, %u.%02u MiB/s, %u.%02u%% DDR theoretical peak\n",
        direction,beats,cycles,(unsigned)(bps/1000000),(unsigned)((bps%1000000)/1000),
        (unsigned)(mib100/100),(unsigned)(mib100%100),utilization/100,utilization%100);
}
static void native_dma_handler(int nb_params,char **params)
{
    unsigned long address=MAIN_RAM_BASE+0x01000000,length=0x04000000,random=1,read_only=0,skip_beats=0;
    unsigned long repeat_verify=0,repeat_idle_cycles=0;
    if((nb_params>0 && !parse_ulong(params[0],&address)) ||
       (nb_params>1 && !parse_ulong(params[1],&length)) ||
       (nb_params>2 && !parse_ulong(params[2],&random)) ||
       (nb_params>3 && !parse_ulong(params[3],&read_only)) ||
       (nb_params>4 && !parse_ulong(params[4],&skip_beats)) ||
       (nb_params>5 && !parse_ulong(params[5],&repeat_verify)) ||
       (nb_params>6 && !parse_ulong(params[6],&repeat_idle_cycles))) {
        printf("DMA: invalid parameter\n");return;
    }
    if(address<MAIN_RAM_BASE || !length ||
        ((uint64_t)address+length)>((uint64_t)MAIN_RAM_BASE+MAIN_RAM_SIZE) ||
        ((address|length)&(dma_bench_data_width_read()/8-1)) || random>1 || read_only>1 ||
        skip_beats>0xffffffffUL || repeat_verify>1 ||
        repeat_idle_cycles>0xffffffffUL || (!repeat_verify && repeat_idle_cycles)) {
        printf("DMA: require aligned nonempty region within DDR and boolean flags\n");return;
    }
    if(dma_bench_busy_read() || dma_bench_fault_read()>=3) {
        native_dma_quarantine();
        printf("DMA unavailable: busy or fatal fault; reconfigure FPGA after fatal fault\n");return;
    }
    if(!native_dma_admission_ready()) {
        printf("DMA refused: SDRAM calibration and memory test have not passed\n");return;
    }
#ifndef CSR_DMA_BENCH_REPEAT_VERIFY_ADDR
    if(repeat_verify) {
        printf("DMA repeat verification is not included in this build\n");return;
    }
#endif
    printf("Native DMA: mode=%s addr=%08lx bytes=%lu pattern=%s read_only=%lu skip_beats=%lu repeat_verify=%lu repeat_idle_cycles=%lu width=%u fifo=%u clock=%u Hz\n",
        NATIVE_DMA_MODE_NAME,address,length,random ? "PRBS31" : "counter",read_only,skip_beats,
        repeat_verify,repeat_idle_cycles,
        (unsigned)dma_bench_data_width_read(),
        (unsigned)dma_bench_fifo_depth_read(), CONFIG_CLOCK_FREQUENCY);
    /* Prevent deferred CPU writes from modifying DMA data. */
    flush_cpu_dcache();flush_l2_cache();
#ifdef CONFIG_SDRAM_NATIVE_DMA_DIAGNOSTICS
    dma_diag_clear_write(1);
#endif
    dma_bench_base_write(address-MAIN_RAM_BASE);dma_bench_length_write(length);
    dma_bench_random_write(random);dma_bench_read_only_write(read_only);
    dma_bench_skip_beats_write((unsigned)skip_beats);
#ifdef CSR_DMA_BENCH_REPEAT_VERIFY_ADDR
    dma_bench_repeat_verify_write((unsigned)repeat_verify);
    dma_bench_repeat_idle_cycles_write((unsigned)repeat_idle_cycles);
#endif
    dma_bench_timeout_write(NATIVE_DMA_HW_TIMEOUT_CYCLES);dma_bench_start_write(1);
    /* The DMA engine owns the primary cycle timeout. This outer deadline
     * catches missing completion reporting without depending on CPU speed. */
    struct timeout completion_timeout;
    timeout_start(&completion_timeout,
        native_dma_completion_timeout_us((unsigned)repeat_verify,
            (unsigned)repeat_idle_cycles));
    while(!dma_bench_done_read() && !timeout_expired(&completion_timeout)) {}
    if(!dma_bench_done_read()) {
        native_dma_quarantine();
        printf("DMA completion timeout: reconfigure FPGA before further DDR use\n");
        return;
    }
    unsigned fault=dma_bench_fault_read(),errors=dma_bench_errors_read();
    unsigned writes=dma_bench_write_beats_read(),reads=dma_bench_read_beats_read();
    native_dma_rate("WRITE",writes,dma_bench_write_cycles_read());
    native_dma_rate("READ",reads,dma_bench_read_cycles_read());
    printf("DMA counters: write_cmd_stalls=%u read_cmd_stalls=%u errors=%u fault=%u first_error_offset=%08x\n",
        (unsigned)dma_bench_write_stalls_read(),
        (unsigned)dma_bench_read_stalls_read(), errors, fault,
        (unsigned)dma_bench_first_error_offset_read());
#ifdef CSR_DMA_BENCH_REPEAT_VERIFY_ADDR
    unsigned repeat_errors=repeat_verify ? dma_bench_repeat_errors_read() : 0;
    unsigned repeat_reads=repeat_verify ? dma_bench_repeat_read_beats_read() : 0;
    if(repeat_verify) {
        native_dma_rate("REPEAT_READ",repeat_reads,dma_bench_repeat_read_cycles_read());
        printf("DMA repeat counters: read_cmd_stalls=%u errors=%u first_error_offset=%08x\n",
            (unsigned)dma_bench_repeat_read_stalls_read(),repeat_errors,
            (unsigned)dma_bench_repeat_first_error_offset_read());
    }
#else
    unsigned repeat_errors=0,repeat_reads=0;
#endif
#ifdef CSR_DMA_BENCH_DQ_ERROR_MASK_ADDR
    if (errors) {
        uint64_t dq_error_mask = dma_bench_dq_error_mask_read();
        native_dma_print_dq_error_mask(dq_error_mask);
#if defined(CSR_DMA_BENCH_FIRST_ERROR_ACTUAL_ADDR) && \
    defined(CSR_DMA_BENCH_FIRST_ERROR_EXPECTED_ADDR)
        for (unsigned word=0; word<CSR_DMA_BENCH_FIRST_ERROR_XOR_SIZE; ++word) {
            unsigned xor_word = csr_read_simple(CSR_DMA_BENCH_FIRST_ERROR_XOR_ADDR + 4*word);
            if (xor_word) {
                printf("DMA first-error word=%u xor=%08x actual=%08x expected=%08x\n",
                    word,xor_word,
                    (unsigned)csr_read_simple(CSR_DMA_BENCH_FIRST_ERROR_ACTUAL_ADDR + 4*word),
                    (unsigned)csr_read_simple(CSR_DMA_BENCH_FIRST_ERROR_EXPECTED_ADDR + 4*word));
            }
        }
#endif
    }
#endif
#if defined(CSR_DMA_BENCH_REPEAT_FIRST_ERROR_XOR_ADDR) && \
    defined(CSR_DMA_BENCH_REPEAT_FIRST_ERROR_ACTUAL_ADDR) && \
    defined(CSR_DMA_BENCH_REPEAT_FIRST_ERROR_EXPECTED_ADDR)
    if (repeat_errors) {
        for (unsigned word=0; word<CSR_DMA_BENCH_REPEAT_FIRST_ERROR_XOR_SIZE; ++word) {
            unsigned xor_word = csr_read_simple(CSR_DMA_BENCH_REPEAT_FIRST_ERROR_XOR_ADDR + 4*word);
            if (xor_word) {
                printf("DMA repeat first-error word=%u xor=%08x actual=%08x expected=%08x\n",
                    word,xor_word,
                    (unsigned)csr_read_simple(CSR_DMA_BENCH_REPEAT_FIRST_ERROR_ACTUAL_ADDR + 4*word),
                    (unsigned)csr_read_simple(CSR_DMA_BENCH_REPEAT_FIRST_ERROR_EXPECTED_ADDR + 4*word));
            }
        }
    }
#endif
#ifdef CONFIG_SDRAM_NATIVE_DMA_DIAGNOSTICS
    printf("DMA diagnostic: cycles=%u write_cmd_stalls=%u bank_cmd_pending=%u cas=%u writer_fifo_full=%u read_admission_blocked=%u\n",
        (unsigned)dma_diag_cycles_read(),
        (unsigned)dma_diag_command_stall_cycles_read(),
        (unsigned)dma_diag_bank_command_pending_cycles_read(),
        (unsigned)dma_diag_controller_cas_commands_read(),
        (unsigned)dma_diag_writer_fifo_full_cycles_read(),
        (unsigned)dma_diag_reader_reservation_full_cycles_read());
#endif
    unsigned expected=length/(dma_bench_data_width_read()/8);
    int pass=!fault && !errors && reads==expected && writes==(read_only?0:expected) &&
        (!repeat_verify || (!repeat_errors && repeat_reads==expected));
    printf("DMA_RESULT %s mode=%s addr=%08lx bytes=%lu pattern=%lu read_only=%lu repeat_verify=%lu\n",
        pass?"PASS":"FAIL",NATIVE_DMA_MODE_NAME,address,length,random,read_only,repeat_verify);
    if(!pass) native_dma_quarantine();
    if(fault>=3) printf("DMA fatal fault: reconfigure FPGA before further DDR use\n");
}
define_command_args(native_dma,native_dma_handler,"Native-width DDR DMA throughput and integrity test (destructive)",
    "native_dma [address] [bytes] [PRBS31=1] [read_only=0] [skip_beats=0] [repeat_verify=0] [repeat_idle_cycles=0]",0,7,MEM_CMDS);
