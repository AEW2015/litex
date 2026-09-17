/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Diagnostic branch only. Change one RX DQ on the CPU so JTAG/console latency
 * cannot hold DFI ownership and suppress controller refresh for seconds. */
#ifndef USNATIVE_RX_DIAGNOSTIC_CYCLES
static unsigned native_rx_diagnostic_cycles(void)
{
    unsigned value;
    __asm__ volatile ("csrr %0, mcycle" : "=r"(value) : : "memory");
    return value;
}
#define USNATIVE_RX_DIAGNOSTIC_CYCLES() native_rx_diagnostic_cycles()
#endif

static int native_rx_diagnostic_set(unsigned tap)
{
    if(!ddrphy_tap_allowed_read()) return 0;
    ddrphy_tap_rx_rst_write(1);cdelay(100);
    for(unsigned i=0;i<tap;++i) {ddrphy_tap_rx_inc_write(1);cdelay(100);}
    return native_tap_wait() && ddrphy_tap_rx_count_read()==tap;
}

int sdram_usnative_rx_delay(unsigned bit,unsigned tap)
{
    if(bit>=16 || tap>511 || dma_bench_busy_read() || dma_bench_fault_read() ||
       !dma_bench_software_ready_read() || !ddrphy_ready_read() ||
       ddrphy_dly_rdy_read()!=255 || ddrphy_vtc_rdy_read()!=255 ||
       !ddrphy_en_vtc_read() || ddrphy_training_stage_read()!=5 ||
       ddrphy_training_error_read() || ddrphy_bisc_only_read() ||
       sdram_dfii_control_read()!=DFII_CONTROL_HARDWARE) {
        printf("RX_TAP REFUSED: requires trained idle DDR/DMA and valid bit/tap\n");
        return 0;
    }
    unsigned previous_select=ddrphy_tap_select_read();
    ddrphy_tap_select_write(nd_sites[bit]);
    if(!native_tap_wait()) {ddrphy_tap_select_write(previous_select);return 0;}
    unsigned original=ddrphy_tap_rx_count_read();
    if(original>511) {ddrphy_tap_select_write(previous_select);return 0;}
    flush_cpu_dcache();flush_l2_cache();
    dma_bench_software_ready_write(0);
    if(dma_bench_busy_read()) {ddrphy_tap_select_write(previous_select);return 0;}
    /* All console output and readiness polling occur after ownership returns.
     * The paused-refresh interval contains only bounded local CSR operations. */
    unsigned paused_start=USNATIVE_RX_DIAGNOSTIC_CYCLES();
    sdram_dfii_control_write(DFII_CONTROL_CKE|DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
    ddrphy_en_vtc_write(0);cdelay(1000);
    int ok=native_rx_diagnostic_set(tap);
    unsigned before_vtc=ddrphy_tap_rx_count_read();
    int rollback=1;
    if(!ok) rollback=native_rx_diagnostic_set(original);
    sdram_dfii_control_write(DFII_CONTROL_HARDWARE);
    unsigned paused_cycles=USNATIVE_RX_DIAGNOSTIC_CYCLES()-paused_start;
    ddrphy_en_vtc_write(1);
    int ready=nc_wait_vtc();
    unsigned after_vtc=ddrphy_tap_rx_count_read();
    if(ok && !ready) {
        sdram_dfii_control_write(DFII_CONTROL_CKE|DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
        ddrphy_en_vtc_write(0);cdelay(1000);
        rollback=native_rx_diagnostic_set(original);
        sdram_dfii_control_write(DFII_CONTROL_HARDWARE);
        ddrphy_en_vtc_write(1);
        (void)nc_wait_vtc();
    }
    ddrphy_tap_select_write(previous_select);
    if(ok && ready) dma_bench_software_ready_write(1);
    printf("RX_TAP %s bit=%u original=%u requested=%u before_vtc=%u after_vtc=%u rollback=%u ready=%u paused_cycles=%u\n",
        ok && ready?"PASS":"FAIL",bit,original,tap,before_vtc,after_vtc,rollback,ready,paused_cycles);
    return ok && ready;
}
