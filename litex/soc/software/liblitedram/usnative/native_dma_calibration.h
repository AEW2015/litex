/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Counter/PRBS DMA refinement and measured per-bit guard margins.
 * Experimental, destructive calibration; see doc/usnative_bios.md.
 */
#include "native_dma_io.h"

static unsigned nd_dma_refine(unsigned *centers)
{
    /* Score physical bits independently over the complete DMA transfer.
     * Counter and PRBS eyes must overlap; aggregate errors are diagnostic
     * only and never used to score a different DQ. */
    unsigned clean[29],trial[16],window_first[16],window_last[16];
    for(unsigned sample=0;sample<29;++sample) clean[sample]=0xffff;
    for(unsigned pattern=0;pattern<2;++pattern) {
        if(!nd_program(centers,3,0)) return nb_fail(12);
        unsigned seed=nd_dma_check(pattern,0);
        USNATIVE_DEBUG("DMA deskew seed pattern=%u errors=%u mask=%04x\n",
            pattern, seed, (unsigned)dma_bench_dq_error_mask_read());
        if(seed==0xffffffffu) return nb_fail(16);
        for(unsigned sample=0;sample<29;++sample) {
            unsigned tap=12+2*sample;
            for(unsigned bit=0;bit<16;++bit) trial[bit]=tap;
            if(!nd_program(trial,3,0)) return nb_fail(12);
            unsigned errors=nd_dma_check(pattern,1);
            unsigned mask=dma_bench_dq_error_mask_read();
            USNATIVE_DEBUG("DMA_DESKEW_MASK pattern=%u tap=%u errors=%u mask=%04x\n",pattern,tap,errors,mask);
            if(errors==0xffffffffu || (!!errors != !!mask)) return nb_fail(16);
            clean[sample] &= ~mask;
        }
    }
    for(unsigned bit=0;bit<16;++bit) {
        unsigned run=0,best=0,end=0;
        for(unsigned sample=0;sample<29;++sample) {
            if(clean[sample] & (1u<<bit)) ++run;else run=0;
            if(run>best) {best=run;end=12+2*sample;}
        }
        if(best<5) {USNATIVE_DEBUG("DMA_DESKEW_NO_WINDOW bit=%u span=%u\n",bit,best?2*(best-1):0);return nb_fail(17);}
        unsigned first=end-2*(best-1);
        centers[bit]=first+2*((best-1)/2);
        window_first[bit]=first;window_last[bit]=end;
        USNATIVE_DEBUG("DMA_DESKEW_INITIAL %u WINDOW %u %u CENTER %u\n",bit,first,end,centers[bit]);
    }
    /* Refine a guard edge without reducing its required margin. Every
     * adjustment stays inside the measured counter/PRBS window, and all
     * six fresh guard transfers restart after any adjustment. */
    unsigned accepted=0;
#ifdef CONFIG_SDRAM_USNATIVE_DEBUG
    unsigned guard_errors[6],guard_masks[6],accepted_attempt=0;
#endif
    for(unsigned attempt=0;attempt<8 && !accepted;++attempt) {
        unsigned retry=0;
        for(int offset=-4;offset<=4 && !retry;offset+=4) {
            if(!nd_program(centers,3,offset)) return nb_fail(12);
            for(unsigned random=0;random<2;++random) {
                unsigned errors=nd_dma_check(random,0);
                unsigned mask=dma_bench_dq_error_mask_read();
#ifdef CONFIG_SDRAM_USNATIVE_DEBUG
                unsigned slot=(unsigned)(offset+4)/4*2+random;
                guard_errors[slot]=errors;guard_masks[slot]=mask;
#endif
                USNATIVE_DEBUG("DMA_GUARD_SEARCH attempt=%u offset=%d pattern=%u errors=%u mask=%04x\n",attempt,offset,random,errors,mask);
                if(errors==0xffffffffu || (!!errors != !!mask)) return nb_fail(16);
                if(errors) {
                    if(offset==0) return nb_fail(18);
                    for(unsigned bit=0;bit<16;++bit) if(mask & (1u<<bit)) {
                        int next=(int)centers[bit]+(offset<0?2:-2);
                        if(next-4<(int)window_first[bit] || next+4>(int)window_last[bit]) {
                            USNATIVE_DEBUG("DMA_GUARD_WINDOW_EXHAUSTED bit=%u center=%u\n",bit,centers[bit]);
                            return nb_fail(18);
                        }
                        USNATIVE_DEBUG("DMA_GUARD_ADJUST bit=%u old=%u new=%d\n",bit,centers[bit],next);
                        centers[bit]=(unsigned)next;
                    }
                    retry=1;break;
                }
            }
        }
        if(!retry) {
            accepted=1;
#ifdef CONFIG_SDRAM_USNATIVE_DEBUG
            accepted_attempt=attempt;
#endif
        }
    }
    if(!accepted) return nb_fail(18);
#ifdef CONFIG_SDRAM_USNATIVE_DEBUG
    USNATIVE_DEBUG("DMA_GUARD_ACCEPT attempt=%u\n",accepted_attempt);
    for(unsigned bit=0;bit<16;++bit)
        USNATIVE_DEBUG("DMA_DESKEW_BIT %u WINDOW %u %u CENTER %u\n",bit,window_first[bit],window_last[bit],centers[bit]);
    for(int offset=-4;offset<=4;offset+=4) for(unsigned random=0;random<2;++random) {
        unsigned slot=(unsigned)(offset+4)/4*2+random;
        USNATIVE_DEBUG("DMA_DESKEW_GUARD offset=%d pattern=%u errors=%u mask=%04x\n",offset,random,guard_errors[slot],guard_masks[slot]);
    }
#endif
    if(!nd_program(centers,3,0)) return nb_fail(12);
    for(unsigned random=0;random<2;++random) {
        unsigned errors=nd_dma_check(random,0);
        USNATIVE_DEBUG("DMA_DESKEW_FINAL pattern=%u errors=%u\n",random,errors);
        if(errors) return nb_fail(19);
    }
    ddrphy_training_stage_write(5);USNATIVE_SNAPSHOT();
    return 0;
}
