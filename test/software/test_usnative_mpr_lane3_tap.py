#
# This file is part of LiteX.
#
# SPDX-License-Identifier: BSD-2-Clause

"""Host C regressions for the opt-in lane-3 MPR RX-tap fallback helpers."""

import unittest

from test.software import test_usnative as firmware


@unittest.skipUnless(firmware.CC, "Host C compiler required")
class TestUSNativeMPRLane3Tap(unittest.TestCase):
    compile_run = firmware.TestUSNativeFirmware.compile_run

    def test_bounded_tap_choice_epoch_health_joint_verdict_and_per_dq_restore(self):
        self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
#include "native_mpr_lane3_tap.h"

static unsigned actual[32];
static int fail_dq=-1, corrupt_dq=-1;
static unsigned writes, reads;

static int restore_one(unsigned dq,unsigned tap,void *context) {
    (void)context;
    ++writes;
    if ((int)dq==fail_dq) return 0;
    actual[dq]=tap;
    return 1;
}
static int read_one(unsigned dq,unsigned *tap,void *context) {
    (void)context;
    ++reads;
    if ((int)dq==fail_dq) return 0;
    *tap=actual[dq];
    if ((int)dq==corrupt_dq) ++*tap;
    return 1;
}

int main(void) {
    unsigned taps[8]={3,7,11,15,19,23,27,31};
    unsigned passing[USNATIVE_MPR_LANE3_TAP_POINTS]={0};
    unsigned chosen=0,first=0,last=0,count=0;

    assert(usnative_mpr_lane3_tap_candidate(1,12,12,0x8));
    assert(!usnative_mpr_lane3_tap_candidate(0,12,12,0x8)); /* failed/empty epoch */
    assert(!usnative_mpr_lane3_tap_candidate(1,11,12,0x8));
    assert(!usnative_mpr_lane3_tap_candidate(1,12,11,0x8)); /* bad edge/pop count */
    assert(!usnative_mpr_lane3_tap_candidate(1,12,12,0)); /* fresh, incoherent DQ rotations */
    assert(usnative_mpr_lane3_joint_confirmed(1,12,12,12,0xf));
    assert(!usnative_mpr_lane3_joint_confirmed(1,12,12,12,0x7)); /* odd lane incoherent */
    assert(!usnative_mpr_lane3_joint_confirmed(1,12,11,12,0xf));
    assert(!usnative_mpr_lane3_joint_confirmed(1,11,12,12,0xf));
    assert(!usnative_mpr_lane3_joint_confirmed(1,12,12,11,0xf));

    /* The wider of two clean runs wins; an isolated passing point cannot
     * displace it or masquerade as a robust candidate. */
    passing[2]=passing[3]=passing[4]=1;
    passing[8]=passing[9]=1;
    assert(usnative_mpr_lane3_choose_tap(passing,32,&chosen,&first,&last,&count));
    assert(first==16 && last==32 && count==3 && chosen==24);
    for(unsigned i=0;i<USNATIVE_MPR_LANE3_TAP_POINTS;++i) passing[i]=0;
    assert(!usnative_mpr_lane3_choose_tap(passing,48,&chosen,&first,&last,&count));

    for(unsigned dq=0;dq<32;++dq) actual[dq]=100+dq;
    writes=reads=0;
    assert(usnative_mpr_restore_lane_taps(3,taps,8,restore_one,read_one,0));
    assert(writes==8 && reads==8);
    for(unsigned bit=0;bit<8;++bit) assert(actual[24+bit]==taps[bit]);

    /* A failed setter or mismatching readback must fail restoration. The
     * remaining independent DQs should still be restored best-effort. */
    writes=reads=0;fail_dq=26;
    assert(!usnative_mpr_restore_lane_taps(3,taps,8,restore_one,read_one,0));
    assert(writes==8 && reads==7);
    for(unsigned bit=0;bit<8;++bit)
        if (bit!=2) assert(actual[24+bit]==taps[bit]);
    fail_dq=-1;corrupt_dq=27;writes=reads=0;
    assert(!usnative_mpr_restore_lane_taps(3,taps,8,restore_one,read_one,0));
    assert(writes==8 && reads==8);
    assert(!usnative_mpr_restore_lane_taps(4,taps,8,restore_one,read_one,0));
    return 0;
}
''')


if __name__ == "__main__":
    unittest.main()
