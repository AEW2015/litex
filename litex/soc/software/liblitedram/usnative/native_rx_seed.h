/*
 * This file is part of LiteX.
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef USNATIVE_RX_SEED_H
#define USNATIVE_RX_SEED_H

/* Keep the boot/coarse seed when every observation at every candidate failed.
 * Otherwise use the scan's clean-window center or its least-bad tap. */
static inline unsigned usnative_coarse_seed_select(unsigned previous, unsigned candidate,
    unsigned minimum_errors, unsigned sample_count)
{
    return minimum_errors >= sample_count ? previous : candidate;
}

/* Return bounded common RX seeds for recovery after a coarse scan has no clean
 * point. First use the median of lane estimates, then the coarse scan midpoint.
 * If estimates already equal the midpoint, move one coarse step inward. */
static inline unsigned usnative_rx_seed_fallback(const unsigned *seeds, unsigned count,
    unsigned attempt, unsigned first, unsigned last, unsigned step)
{
    unsigned sorted[8];
    if (count == 0 || count > 8 || first > last || step == 0) return first;
    for (unsigned i=0; i<count; ++i) {
        unsigned value=seeds[i];
        unsigned j=i;
        while (j && sorted[j-1] > value) {
            sorted[j]=sorted[j-1];
            --j;
        }
        sorted[j]=value;
    }
    unsigned middle=count/2;
    unsigned median=(count & 1) ? sorted[middle] :
        sorted[middle-1] + (sorted[middle]-sorted[middle-1])/2;
    unsigned center=first+(last-first)/2;
    if (attempt == 0) return median < first ? first : median > last ? last : median;
    if (center != median) return center;
    if (center + step <= last) return center + step;
    return center >= first + step ? center - step : center;
}

/* Keep a measured TX seed when a retry scan finds no clean point. */
static inline unsigned usnative_tx_seed_select(unsigned previous, unsigned candidate,
    unsigned clean_samples)
{
    return clean_samples ? candidate : previous;
}

/* Select the least-bad point in a bounded joint RX/TX sweep. Ties prefer the
 * point closest to the starting pair, then lower RX and TX taps for stable
 * firmware traces. */
struct usnative_joint_seed {
    unsigned rx, tx, errors, valid;
};

static inline void usnative_joint_seed_consider(struct usnative_joint_seed *best,
    unsigned rx, unsigned tx, unsigned errors, unsigned previous_rx,
    unsigned previous_tx)
{
    unsigned distance=(rx > previous_rx ? rx-previous_rx : previous_rx-rx) +
        (tx > previous_tx ? tx-previous_tx : previous_tx-tx);
    unsigned best_distance=best->valid ?
        (best->rx > previous_rx ? best->rx-previous_rx : previous_rx-best->rx) +
        (best->tx > previous_tx ? best->tx-previous_tx : previous_tx-best->tx) : 0;
    if (!best->valid || errors < best->errors ||
        (errors == best->errors && (distance < best_distance ||
         (distance == best_distance && (rx < best->rx ||
          (rx == best->rx && tx < best->tx)))))) {
        best->rx=rx; best->tx=tx; best->errors=errors; best->valid=1;
    }
}

#endif /* USNATIVE_RX_SEED_H */
