#include "zenith/utilities/running_avg.h"

void init_running_avg(running_avg_t *r, int large_size_bits,
                      int small_size_bits) {
    *r = (running_avg_t){
        .buf = {0},
        .ind = 0,
        .running_sum_large = 0,
        .running_sum_small = 0,
        .large_size = (1 << large_size_bits),
        .small_size = (1 << small_size_bits),
    };
}

void update_running_avg(running_avg_t *r, uint16_t in) {

    int ind_minus_small = (r->ind >= r->small_size)
                              ? (r->ind - r->small_size)
                              : ((r->ind - r->small_size) + r->large_size);
    int ind_minus_large = (r->ind >= r->large_size)
                              ? (r->ind - r->large_size)
                              : ((r->ind - r->large_size) + r->large_size);
    r->running_sum_large -= r->buf[ind_minus_large];
    r->running_sum_small -= r->buf[ind_minus_small];

    r->buf[r->ind] = in;
    r->running_sum_large += in;
    r->running_sum_small += in;
    r->ind = (r->ind >= r->large_size - 1) ? 0 : r->ind + 1;
}