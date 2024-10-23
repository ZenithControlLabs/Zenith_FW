#ifndef ZENITH_RUNNING_AVG_H
#define ZENITH_RUNNING_AVG_H

#include <stdint.h>

#define RA_MAX_BUF_SIZE 64

typedef struct {
    uint16_t buf[RA_MAX_BUF_SIZE];
    int ind;
    uint32_t running_sum_large;
    uint32_t running_sum_small;
    uint32_t large_size;
    uint32_t small_size;
} running_avg_t;

void init_running_avg(running_avg_t *r, int large_size_bits,
                      int small_size_bits);

void update_running_avg(running_avg_t *r, uint16_t in);

#endif // ZENITH_RUNNING_AVG_H