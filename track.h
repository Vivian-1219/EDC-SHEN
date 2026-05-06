#ifndef TRACK_H
#define TRACK_H

#include <stdint.h>

typedef struct {
    uint8_t raw;
    uint8_t black_mask;
    uint8_t black_count;
    int error;
} track_sample_t;

void track_init(void);
track_sample_t track_read(void);
int track_get_error(void);
int track_is_lost(void);
int track_is_cross(void);

#endif
