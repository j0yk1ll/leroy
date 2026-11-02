#pragma once

#include <stdint.h>

typedef struct {
    int32_t front_right;
    int32_t rear_right;
    int32_t rear_left;
    int32_t front_left;
} encoder_counts_t;

void encoders_init(void);
void encoders_get_counts(encoder_counts_t *counts);
void encoders_reset(void);
