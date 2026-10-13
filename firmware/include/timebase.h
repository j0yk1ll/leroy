#pragma once

#include <stdint.h>

void clock_init(void);
void timebase_init(void);
void fast_tick_init(void);
uint32_t millis(void);
uint32_t micros(void);
