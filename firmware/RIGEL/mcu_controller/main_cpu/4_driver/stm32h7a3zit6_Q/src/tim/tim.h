#ifndef TIM_H
#define TIM_H

#include "stm32h7xx.h"

// Initialize TIM6 as a free-running 1 µs resolution timer (wraps at ~65.5 ms).
// Must be called after rcc_init() so the APB1 timer clock is valid.
void tim6_init(void);

// Blocking microsecond delay (max ~65535 µs per call due to 16-bit counter).
void delay_us(uint32_t us);

// Blocking millisecond delay (built from repeated 1 ms delays).
void delay_ms(uint32_t ms);

// Return the current microsecond counter value (free-running, wraps at 65535).
uint32_t micros(void);

#endif