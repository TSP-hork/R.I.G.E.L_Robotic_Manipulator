#include "tim.h"

// ============================================================================
// tim6_init: configure TIM6 as a free-running microsecond timer.
//
// TIM6 is a basic 16-bit timer with no output channels — ideal for simple
// time-base / delay functions.
//
// Clock derivation:
//   APB1 prescaler = /2 → APB1 timer clock = 2 × 140 MHz = 280 MHz
//   TIM6 PSC = 279 → counter ticks at 280 MHz / 280 = 1 MHz (1 µs per tick)
//   TIM6 ARR = 0xFFFF → free-running 16-bit counter, wraps every ~65.5 ms
// ============================================================================
void tim6_init(void) {
    // Enable TIM6 peripheral clock on APB1
    RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;
    for (volatile int i = 0; i < 100; i++);  // Short delay for clock stabilization

    // Prescaler: divide 280 MHz timer clock by 280 → 1 tick = 1 µs
    TIM6->PSC = 280 - 1;
    // Auto-reload: maximum 16-bit value → counter counts 0…65535 then wraps
    TIM6->ARR = 0xFFFF;
    // Generate an update event to load PSC and ARR into the shadow registers immediately
    TIM6->EGR = TIM_EGR_UG;
    // Start the counter
    TIM6->CR1 |= TIM_CR1_CEN;
}

// ============================================================================
// delay_us: blocking delay for the specified number of microseconds.
// Uses TIM6's 1 µs tick. Maximum single delay = 65535 µs (~65 ms) due to
// the 16-bit counter width.
// ============================================================================
void delay_us(uint32_t us) {
    TIM6->CNT = 0;             // Reset counter to zero
    while (TIM6->CNT < us);   // Spin until the required number of ticks elapsed
}

// ============================================================================
// delay_ms: blocking delay for the specified number of milliseconds.
// Implemented as repeated 1000 µs delays to avoid 16-bit counter overflow.
// ============================================================================
void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000);        // 1 ms = 1000 µs
    }
}

// ============================================================================
// micros: return the current value of the free-running microsecond counter.
// Useful for non-blocking timing measurements. Note: wraps every ~65.5 ms.
// ============================================================================
uint32_t micros(void) {
    return TIM6->CNT;
}