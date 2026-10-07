#include "stm32h7xx.h"

// ============================================================================
// gpio_init: configure the three on-board diagnostic LEDs as push-pull outputs.
//
// Pin assignments (accent active-low on some boards, directly driven here):
//   PB0  — Green LED
//   PE1  — Yellow LED
//   PB14 — Red LED
//
// These LEDs are toggled/set inside the SPI EXTI ISR to provide real-time
// visual feedback on link status, FOC phase, and ADC health.
// ============================================================================
void gpio_init(void) {
    // Enable GPIO port clocks for ports A, B, and E
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOEEN;

    // PB0 (Green LED): set to general-purpose output mode
    GPIOB->MODER &= ~(3U << (0 * 2));   // Clear mode bits for PB0
    GPIOB->MODER |=  (1U << (0 * 2));   // Set mode = 01 (output)

    // PE1 (Yellow LED): set to general-purpose output mode
    GPIOE->MODER &= ~(3U << (1 * 2));   // Clear mode bits for PE1
    GPIOE->MODER |=  (1U << (1 * 2));   // Set mode = 01 (output)

    // PB14 (Red LED): set to general-purpose output mode
    // (Actively used by the ISR for FOC phase indication — must be configured)
    GPIOB->MODER &= ~(3U << (14 * 2));  // Clear mode bits for PB14
    GPIOB->MODER |=  (1U << (14 * 2));  // Set mode = 01 (output)
}