#include "stm32h7xx.h"

// Forward declarations for each peripheral init function
void rcc_init(void);        // Clock tree: 280 MHz from HSE+PLL (rcc.c)
void gpio_init(void);       // Diagnostic LED pins (gpio.c)
void tim6_init(void);       // Microsecond delay timer (tim.c)
void spi_slave_init(void);  // SPI3 slave with DMA + EXTI (spi.c)

// ============================================================================
// hw_binding_init: one-call hardware initialization entry point.
//
// Calls all peripheral init functions in the correct dependency order:
//   1. rcc_init    — must be first (all peripherals need clocks)
//   2. gpio_init   — configure LED pins (needs GPIO clocks from rcc)
//   3. tim6_init   — microsecond timer (needs APB1 clock from rcc)
//   4. spi_slave_init — SPI3 + DMA + EXTI (needs all clocks + SYSCFG)
// ============================================================================
void hw_binding_init(void) {
    rcc_init();
    gpio_init();
    tim6_init();
    spi_slave_init();
}