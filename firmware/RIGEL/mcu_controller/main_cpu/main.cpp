#include "stm32h7xx.h"
#include "4_driver/stm32h7a3zit6_Q/hw_impl.h"

// These functions are implemented in C source files, so they need
// extern "C" linkage when called from a C++ translation unit.
extern "C" void hw_binding_init(void);  // Initializes all MCU peripherals (rcc, gpio, tim6, spi)
extern "C" void foc_init(void);         // Initializes FOC PI controllers and current references

int main(void) {
    // Set the vector table address to the start of Flash.
    // Required when using a bootloader or when the default VTOR doesn't
    // point to the application's vector table.
    SCB->VTOR = 0x08000000;

    // Initialize all hardware peripherals in the correct dependency order:
    //   rcc_init()       → clock tree (280 MHz)
    //   gpio_init()      → diagnostic LED pins
    //   tim6_init()      → microsecond delay timer
    //   spi_slave_init() → SPI3 + DMA + EXTI4 interrupt
    hw_binding_init();

    // Initialize the FOC current-loop PI controllers (zero integrators, set gains)
    foc_init();

    // Enable global interrupts — the EXTI4 ISR will now fire on every
    // SPI CS edge from the FPGA (~20 kHz), running the entire FOC pipeline.
    __enable_irq();

    // Main loop: all real-time work happens inside EXTI4_IRQHandler.
    // The CPU sleeps between interrupts to save power; WFI (Wait For Interrupt)
    // halts the core until the next interrupt arrives.
    while (1) {
        __WFI();  // Sleep until the next EXTI4 (or any other) interrupt
    }
}