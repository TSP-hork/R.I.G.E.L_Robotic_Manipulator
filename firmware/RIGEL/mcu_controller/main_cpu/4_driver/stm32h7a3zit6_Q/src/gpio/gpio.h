#ifndef GPIO_H
#define GPIO_H

#include "stm32h7xx.h"

// Initialize diagnostic LED pins (PB0 = Green, PE1 = Yellow, PB14 = Red)
// as push-pull outputs. Must be called once during system startup.
void gpio_init(void);

// Optional convenience macros for LED control (accent active-high here):
// #define LED_GREEN_ON()   (GPIOB->BSRR = (1u << 0))
// #define LED_GREEN_OFF()  (GPIOB->BSRR = (1u << 16))
// #define LED_YELLOW_ON()  (GPIOE->BSRR = (1u << 1))
// #define LED_YELLOW_OFF() (GPIOE->BSRR = (1u << 17))
// #define LED_RED_ON()     (GPIOB->BSRR = (1u << 14))
// #define LED_RED_OFF()    (GPIOB->BSRR = (1u << 30))

#endif // GPIO_H