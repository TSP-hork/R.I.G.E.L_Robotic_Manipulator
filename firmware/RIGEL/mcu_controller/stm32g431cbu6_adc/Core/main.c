#include "stm32g4xx.h"
#include "rcc.h"
#include "adc.h"
#include "spi.h"

int main(void) {

    //Init

    RCC_Init(); // 170 MHz
    ADC_Init(); // 42 MHz
    SPI_Init(); // Slave

    // ADC and SPI (u never know what)

    ADC_Start();
    SPI_Start();

    while (1) {
        __asm("wfi"); 
    }
}
