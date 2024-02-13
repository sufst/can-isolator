#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

int main(void) {
    rcc_clock_setup_in_hse_25mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOB);
    
}