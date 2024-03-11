#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/can.h>

#define UART_PORT       GPIOA
#define UART_CK         GPIO8
#define UART_TX         GPIO9
#define UART_RX         GPIO10

// LV is CAN1
#define LV_CAN_PORT     GPIOB
#define LV_CAN_RX       GPIO8
#define LV_CAN_TX       GPIO9

// HV is CAN2
#define HV_CAN_PORT     GPIOB
#define HV_CAN_RX       GPIO12
#define HV_CAN_TX       GPIO13

// sizeof(can_frame_t) = 16 bytes
// so max queue size is 1 kB

#define CAN_FRAME_SIZE              sizeof(can_frame_t)
#define CAN_MAX_MESSAGE_SIZE_BYTES  8
#define CAN_MAX_QUEUE_LENGTH        64

typedef struct {
    uint32_t identifier;
    bool extended;
    uint8_t length;
    uint8_t data[CAN_MAX_MESSAGE_SIZE_BYTES];
} can_frame_t;

ssize_t _write(int file, const char *ptr, ssize_t len) {
    // we can't write to anything other than stdout or stderr
    if (file != STDOUT_FILENO && file != STDERR_FILENO) {
        errno = EIO;
        return -1;
    }

    int i;
    for (i = 0; i < len; i++) {
        // funny serial terminal behaviour fix
        if (ptr[i] == '\n') usart_send_blocking(USART1, "\r");

        usart_send_blocking(USART1, ptr[i]);
    }

    return i;
}

static void usart_setup(void) {
    rcc_periph_clock_enable(RCC_USART1);
    gpio_set_mode(UART_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, UART_CK | UART_TX | UART_RX);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);

    // output buffering is pointless on embedded devices and can cause bug-like behaviour
    setbuf(stdout, NULL);
}

static void usart_puts(char *string) {
    while (*string) {
        usart_send_blocking(USART1, *string);
        string++;
    }
}

static void usart_putln(char *string) {
    usart_puts(string);
    usart_puts("\r\n");
}

static void lv_can_setup(void) { // CAN1
    rcc_periph_clock_enable(RCC_CAN1);
    can_reset(CAN1);

    can_init(CAN1, false, true, true, false, false, false, CAN_BTR_SJW_1TQ, CAN_BTR_TS1_11TQ, CAN_BTR_TS2_4TQ, 6, false, false);
}

static void hv_can_setup(void) { // CAN2
    rcc_periph_clock_enable(RCC_CAN2);
    can_reset(CAN2);

    can_init(CAN2, false, true, true, false, false, false, CAN_BTR_SJW_1TQ, CAN_BTR_TS1_11TQ, CAN_BTR_TS2_4TQ, 6, false, false);
}

int main(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE25_72MHZ]);
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_primary_remap(false, AFIO_MAPR_CAN1_REMAP_PORTB);

    for (;;) {
        gpio_toggle(GPIOB, GPIO8);
    }

    return 0;
}