#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

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

// wait for interrupt
#define WFI()   __asm__ volatile("wfi");

/**
 * @brief A CAN message.
 */
typedef struct {
    /**
     * @brief The identifier of the message.
     */
    uint32_t identifier;

    /**
     * @brief Whether the message is an extended CAN message (probably not).
     */
    bool extended;

    /**
     * @brief The length of the message in ????.
     */
    uint8_t length;

    /**
     * @brief The message data.
     */
    uint8_t data[CAN_MAX_MESSAGE_SIZE_BYTES];
} can_frame_t;

static can_frame_t can1_rx_frame;
static volatile bool can1_rx_flag;

static can_frame_t can2_rx_frame;
static volatile bool can2_rx_flag;

/**
 * @brief Overridden write syscall.
 */
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

/**
 * @brief Setup the USART peripheral.
 */
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

/**
 * @brief Put a character to the USART.
 * 
 * @param string the character to put
 */
static void usart_puts(char *string) {
    while (*string) {
        usart_send_blocking(USART1, *string);
        string++;
    }
}

/**
 * @brief Put an entire line to the USART and terminate it with a CRLF.
 * 
 * @param string the line to put, no need to add a CRLF
 */
static void usart_putln(char *string) {
    usart_puts(string);
    usart_puts("\r\n");
}

/**
 * @brief Initialise a CAN peripheral.
 * 
 * @param can_port the CAN peripheral to init
 */
static void can_setup(uint32_t can_port) {
    can_reset(can_port);

    /*
    can_init is essentially a fancy way of setting the bxCAN periph registers, the most important of
    these being CAN_BTR (the bit timing register)

    seealso: http://www.bittiming.can-wiki.info/
    */
    can_init(
        can_port,
        false,
        true,
        true,
        false,
        false,
        false,
        CAN_BTR_SJW_1TQ,
        CAN_BTR_TS1_15TQ,
        CAN_BTR_TS2_2TQ,
        4,
        false,
        false
    );
}

/**
 * @brief Initialise the LV CAN peripheral.
 */
static void lv_can_setup(void) { // CAN1
    gpio_primary_remap(false, AFIO_MAPR_CAN1_REMAP_PORTB);
    rcc_periph_clock_enable(RCC_CAN1);
    can_setup(CAN1);
    can_enable_irq(CAN1, CAN_IER_FMPIE0);
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
}

/**
 * @brief Initialise the HV CAN peripheral.
 */
static void hv_can_setup(void) { // CAN2
    rcc_periph_clock_enable(RCC_CAN2);
    can_setup(CAN2);
    can_enable_irq(CAN2, CAN_IER_FMPIE0);
    nvic_enable_irq(NVIC_CAN2_RX0_IRQ);
}

/**
 * @brief Firmware entrypoint.
 */
int main(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE25_72MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    usart_setup();

    usart_putln("Southampton University Formula Student Team");
    usart_putln("CAN isolator firmware");
    usart_putln("I=info, W=warning, E=error, F=fatal");
    usart_putln("I: starting...\r\n"); // double line break

    // enters a sleep state when exiting an ISR
    SCB_SCR |= SCB_SCR_SLEEPONEXIT;

    usart_putln("I: system initialisation done");

    for (;;) {
        WFI();
    }

    usart_putln("F: exceeded main loop");
    return 0;
}

/*
In both interrupt service routines, we just set a flag. The main loop is still responsible for
actually handling the data.

This keeps the time spent in ISRs at a minimum.
*/

/**
 * @brief CAN1 FIFO 0 interrupt service routine.
 * 
 * On non-F103 STM32F1s, the USB and CAN share a SRAM memory block, cannot be used at once, and so
 * share an interrupt line.
 * 
 * The name of this ISR is a slight misnomer for connectivity line devices like the F103 as
 * libopencm3 assumes we are not using a connectivity line chip. The interrupt vector address is
 * still correct, however.
 * 
 * The canonical name for this interrupt vector is `CAN1_RX0` on connectivity line chips.
 */
void usb_lp_can_rx0_isr(void) {
    can1_rx_flag = true;
}

/**
 * @brief CAN2 FIFO 0 interrupt service routine.
 */
void can2_rx0_isr(void) {
    can2_rx_flag = true;
}
