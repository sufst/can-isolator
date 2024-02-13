# CAN isolator

The CAN isolator project is a very simple firmware package designed to run on the SUFST CAN isolator board. This is an STM32F105-based board.

The CAN isolator sits between the low and high voltage electronics in the car and sends CAN messages between them (from and to CAN1 and CAN2 respectively).

## Hardware

The following details about the hardware are pertinent:

- the microcontroller is an STM32F105RB with 128 Kbytes of flash
- CAN1 is on PB8 (RX) and PB9 (TX); this is the LV side
- CAN2 is on PB12 (RX) and PB13 (TX); this is the HV side
- the microcontroller is connected to an external 25 MHz clock
- 
