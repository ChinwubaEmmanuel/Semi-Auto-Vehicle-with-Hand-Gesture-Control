// UART2 Library

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U2TX (PD7) and U2RX (PD6) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart2.h"
#include "gpio.h"

// Pins
#define UART_TX PORTD,7
#define UART_RX PORTD,6

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART2
void initUart2(void)
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R2;
    _delay_cycles(3);
    enablePort(PORTD);

    // Configure UART2 pins
    setPinCommitControl(UART_TX);
    selectPinPushPullOutput(UART_TX);
    selectPinDigitalInput(UART_RX);
    setPinAuxFunction(UART_TX, GPIO_PCTL_PD7_U2TX);
    setPinAuxFunction(UART_RX, GPIO_PCTL_PD6_U2RX);

    // Configure UART0 with default baud rate
    UART2_CTL_R = 0;                                    // turn-off UART2 to allow safe programming
    UART2_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (usually 40 MHz)
}

// Set baud rate as function of instruction cycle frequency
void setUart2BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART2_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART2_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART2_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART2_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART2_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART2
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart2(char c)
{
    while (UART2_FR_R & UART_FR_TXFF);               // wait if uart2 tx fifo full
    UART2_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart2(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart2(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart2(void)
{
    while (UART2_FR_R & UART_FR_RXFE);               // wait if uart2 rx fifo empty
    return UART2_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart2(void)
{
    return !(UART2_FR_R & UART_FR_RXFE);
}
