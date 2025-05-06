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

#ifndef UART2_H_
#define UART2_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart2();
void setUart2BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart2(char c);
void putsUart2(char* str);
char getcUart2();
bool kbhitUart2();

#endif /* UART2_H_ */
