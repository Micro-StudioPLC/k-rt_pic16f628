/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#include "user.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

void InitApp(void)
{

    // Timer0 setup
    OPTION_REG = 0b00000100; // Prescaler 1:32 assigned to Timer0

    // Preload TMR0
    TMR0 = 131;

    /* TODO Initialize User Ports/Peripherals/Project here */

    // Serial Com setting, UART asynchronous, 19'200 Baud
    // RX = pin 7 (RB1/RX/DT) and TX = pin 8 (RB2/TX/CK)
    {
      SYNC = 0;
      SPEN = 1; // Enable XMT
      CREN = 1; // Enable RCV
      // TRISB = ((1 << 2) | (1 << 1)); // Same as Reset
      TXEN = 1;
      // Set for 19'200 Baud (4 MHz, 0.160% error)
      SPBRG = 12;
      BRGH = 1;
    }

    /* Setup analog functionality and port direction */

    // Disable comparators, all PORTA pins digital
    CMCON = 0x07;

    // Set RA1 as output
    TRISA = 0b11111000;  // RA2:0 = 0 (output), others = input
    PORTA = 0;           // Clear output latches

    /* Initialize peripherals */

    /* Enable interrupts */
    T0IE = 1; // Enable Timer0 interrupt
    PEIE = 1; // Enable peripheral interrupts
    TXIE = 0; // USART TX interrupt (TXIF)
    RCIE = 1; // USART RX interrupt (RCIF)
    GIE = 1;  // Global interrupt enable
}
