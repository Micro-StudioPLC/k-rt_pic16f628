/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#include "k_rt_v2.1/k_rt_user.h"

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* Baseline devices don't have interrupts. Note that some PIC16's
 * are baseline devices.  Unfortunately the baseline detection macro is
 * _PIC12 */
#ifndef _PIC12

// void interrupt isr(void)
void __interrupt() isr(void)
{
#if 1
    if (T0IF)
    {
        // Clear / Setup INT
        {
            T0IF = 0;
            TMR0 = 131; // Preload (4ms period)
        }

        // k_rt uses just one call INTR context 4ms.
        k_rt_task_4ms_isr_entry();

    } else if (PIR1bits.TXIF && PIE1bits.TXIE) {
      // Serial TX complete interrupt
      // NOP
    } else if (PIR1bits.RCIF && PIE1bits.RCIE) {
      // Serial RX received interrupt
      uint8_t rx = RCREG; // Read RCREG and clears RCIF
      k_getchar_fill_buff(rx);
    } else {
        // Unhandled interrupts
    }
#endif
}
#endif
