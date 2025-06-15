/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */

#include "k_rt_v2.1/k_rt_user.h"

#define _XTAL_FREQ 4000000  // Internal oscillator frequency (4 MHz)
                            // also update in k_rt_user.c if changing

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

/* i.e. uint8_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
void main(void)
{
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize I/O and Peripherals for application */
    InitApp();

    k_systat_init();

    while (1) {
        // Scheduler call
        while (k_task_is_scheduled() || k_task_is_scheduled_user()) {
            if (k_task_is_scheduled()) k_task_handler();
            if (k_task_is_scheduled_user()) k_task_handler_user();
        }
        // Idle, collecting stats
        k_idle();

        // Max idle before Warning:
        // For tests only
        // __delay_ms(200); // Warning > ~ 210ms
#if 0
        // Generating port perturbation
        RA0 = 1;
        RA0 = 0;
        RA0 = 1;
        RA0 = 0;
        RA0 = 1;
        RA0 = 0;
#endif
    }
}
