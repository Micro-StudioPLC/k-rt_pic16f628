/*
* K-RT (v2.1.4)
* Microcontrollers Real Time Kernel / Scheduler / Tool set
*
* RT      Real time tasks for User and Kernel calls
* ST      Scheduled (stacked) function calls for User and Kernel calls
* shell   For remote control
* systat  For runtime statistics and fault, alarms, status (overload, ...)
*
* K-RT Project Public version is under GPL-3.
* For dual licensing requests please contact author at jfsimon@startmail.com
*/

/*
* K-RT The Real Time Kernel for Microcontrollers.
* Copyright (C) 2025  Jean-François Simon, Chrysalide Engineering
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
* This Software License is only suitable for Open-Source projects.
* For commercial license please contact the author.
*/

/*
* Version 2.0 30 may 2025
*
* The Micro-Kernel and Scheduler are constructed for resource constraint
* environments (eg microcontrollers with kilobyte(s) of flash/ram)
*
* The K-RT Kernel provides:
*   - a Scheduler for RT (Real Time) tasks operating at a fixed period of 4ms
*   - a Scheduler for ST (Scheduled Time) tasks operaing on average at the
*     specified period.
*
* The real time code provides guarantee that the code runs exactly at the
* typical 4ms / 20ms / 100ms / 1s period, +/- the execution time.
*
* Real time task can be monitored by Watchdog with a period as low as 17ms
* (default for PIC16F886).
*
* ST (scheduled tasks) are run by the RT (real time) scheduler, and executed
* outside of interrupt context.
* RT (real time tasks) are run in interrupt context (Timer1) with a base cycle
* of 4ms.
*
* Constraints
*
* All RT tasks have to run within the 4ms period, ie., 4-20-100ms and 1s RT
* tasks will eventually run together within a 4ms period, ie 10k instructions
* at most are allowed. Any code that does not need RT context can be exported
* into ST tasks.
*
* RT supervision
*
* RT task Load is supervised by systat() and provides User with information on
* current load (min, max, avg) of tasks running in RT context.
*
* The present code enables the WDT (watchdog timer) with a 17ms period, which
* is cleared in the 4ms RT task.
*
* ST buffer overload
*
* If the ST buffer is full, next scheduling with result in a NOP and the task
* execution will not be booked in the ST scheduler and lost.
*
* If the tasks scheduled in ST scheduler are not critical, it is not harfmul
* to K-RT that either the scheduler is called with an empty task or called
* while the scheduler task list is already full, and therefore simply discarding
* the request.
*
* Man page v1.0 30/5/2025
*/

/*
 * User space functions
 *
 * Synopsis
 *
 * K_RT uses RT (real time) and ST (scheduled time) function calls. Both
 * are run on periodic bases. RT are called from interrupt and run within
 * interrupt context, they have execution time constraint. ST are loaded
 * in the scheduler for execution once the interrupt returns, and as long
 * as any pending task exists in the scheduler.
 * Once all tasks are executed, the main loop runs mostly idle.
 *
 * RT is interrupt context.
 * Execution time far below 4ms required for all 4ms/20ms/100ms/1s functions.
 * All RT tasks will eventually run during the same interrupt call, therefore
 * the total RT space (User and Kernel for all periods) should be executable
 * within the base cycle (defaults 4ms).
 *
 * Normal tasks are scheduled on best effort.
 * Max execution time within their associated period, on average, but can be
 * deffered. For example the 20ms ST user space function may run after a 1s
 * ST task which coud take 50ms CPU process time. Once execution is passed,
 * all pending tasks, which could include instance(s) of 20ms ST will run.
 *
 * See user.h for more infos
 */

/*
 * Performance
 * Buffer and Interrupts: avoir busy waits, use buffer when possible instead.
 * Check Memory performance for all targets (including PIC 16F628 serie)
 */

// #include include Hardware Definitions as needed eg Timer definition
// #include <xc.h> // Ex. for Microchip XC8
#include <xc.h> // For Microchip XC8
#define _XTAL_FREQ 4000000 // also update in main.c if changing

#include "k_rt_user.h"

inline void k_rt_task_4ms_user(void) {
  // Real time interrupt context, exec below 4 ms

    /* Place your application code hereunder */
    // ...

    if (k_alarm() || k_fault()) {
      RA2 = 1;
    } else {
      RA2 = 0;
    }

    while (rx_content) {
      char ci = k_getchar();
      // Response composing:
      {
        k_putchar(ci); // Readback
        k_command(ci); // Action (Execute K-RT commands and potentially
                       // send back serial data)
      }
    }
}
inline void k_rt_task_20ms_user(void) {
    // Real time interrupt context, exec below 4 ms

    // Monitor and reset if needed USART peripheral
    {
      if (FERR) {
        // Framing Error
        // TODO Feed Systat Com error stat (to be implemented)
        uint8_t rx = RCREG;
      }
      if (OERR) {
        // Overrun Error
        // TODO Feed Systat Com error stat (to be implemented)
        CREN = 0;
        CREN = 1;
      }
    }

      /* Place your application code hereunder */

      // ...
}
inline void k_rt_task_100ms_user(void) {
    // Real time interrupt context, exec below 4 ms

    /* Place your application code hereunder */

    // ...
    }

inline void k_rt_task_1s_user(void) {
    // Real time interrupt context, exec below 4 ms

    /* Place your application code hereunder */
}

/*
 * Non real time functions can be delayed. They are stacked and will
 * eventually pick-up with their scheduled period. They will respect on
 * average the task period unless the scheduler becomes overloaded.
 */

// krt user : 4ms 20ms 100ms 1s
// kst user : 4ms 20ms 100 1s

void k_st_task_4ms_user(void) {
  // Scheduled 4ms task (buffered: execution may be deffered)

  /* Place your application code hereunder */

  // ...

}

void k_st_task_20ms_user(void) {
  // Scheduled 20ms task (buffered: execution may be deffered)

  /* Place your application code hereunder */

  // ...

}

void k_st_task_100ms_user(void) {
  // Scheduled 100ms task (buffered: execution may be deffered)

  /* Place your application code hereunder */

  // ...
}

void k_st_task_1s_user(void) {
  // Scheduled 100ms task (buffered: execution may be deffered)

  /* Place your application code hereunder */

  // ...

  // Heartbeat
  // This is a 1s pulse (2s period) of RA0 pin running from user ST loop.
  // If User scheduler is not running, this task will be halted.
#if 1
  {
      static int ra0 = 0;
      ra0 ^= 1;
      RA0 = ra0;
  }
#endif
}

/* **********************  ********************** */

// Serial Read/Writes and Commands

/*
  Sending through UART

  We load TX reg (initializes the line to TX)
  or the TX buffer if the line was busy.
  Interrupt will then be trig when TX freed and
  proceed with the buffer onwards until empty.

  k_putchar()
  Check is TX in progress
  Free: Loads ci in txreg directly
  Busy: Loads ci in buffer through k_putchar_write_buff()

  k_getchar()
  Upon receipt, the Interrupt trigs and calls RX->buffer function.
  We always immediately load the RX buffer and return from Interrupt.
  Buffer may be free or full, accept/reject RX is handled in
  k_getchar_write_buff().
  Getter k_getchar() always read from buffer.
  Setter k_getchar_write_buff() runs in interrupt context.
*/

// k_getchar() and k_putchar() implementation is custom to Hardware

// getchar() and putchar() are User defined (Hardware specific)
// These are the Serial I/O Drivers.

/* ********************** k_getchar_write_buff() ********************** */

// Receive buffer
int rx_b_ptr = 0;
int rx_e_ptr = 0;
// Running in Interrupt Context
char k_getchar_write_buff(char c_in) {
  if (rx_content < RX_BUF_SIZE) {
    rx_buffer[rx_e_ptr++] = c_in;
    if (rx_e_ptr == RX_BUF_SIZE)
      rx_e_ptr = 0;
    rx_content++; // Important: do this only after fill a buffer slot
                  // We need handle valid data before inform the external code
  } else {
    // Error handler, RX Buff full (buffer overrun)
    // TODO
  }
  return 0;
}

//012345670
//b.......e
//........e
//c8

/* ********************** k_getchar() ********************** */

// Reads serial input
char k_getchar() {
  // Reads serial input from buffer
  // char rv = k_getchar_read_buff()
  if (rx_content > 0) {
    char rv = 0;
    rv = rx_buffer[rx_b_ptr++];
    if (rx_b_ptr == RX_BUF_SIZE) {
      rx_b_ptr = 0;
    }
    rx_content--; // Important: do this only after empty a buffer slot
                  // We need handle valid data before inform the external code
    return rv;
  } else {
    // Error handler, RX Buff empty (buffer underrun)
    // TODO
  }
  return 0;
}

/* ********************** k_putchar_write_buff() ********************** */

// Transmit buffer
int tx_b_ptr = 0;
int tx_e_ptr = 0;

char k_putchar_write_buff(char ci) {
  if (tx_content < TX_BUF_SIZE) {
    tx_buffer[tx_e_ptr++] = ci;
    if (tx_e_ptr == TX_BUF_SIZE)
      tx_e_ptr = 0;
    tx_content++; // Important: do this only after fill a buffer slot
                  // We need handle valid data before inform the external code
    return ci;
  } else {
    // Error handler, TX Buff full (buffer overrun)
    // TODO
  }
  return 0;
}

/* ********************** k_putchar_read_buff() ********************** */

// Called from interrupt context
char k_putchar_read_buff() {
  if (tx_content > 0) {
    char rv = 0;
    rv = tx_buffer[tx_b_ptr++];
    if (tx_b_ptr == TX_BUF_SIZE) {
      tx_b_ptr = 0;
    }
    tx_content--; // Important: do this only after empty a buffer slot
                  // We need handle valid data before inform the external code
    return rv;
  } else {
    // Error handler, TX Buff empty (buffer underrun)
    // TODO
  }
  /*
    TODO: Robustness
    For safety precaution, in ST 4ms loop:
    - Check TX buffer empty
    - If not empty, check TX status & confirm sending
    - If buffer not empty & TX not sending, timeout 5 x 4ms then reset TX
      and raise Logic Fault (auxiliary fault word).
  */
  return 0;
}

/* TODO
  Systat
  Max TX/RX Buffer */

/* ********************** k_putchar() ********************** */

// Sends serial output
char k_putchar(char ci) {
  // Loading TX reg or buffer
  // Interrupt version
  if (TRMT && !tx_content) {
    // No TX in progress and Buffer empty, load TX register
    // Otherwise we let the Interrupt handle TX appropriately.
    TXREG = ci;
    return ci;
  } else {
    // TX in progress, load buffer
    char rv = 0;
    rv = k_putchar_write_buff(ci);

    TXIE = 1; // So that INT will empty the buffer after each TX term
    return rv;
  }
 /* Busyloop version (Less Interrupt load, more cpu load)
 TXREG = ci;
 while (!TRMT);
 return ci; */
}

/* **********************  ********************** */

// Custom User command decoder
inline char k_command_user(char ci) {
  // Refactor / Optimize
  // Program flash, reinit call, caution stack depth

  // Important
  // When adding capabilities, put the init instruction on function end
  // (Stateless instructions: New activity) and stateful instruction in the beggining (Statuful
  // instructions: Continuation)

  static uint8_t state = 0;
  static uint8_t byte_0 = 0;
  static uint8_t byte_1 = 0;
  static uint8_t byte_2 = 0;
  static uint8_t loop_cnt = 0;

  if (ci == CMD_CLR_SERIAL) {
    loop_cnt = 0;
    state = 0;
    byte_0 = 0; // Set upon entry     (state == 0, loop == 0)
    byte_1 = 0; // Set upon reentry   (state == 1, loop == 1)
    byte_2 = 0; // Set upon reentry   (state == 1, loop == 2)
    state = 0;  // Reset upon reentry (state == 1, loop == 3)
    return (char)(0);
  }

  // Statuful instructions: Continuation
#if 1
  {
    if (state) {
      // Timeout handler
      loop_cnt++;
      {
        if (loop_cnt == 1) {
          // byte_0 holds the command, it remains intact.
          // byte_1 holds the GPIO address, we need to clear MSB (user packet).
          byte_1 = ci;
          byte_1 &= 0b00111111; // GPIO Addresses 6 bits wide
        } else if (loop_cnt == 2) {
          byte_2 = ci;
          byte_2 &= 0b01111111; // Only 7 bits values transmitted
        } else  if (loop_cnt == 3) {
          // Overrun
          // Reinit.
          // Notice this code can handle packets of 3 bytes at most.
          // K-RT specs supports much more.
          loop_cnt = 0;
          state = 0;
          byte_0 = 0;
          byte_1 = 0;
          byte_2 = 0;
          return (char)(-1);
        }
      }

      // loop == 1

      // User application handler
      // 2 packets transactions:

      // GPIO set as DI

      if (byte_0 == CMD_GEN_GNCI) {

        // Deals with request now that we have the missing information(s)
        // GPIO Config : adr. ci --> DI

        /*
        *  Mapping:
        *  RA0-7 (not all are HW supported) : addr 0-7
        *  RB0-7 (not all are HW supported) : addr 8-15
        */

        // CI address from SP: 0...127
        // Mapping direct.
        /*
        0  -> TRISA 0b'00000001
        1  -> TRISA 0b'00000010
        2  -> TRISA 0b'00000100
        3  -> TRISA 0b'00001000
        4  -> TRISA 0b'00010000
        5  -> TRISA 0b'00100000
        6  -> TRISA 0b'01000000
        7  -> TRISA 0b'10000000

        8  -> TRISB 0b'00000001
        9  -> TRISB 0b'00000010
        10 -> TRISB 0b'00000100
        11 -> TRISB 0b'00001000
        12 -> TRISB 0b'00010000
        13 -> TRISB 0b'00100000
        14 -> TRISB 0b'01000000
        15 -> TRISB 0b'10000000
        */
        if (byte_1 < 8) {
          TRISA |= (1<<byte_1); // Range 0-7 Setup GPIO ... as DI
        } else if (byte_1 < 16) {
          TRISB |= (1<<(byte_1 - 8)); // Range 8-15 -> 0-7 Setup GPIO ... as DI
        }

        // Reinit
        {
          loop_cnt = 0;
          state = 0;
          byte_0 = 0;
          byte_1 = 0;
          byte_2 = 0;
        }
        return (char)(0); // End of stateful transaction
      } else if (byte_0 == CMD_GEN_GNCO) {

        // GPIO set as DO
        if (byte_1 < 8) {
          TRISA &= ~(1<<byte_1); // Range 0-7 Setup GPIO ... as DI
        } else if (byte_1 < 16) {
          TRISB &= ~(1<<(byte_1 - 8)); // Range 8-15 -> 0-7 Setup GPIO ... as DI
        }

        // Reinit
        {
          loop_cnt = 0;
          state = 0;
          byte_0 = 0;
          byte_1 = 0;
          byte_2 = 0;
        }

      } else if (byte_0 == CMD_GEN_GNCR) {
        // Readback GPIO Digital value --> Configuration
        char rv_l = 0;
        if (byte_1 < 8) {
          rv_l = TRISA & (1<<byte_1);

        } else if (byte_1 < 16) {
          rv_l = TRISB & (1<<(byte_1 - 8)); // Range 8-15 -> 0-7 Setup GPIO ... as DI
        }
        if (rv_l) k_putchar(1);
        else k_putchar(0);
        // Reinit
        {
          loop_cnt = 0;
          state = 0;
          byte_0 = 0;
          byte_1 = 0;
          byte_2 = 0;
        }

      } else if (byte_0 == CMD_GEN_GNCPU) {
        // GPIO configure Pull-Up
        // Set the PORTB Pull up alltogether
        nRBPU = 0; // PU active (/RBPPU)

        // Reinit
        {
          loop_cnt = 0;
          state = 0;
          byte_0 = 0;
          byte_1 = 0;
          byte_2 = 0;
        }

      } else if (byte_0 == CMD_GEN_GNCOD) {
        // GPIO configure Pull-Up
        // Clear the PORTB Pull up alltogether
        nRBPU = 1; // PU inactive (/RBPPU)

        // Reinit
        {
          loop_cnt = 0;
          state = 0;
          byte_0 = 0;
          byte_1 = 0;
          byte_2 = 0;
        }

      } else if (byte_0 == CMD_GEN_GNRD) {
        // Readback GPIO Digital value
        char rv_l = 0;
        if (byte_1 < 8) {
          rv_l = PORTA & (1<<byte_1);

        } else if (byte_1 < 16) {
          rv_l = PORTB & (1<<(byte_1 - 8)); // Range 8-15 -> 0-7 Setup GPIO ... as DI
        }
        if (rv_l) k_putchar(1);
        else k_putchar(0);

        // Reinit
        {
          loop_cnt = 0;
          state = 0;
          byte_0 = 0;
          byte_1 = 0;
          byte_2 = 0;
        }
      } else if (byte_0 == CMD_INF_HWMCU) {

        k_putchar(HW_MCU_MFR);
        k_putchar(HW_MCU_TYPE);

        // Reinit
        {
          loop_cnt = 0;
          state = 0;
          byte_0 = 0;
          byte_1 = 0;
          byte_2 = 0;
        }
      }

      // ... else if ... 2 packet transctions here
      // 3 packets transactions:
      if (loop_cnt == 2) {
        // 3 packets available here
        if (byte_0 == CMD_GEN_GNWD) {
          if (!byte_2) {
            // Write 0 -> PortN[byte_1]
            if (byte_1 < 8) {
              PORTA &= ~(1<<byte_1);
            } else if (byte_1 < 16) {
              PORTB &= ~(1<<(byte_1 - 8));
            }
          } else {
            // Write 1 -> PortN[byte_1]
            if (byte_1 < 8) {
              PORTA |= (1<<byte_1);
            } else if (byte_1 < 16) {
              PORTB |= (1<<(byte_1 - 8));
            }
          }

          // Reinit
          {
            loop_cnt = 0;
            state = 0;
            byte_0 = 0;
            byte_1 = 0;
            byte_2 = 0;
          }

        }
      }
      // Stateful transaction not terminated (we expect more packet(s))
      return 0;
    }
  }
#endif

  // Stateless instructions
  // New activity
  {
    // ci bit 7 always 1. bits 6-0 are user defined.
    if (ci & (1<<7)) {
      // User or Application defined decoder

      /* Implement custom code here ... */

      // Decode all special instructions. These are multi-bytes and need the
      // states be recorded and previous byte(s).
      // This implementation supports 3 bytes at most (present + 2 past recorded)
      // which is sufficient for all GPIO config and manipulations.
      if (ci == CMD_GEN_GNCI) {
        // GPIO Config ci : Digital input
        byte_0 = ci; // Record command
        state = 1; // Stateful activity
      } else if (ci == CMD_GEN_GNCO) {
        // GPIO Config ci : Digital output
        byte_0 = ci; // Record command
        state = 1; // Stateful activity
      } else if (ci == CMD_GEN_GNCR) {
        // GPIO Config cr : Read-back
        byte_0 = ci; // Record command
        state = 1; // Stateful activity
      } else if (ci == CMD_GEN_GNCOD) {
        // GPIO Config cod : Open drain
        byte_0 = ci; // Record command
        state = 1; // Stateful activity
      } else if (ci == CMD_GEN_GNCPU) {
        // GPIO Config cpu : pull-up
        byte_0 = ci; // Record command
        state = 1; // Stateful activity
      } else if (ci == CMD_GEN_GNRD) {
        // GPIO Readback Digital
        byte_0 = ci; // Record command
        state = 1; // Stateful activity
      } else if (ci == CMD_GEN_GNWD) {
        // GPIO Write Digital
        byte_0 = ci; // Record command
        state = 1; // Stateful activity
      } else if (ci == CMD_INF_HWMCU) {
        // MCU Infos
        byte_0 = ci; // Record command
        state = 1; // Stateful activity
      }
      // .. else if ...
    } else {
      // Error, bit 7 at 0
      return (char)(-1);
    }
  }
  return 0;
}

/* **********************  ********************** */
