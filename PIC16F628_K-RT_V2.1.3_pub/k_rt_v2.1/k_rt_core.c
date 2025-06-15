/*
* K-RT (v2.1.2)
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
* RT task Load is supervised by systat and provides User with information on
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
* PIC16F886 Implementation
*
*/

// #define MAIN_CLOCK_FREQUENCY 10000000 // Value in Hz    TODO : Add config file.


#include <stdint.h>         /* For uint8_t definition */
#include "k_rt_core.h"
#include "k_rt_user.h"

// Private
// ST Tasks definitions.
#define TASK_4ms   1
#define TASK_20ms  2
#define TASK_100ms 3
#define TASK_1s    4

/*
 * TODO
 * Performance tests (latencies with all loops under MCU ports at various
 * XTAL, 4 MHz, 10 MHz, 20 MHz etc ...
  */

// Error codes
#define ERRNO_UNHANDLED_COMMAND_REQUEST 1

/*
 * Attributes
 * i : To be init
 * p : Public
 * r : Private (restricted)
 *
 * In k_rt_core file, default is Private for all variables. Unless marked p
 * for Public, variables are r (Private).
 */

/*
 * Command word
 * Fault & Alarm word
 * Status word
 *
 * Command (CD) word
 *   b7: Custom command: if set, other 7 bits are User defined.
 *   b6: sta, status: Return Status, Alarm and Fault words (3 char in this order)
 *   b5: rust, runust: Run U-ST (User Space scheduler)
 *   b4: sust, stopust: Stop U-ST (User Space scheduler)
 *   b3: clr, clear : Clear Fault and Alarm Words
 *   b2: sys, systat: Read-back systat then reset min/max (note 1 below)
 *   b1: rkst, runkst: Run K-ST (Kernel Space scheduler)
 *   b0: skst, stopkst: Stop K-ST (Kernel Space scheduler). Shell should be in RT context.
 *
 * Note 1 :
 *        Read back k_systat_load sends 8 char below in this order:
 *          rt_task_min
 *          rt_task_max
 *          rt_task_avg
 *          rt_tmr_reload_value
 *          rt_tmr_rollover
 *          st_task_min
 *          st_task_max
 *          st_task_min
 *          st_task_max
 *
 * Status (ST) word : k_status_word
 *   b7: A Fault for User space exists
 *   b6: An Alarm for User space exists
 *   b5: Provision (reads 0)
 *   b4: U-ST (User Space scheduler) running
 *   b3: A Fault for Kernel exists
 *   b2: An Alarm for Kernel exists
 *   b1: Provision (reads 0)
 *   b0: K-ST (Kernel Space scheduler) running
 *
 * Fault (FL) word : k_fault_word
 *   b7: Provision (reads 0)
 *   b6: Provision (reads 0)
 *   b5: Provision (reads 0)
 *   b4: US (User Space) stack full
 *   b3: Wadtchdog timer triggered (RT overload) TODO
 *   b2: Provision (reads 0)
 *   b1: Provision (reads 0)
 *   b0: RT (Real time) stack Full
 *
 * Alarm (AL) word : k_alarm_word
 *   b7: Provision (reads 0)
 *   b6: Provision (reads 0)
 *   b5: Provision (reads 0)
 *   b4: US (User Space) Stack load is high
 *   b3: Provision (reads 0)
 *   b2: Provision (reads 0)
 *   b1: Provision (reads 0)
 *   b0: RT (Real time) Stack load is high
 *
 * Fault word (FL) does not reset. Fault needs to be cleared by user.
 *
 * Alarm (AL) and Status (ST) words are update constantly and reflect actual
 * state. They do not need to be cleared.
 */

  // Public accessors

  // Private

  // Fault, Alarm, Status, Control

 char k_fault_word = 0;  // i,r
 char k_alarm_word = 0;  // i,r
 char k_status_word = ((1<<4) | (1<<0)); // i,r

 // Returns fault word
 inline char k_fault() {
   return k_fault_word;
 }
 // Returns alarm word
 inline char k_alarm() {
   return k_alarm_word;
 }
 // Clear fault word
 inline void k_fault_clr() {
   k_fault_word = 0;
 }
 // Clear alarm word
 inline void k_alarm_clr() {
   k_alarm_word = 0;
 }
 // Returns status word
 inline char k_status() {
   return k_status_word;
 }

 // K-RT Command Handler (responds to external requests)
 inline char k_command(char ci) {
   // Bit decoder
   if (!(ci & (1<<7))) {
     // Standard command
    if (ci & (1<<6)) {
      // SP Command: sta, status
      // Read back Status, Alarm, Fault words chars
      k_putchar(k_status_word);
      k_putchar(k_alarm_word);
      k_putchar(k_fault_word);
    } else if (ci & (1<<5)) {
      // SP Command: rust, runust
      // Run U-ST (User Space scheduler)
      k_status_word |= (1<<4); // U-ST 1
    } else if (ci & (1<<4)) {
      // SP Command: sust, stopust
      // Stop U-ST (User Space scheduler)
      k_status_word &= ~(1<<4); // U-ST 0
    } else if (ci & (1<<3)) {
      // SP Command:
      // Clear Fault Word
      // Usage of private words. Not doing so is a performance bug.
      k_fault_word = 0; // Private used in private context, ok
      k_alarm_word = 0; // Private used in private context, ok
    } else if (ci & (1<<2)) {
      // SP Command: sys, systat
      // Read-back systat
      /*
       * V2.1.1 list of 8 elements in order:
       * RT Tasks load Min (uint8_t)
       * RT Tasks load Max (uint8_t)
       * RT Tasks load Avg (uint8_t)
       * Tmr reload        (uint8_t)
       * Tmr rollover      (uint8_t)
       * ST Task load Min  (uint8_t)
       * ST Task load Max  (uint8_t)
       * ST Task load Min  (uint8_t)
       * ST Task load Max  (uint8_t)
       *
       */

      k_putchar((char)k_systat_load_rt_task_min());
      k_putchar((char)k_systat_load_rt_task_max());
      k_putchar((char)k_systat_load_rt_task_avg());
      k_putchar((char)k_systat_load_rt_tmr_reload());
      k_putchar((char)k_systat_load_rt_tmr_rollover());

      k_putchar((char)k_systat_load_st_task_min());
      k_putchar((char)k_systat_load_st_task_max());
      k_putchar((char)u_systat_load_st_task_min());
      k_putchar((char)u_systat_load_st_task_max());

      // TODO
      // Add a command word to read Kernel version, configuration, clock, ...
      // Export static readings into it (performance)
      k_putchar((char)k_systat_load_st_task_avail());
      k_putchar((char)u_systat_load_st_task_avail());

      // Reset all systat counters
      k_systat_init();

    } else if (ci & (1<<1)) {
      // SP Command: rkst, runkst
      // Run K-ST (Kernel Space scheduler)
      k_status_word |= (1<<0); // U-KT 1
    } else if (ci & (1<<0)) {
      // SP Command: skst, stopkst
      // Stop K-ST (Kernel Space scheduler)
      k_status_word &= ~(1<<0); // U-KT 0
    } else {
      // Unhandled request
      return ERRNO_UNHANDLED_COMMAND_REQUEST;
    }
   // Custom commands
   } else {
     k_command_user(ci);
   }

   // On failure return error code
   // return errno;

   // On success return 0
   return 0;
 }

// Private
// Scheduler
char scheduler_stack[K_ST_SCHEDULER_STACK_SIZE]; // i,r
char k_sched_ptr_begin = 0;
char k_sched_ptr_end = 0;
char act_stack = 0;
char t20ms = 0;
char t100ms = 0;
char t1s = 0;

char scheduler_stack_user[U_ST_SCHEDULER_STACK_SIZE]; // i,r
char u_sched_ptr_begin = 0;
char u_sched_ptr_end = 0;
char act_stack_user = 0;

// systat
char act_stack_min = K_ST_SCHEDULER_STACK_SIZE;
char act_stack_max = 0;
char act_stack_user_min = U_ST_SCHEDULER_STACK_SIZE;
char act_stack_user_max = 0;

// RT context
 inline void k_rt_task_4ms_isr_entry() {
    // Systat works
    k_systat_load_rt_task_in();
    // Watchdog clearing
    asm("CLRWDT");
    // Scheduling
    t20ms++;
    if (t20ms==5) {
        t20ms=0;
        k_rt_task_20ms();
        k_rt_task_20ms_user();
    }

    t100ms++;
        if (t100ms==25) {
        t100ms=0;
        k_rt_task_100ms();
        k_rt_task_100ms_user();
    }

    t1s++;
    if (t1s==250) {
        t1s=0;
        k_rt_task_1s();
        k_rt_task_1s_user();
    }

    // Fast task running
    k_rt_task_4ms_user();

    // Scheduler works for fast tasks
    k_schedule_task(TASK_4ms);
    k_schedule_task_user(TASK_4ms);

    // Systat works
    k_systat_load_rt_task_out();
    k_systat_load_rt_task_calc();
    k_systat_update_status();

    // Heartbeat
    // This is a 4ms pulse (8ms period) of RA1 pin running from Kernel RT loop.
    // It should always be running as long as the MCU and Timer interrupt are
    // good.
#if 0
    {
        static int ra1 = 0;
        ra1 ^= 1;
        RA1 = ra1;
    }
#endif
}

// RT context
inline void k_rt_task_20ms() {
    // Places tasks in scheduler
    k_schedule_task(TASK_20ms);
    k_schedule_task_user(TASK_20ms);
    //
#if 0
    {
        static int ra1 = 0;
        ra1 ^= 1;
        RA1 = ra1;
    }
#endif
}

// RT context
inline void k_rt_task_100ms() {
    // Place ST tasks in scheduler
    k_schedule_task(TASK_100ms);
    k_schedule_task_user(TASK_100ms);
}

// RT context
inline void k_rt_task_1s() {
    // Place ST tasks in scheduler
    k_schedule_task(TASK_1s);
    k_schedule_task_user(TASK_1s);
}

// Kernel Scheduled tasks

// ST context

void k_st_task_4ms() {
  // ST Hearbeat
#if 1
  {
      static int ra1 = 0;
      ra1 ^= 1;
      RA1 = ra1;
  }
#endif
}

void k_st_task_20ms() {
  // ST Hearbeat
#if 0
  {
      static int ra1 = 0;
      ra1 ^= 1;
      RA1 = ra1;
  }
#endif
}

// ST context
void k_st_task_100ms() {
}

// ST context
void k_st_task_1s() {
}

/* Loads the scheduler stack with the function requested */

// RT context
inline void k_schedule_task(char task) {
    if (scheduler_stack[k_sched_ptr_end]==0) {
      // Load entry
      {
        act_stack++;
        scheduler_stack[k_sched_ptr_end]=task;
        if (k_sched_ptr_end<(K_ST_SCHEDULER_STACK_SIZE - 1))
        k_sched_ptr_end++;
        else
        k_sched_ptr_end=0;
      }
      // Check stack for high load
      {
        if (scheduler_stack[k_sched_ptr_end]) // Alarm on stack full (not overrun)
          k_alarm_word |= (1<<0); // Kernel ST stack full
      }
    }
    else k_stack_full(task);
}

// RT context
void k_task_handler(void) {
    // systat
    {
      if (act_stack > act_stack_max)  {
        act_stack_max = act_stack;
      } else if (act_stack < act_stack_min) {
        act_stack_min = act_stack;
      }
    }
    // Stack works
    {
      if (!(k_status_word & (1<<0))) return; // K-ST scheduler not running
      if (!act_stack) return; // Stack empty
      act_stack--;
      if (scheduler_stack[k_sched_ptr_begin]==TASK_4ms) k_st_task_4ms();
      else if (scheduler_stack[k_sched_ptr_begin]==TASK_20ms) k_st_task_20ms();
      else if (scheduler_stack[k_sched_ptr_begin]==TASK_100ms) k_st_task_100ms();
      else if (scheduler_stack[k_sched_ptr_begin]==TASK_1s) k_st_task_1s();

      scheduler_stack[k_sched_ptr_begin]=0;

      // Circular buffer: prep next task entry
      if (k_sched_ptr_begin<(K_ST_SCHEDULER_STACK_SIZE - 1)) k_sched_ptr_begin++;
      else k_sched_ptr_begin=0;
    }
    // TODO: Add in testsscope, test all tasks ST/RT all schedule + run/stop functions.
}

// RT context
void k_stack_full(char task) {
    k_fault_word|=(1 << 0); // Kernel ST full bit 1
}

// RT context
inline void k_schedule_task_user(char task) {
    if (scheduler_stack_user[u_sched_ptr_end]==0) {
      // Load entry
      {
        act_stack_user++;
        scheduler_stack_user[u_sched_ptr_end]=task;
        if (u_sched_ptr_end<(U_ST_SCHEDULER_STACK_SIZE - 1))
        u_sched_ptr_end++;
        else
        u_sched_ptr_end=0;
      }
      // Check stack for high load:
      {
        if (scheduler_stack_user[u_sched_ptr_end]) // Alarm on stack full (not overrun)
          k_alarm_word |= (1<<4); // User ST stack full
      }
    }
    else k_stack_full_user(task);
}

// RT context
void k_task_handler_user(void) {
    // systat
    {
      if (act_stack_user > act_stack_user_max)  {
        act_stack_user_max = act_stack_user;
      } else if (act_stack_user < act_stack_user_min) {
        act_stack_user_min = act_stack_user;
      }
    }
    // Stack works
    {
      if (!(k_status_word & (1<<4))) return; // U-ST scheduler not running
      if (!act_stack_user) return; // Stack empty
      act_stack_user--;

      if (scheduler_stack_user[u_sched_ptr_begin]==TASK_4ms) k_st_task_4ms_user();
      else if (scheduler_stack_user[u_sched_ptr_begin]==TASK_20ms) k_st_task_20ms_user();
      else if (scheduler_stack_user[u_sched_ptr_begin]==TASK_100ms) k_st_task_100ms_user();
      else if (scheduler_stack_user[u_sched_ptr_begin]==TASK_1s) k_st_task_1s_user();

      scheduler_stack_user[u_sched_ptr_begin]=0;

      // Circular buffer: prep next task entry
      if (u_sched_ptr_begin<(U_ST_SCHEDULER_STACK_SIZE - 1)) u_sched_ptr_begin++;
      else u_sched_ptr_begin=0;
    }
    // TODO: Add in testsscope, test all tasks ST/RT all schedule + run/stop functions.
}

// RT context
void k_stack_full_user(char task) {
    k_fault_word|=(1<<4); // User ST full bit 1
}

// ST context
int k_task_is_scheduled(void) {
    return ((k_sched_ptr_end!=k_sched_ptr_begin) | (scheduler_stack[k_sched_ptr_begin]>0));
}

// ST context
int k_task_is_scheduled_user(void) {
    return ((u_sched_ptr_end!=u_sched_ptr_begin) | (scheduler_stack_user[u_sched_ptr_begin]>0));
}

// systat performance tools

// Load measure time with Timer registers

// Private

#define K_SYSTAT_LOAD_RT_ENTRY_INIT 0x00;
#define K_SYSTAT_LOAD_RT_EXIT_INIT 0x00;
#define K_SYSTAT_LOAD_RT_MIN_INIT 0xff;
#define K_SYSTAT_LOAD_RT_MAX_INIT 0x00;
#define K_SYSTAT_LOAD_RT_AVG_INIT 0x00;
#define K_SYSTAT_LOAD_RT_TMR_RELOAD_INIT 0xFF;
#define K_SYSTAT_LOAD_RT_TMR_ROLL_INIT 0x00;

uint8_t k_systat_load_rt_entry = K_SYSTAT_LOAD_RT_ENTRY_INIT;
uint8_t k_systat_load_rt_exit = K_SYSTAT_LOAD_RT_EXIT_INIT;
uint8_t k_systat_load_rt_min = K_SYSTAT_LOAD_RT_MIN_INIT;
uint8_t k_systat_load_rt_max = K_SYSTAT_LOAD_RT_MAX_INIT;
uint8_t k_systat_load_rt_avg = K_SYSTAT_LOAD_RT_AVG_INIT;
uint8_t k_systat_load_rt_tmr_reload_value = K_SYSTAT_LOAD_RT_TMR_RELOAD_INIT;
uint8_t k_systat_load_rt_tmr_roll = K_SYSTAT_LOAD_RT_TMR_ROLL_INIT;

// Public
void k_init() {
  // Init/Reset systat.
  k_systat_init();
  // Initialyze the Scheduler.
  {
    for (int i = 0; i < K_ST_SCHEDULER_STACK_SIZE; i++)
      scheduler_stack[i] = 0;
    for (int i = 0; i < U_ST_SCHEDULER_STACK_SIZE; i++)
      scheduler_stack_user[i] = 0;
  }
}

// Public
// k_systat init/reset values
inline void k_systat_init(void) {
  // RT
  k_systat_load_rt_entry = K_SYSTAT_LOAD_RT_ENTRY_INIT;
  k_systat_load_rt_exit = K_SYSTAT_LOAD_RT_EXIT_INIT;
  k_systat_load_rt_min = K_SYSTAT_LOAD_RT_MIN_INIT;
  k_systat_load_rt_max = K_SYSTAT_LOAD_RT_MAX_INIT;
  k_systat_load_rt_avg = K_SYSTAT_LOAD_RT_AVG_INIT;
  k_systat_load_rt_tmr_roll = K_SYSTAT_LOAD_RT_TMR_ROLL_INIT;

  // ST
  act_stack_min = K_ST_SCHEDULER_STACK_SIZE;
  act_stack_max = 0;
  act_stack_user_min = U_ST_SCHEDULER_STACK_SIZE;
  act_stack_user_max = 0;
}

// Private
// Check-in
void k_systat_load_rt_task_in() {
  uint8_t tmr = HW_TIMER_VALUE;
  k_systat_load_rt_entry = tmr;
}
// Private
// Check-out
void k_systat_load_rt_task_out() {
  uint8_t tmr = HW_TIMER_VALUE;
  k_systat_load_rt_exit = tmr;
}

// Public
void k_systat() {
  // Update monitoring values
  k_systat_load_rt_tmr_monitor();
}

// Public
void k_idle() {
  // Update monitoring values
  k_systat_load_rt_tmr_monitor();
  // Run additional custom code
  // ...
}

// Private
void k_systat_load_rt_tmr_monitor() {
  // Monitor timer min/max values (available RT MCU cycles)
  uint8_t tmr = HW_TIMER_VALUE;
  if (k_systat_load_rt_tmr_roll < tmr)
    k_systat_load_rt_tmr_roll = tmr;
  if (k_systat_load_rt_tmr_reload_value > tmr)
    k_systat_load_rt_tmr_reload_value = tmr;
}
// Private
void k_systat_load_rt_task_calc() {
  // Load locally to prevent timer Update while calc.
  // We may still have an INT between the 2 loads hereunder
  // More rubust solution will need a double buffer (TODO).

  uint8_t loc_k_systat_load_rt_exit = k_systat_load_rt_exit;
  uint8_t loc_k_systat_load_rt_entry = k_systat_load_rt_entry;

  // TODO : Calculation with Roll-over correction
  if (loc_k_systat_load_rt_exit < loc_k_systat_load_rt_entry)
    return;

  uint8_t k_systat_load_rt_delta;
  k_systat_load_rt_delta = loc_k_systat_load_rt_exit - loc_k_systat_load_rt_entry;

  // Determine and update min/max runtime
  // uint8_t k_systat_load_rt_delta = loc_k_systat_load_rt_exit - loc_k_systat_load_rt_entry;
  // Stats
  if (k_systat_load_rt_min > k_systat_load_rt_delta)
    k_systat_load_rt_min = k_systat_load_rt_delta;
  if (k_systat_load_rt_max < k_systat_load_rt_delta)
    k_systat_load_rt_max = k_systat_load_rt_delta;

  // Average calculation
  // Mix (1 new + 15 previous) / 16
  /*
  uint16_t A = (k_systat_load_rt_avg >> 4);
  uint16_t B = (k_systat_load_rt_delta - (k_systat_load_rt_delta >> 4));
  uint16_t R = A + B;
  k_systat_load_rt_avg = R;
  */
  k_systat_load_rt_avg = (k_systat_load_rt_avg >> 4) + \
    (k_systat_load_rt_delta - (k_systat_load_rt_delta >> 4));
}

/*
* Status (ST) word : k_status_word
*   b7: A Fault for User space exists
*   b6: An Alarm for User space exists
*   b5: Provision (reads 0)
*   b4: U-ST (User Space scheduler) running
*   b3: A Fault for Kernel exists
*   b2: An Alarm for Kernel exists
*   b1: Provision (reads 0)
*   b0: K-ST (Kernel Space scheduler) running
*/

inline void k_systat_update_status() {
  // Self reset each cycle except
  // Running bits which control K-RT.
  k_status_word &= ((1<<4)|(1<<0)); // Keep current running state
  // Set Status bits

  // User space common Fault
  if (k_fault_word & (1<<4))
    k_status_word |= (1<<7);

  // User space common Alarm
  if (k_alarm_word & (1<<4))
    k_status_word |= (1<<6);

  // Kernel common Fault
  if (k_fault_word & ((1<<3) | (1<<0)))
    k_status_word |= (1<<3);

  // Kernel common Alarm
  if (k_alarm_word & (1<<0))
    k_status_word |= (1<<2);
}

// Serial input/outputs

// printf to serial output
char k_printf(const char *ci , ...) {
  // TODO
  // Optimal MCU implementation
  return 0;
}

// Public
uint8_t k_systat_load_rt_task_min() {
  return k_systat_load_rt_min;
}
// Public
uint8_t k_systat_load_rt_task_max() {
  return k_systat_load_rt_max;
}
// Public
// Load ref to rollover
uint8_t k_systat_load_rt_task_avg() {
  return k_systat_load_rt_avg;
}
// Public
uint8_t k_systat_load_rt_tmr_reload() {
  return k_systat_load_rt_tmr_reload_value;
}

uint8_t k_systat_load_rt_tmr_rollover() {
  return k_systat_load_rt_tmr_roll;
}

// Kernel scheduler load min/max
uint8_t k_systat_load_st_task_min() {
  return act_stack_min;
}
uint8_t k_systat_load_st_task_max() {
  return act_stack_max;
}

// User space scheduler load min/max
uint8_t u_systat_load_st_task_min() {
  return act_stack_user_min;
}
uint8_t u_systat_load_st_task_max() {
  return act_stack_user_max;
}

// Returns stack size for K-ST tasks storage (Kernel stats)
uint8_t k_systat_load_st_task_avail() {
  return K_ST_SCHEDULER_STACK_SIZE;
}

// Returns stack size for U-ST tasks storage (User space stats)
uint8_t u_systat_load_st_task_avail() {
  return U_ST_SCHEDULER_STACK_SIZE;
}
