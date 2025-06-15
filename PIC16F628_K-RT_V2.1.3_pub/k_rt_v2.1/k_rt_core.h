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
 * System functions
 * Periodic functions, RT (interrupt) and scheduled contexts.
 */

#ifndef K_RT_CORE_H
#define K_RT_CORE_H

 #ifdef	__cplusplus
 extern "C" {
 #endif

#define COMPILATION_WARNING "Private calls. Include only k_rt_user.h"

// Public
// These 2 function definitions are part of Public interface (k_rt_user.h)
// void k_init(void);               // Public Initializer
// inline void k_rt_task_4ms(void); // Real time 4 ms - Interrupt context

// Private
inline void k_rt_task_20ms(void);  // Real time 20 ms period function - Interrupt context
inline void k_rt_task_100ms(void); // Real time 100 ms period function - Interrupt context
inline void k_rt_task_1s(void);    // Real time 1s - Interrupt context

void k_st_task_4ms(void);          // Scheduled 4 ms period function - Via scheduler
void k_st_task_20ms(void);         // Scheduled 20 ms period function - Via scheduler
void k_st_task_100ms(void);        // Scheduled 100 ms period function - Via scheduler
void k_st_task_1s(void);           // Scheduled 1 s - Via scheduler

inline void k_schedule_task(char); // call with 1 for 100 ms, 2 for 1s
void k_stack_full(char);           // Handles a full stack situation

inline void k_schedule_task_user(char); // call with 1 for 100 ms, 2 for 1s
void k_stack_full_user(char);      // Handles a full stack situation

// Systat performance tools
void k_systat_load_rt_task_in();
void k_systat_load_rt_task_out();
void k_systat_load_rt_task_calc();
void k_systat_load_rt_tmr_monitor();

// Status word
inline void k_systat_update_status();

// Public IF see user.h
//#include "k_rt_user.h"

#ifdef	__cplusplus
}
#endif

#endif // K_RT_CORE_H
