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

// Main Sample Code:

/* We need to call scheduler in main loop.
int main() {
   // Init ...
   while(1)
   {
       // Scheduler call
       while (k_task_is_scheduled() || k_task_is_scheduled_user()) {
           if (k_task_is_scheduled()) k_task_handler();
           if (k_task_is_scheduled_user()) k_task_handler_user();
       }
       // Idle, collecting stats
       k_idle();
   }
}
*/

/*
  Port 16F628 LEDs:

  Led RT 4ms	RA0  g0ci g0co
  Led ST 1s	  RA1 g1ci g1co
  Led Alm/Flt	RA2 g2ci g2co
  Led G1		  RA3 g3ci g3co
  Led G2		  RA4 g4ci g4co
  Led G3		  RB0 g8ci g8co
  Led G4		  RB3 g11ci g11co
  Led G5		  RB4 g12ci g12co
  Led Y6		  RB5 g13ci g13co
  g0co g1co g2co g3co g4co g8co g11co g12co g13co
  g0ci g1ci g2ci g3ci g4ci g8ci g11ci g12ci g13ci
*/


/*
 * User functions
 * Periodic functions, RT (interrupt) and scheduled contexts (ST).
 */

 #ifndef K_RT_USER_H
 #define K_RT_USER_H

 #ifdef	__cplusplus
 extern "C" {
 #endif

/* User Configuration */

// #include include Hardware Definitions as needed eg Timer definition
// #include <xc.h> // Ex. for Microchip XC8
#include <xc.h> // Hardware definition
#include <stdint.h>

// Hardware depending configuration

#define HW_TIMER_VALUE TMR0
// #define MAIN_CLOCK_FREQUENCY 10000000 // Value in Hz    TODO : Add config file.

// Scheduled Time stacks size.

#define K_ST_SCHEDULER_STACK_SIZE 16 // Depending on User application
#define U_ST_SCHEDULER_STACK_SIZE 16 // Depending on User application

    /*
     * 1/ Real time tasks
     *
     * RT tasks run in interrupt context.
     *
     * They must:
     * - Execute in much less than 4ms independently from the period they
     *   are run at (4, 20, 100ms or 1s).
     * - Be safe to execute in interrupt context.
     *
     * Tasks which can't promise those requirements have to be placed in
     * the "Scheduled" context.
     *
     *
     * 2/ Scheduled tasks
     *
     * Scheduled tasks are run in best effort context.
     *
     * The tasks 4ms, 20ms, 100ms and 1s are called once per period. They must
     * each execute within their associated period.
     *
     * If the system is overloaded and cannot push a new stack for execution,
     * it dropps new ones until the stack has an available slot.
     *
     * Whenever the Scheduler cannot assign an execution slot, it will raise the
     * fault flag. User can view, monitor and assert the max stack depth reached
     * and the fault flag to ensure the application is properly timed.
     *
     * If a task is taking too much time to run, it can be placed in the next
     * larger execution period 4 -> 20 -> 100ms -> 1s tasks.
     *
     * Functions that cannot accept those constraints must promise to execute
     * in the period and be placed in RT context.
     *
     */

     // Public

     void k_init(void);                         // Public Initializer
     inline void k_rt_task_4ms_isr_entry();     // Real time 4 ms - Interrupt context - Kernel interrupt entry point

     inline void k_rt_task_4ms_user(void);      // Real time 4 ms - Interrupt context
     inline void k_rt_task_20ms_user(void);     // Real time 20 ms period function - Interrupt context
     inline void k_rt_task_100ms_user(void);    // Real time 100 ms period function - Interrupt context
     inline void k_rt_task_1s_user(void);       // Real time 1s - Interrupt context

     void k_st_task_4ms_user(void);             // Scheduled 4 ms period function - Via scheduler
     void k_st_task_20ms_user(void);            // Scheduled 20 ms period function - Via scheduler
     void k_st_task_100ms_user(void);           // Scheduled 100 ms period function - Via scheduler
     void k_st_task_1s_user(void);              // Scheduled 1 s - Via scheduler

     int k_task_is_scheduled(void);
     int k_task_is_scheduled_user(void);
     void k_task_handler(void);         // Handles task stack
     void k_task_handler_user(void);    // Handles task stack

     // RT monitor
     // Real time tasks have to run within 4ms including
     // all RT context functions calls.
     // We monitor all RT tasks (user and kernel with all
     // periods) as one single RT context execution thread.
     // This is due to the fact RT functions are run alltogether
     // within the same hardware timer interrupt.
     uint8_t k_systat_load_rt_task_min();     // Returns current systat RT task min HW Timer count
     uint8_t k_systat_load_rt_task_max();     // Returns current systat RT task max HW Timer count
     uint8_t k_systat_load_rt_task_avg();     // Returns current systat RT task avg HW Timer count
     uint8_t k_systat_load_rt_tmr_reload();   // Returns current systat RT task HW Timer reload count
     uint8_t k_systat_load_rt_tmr_rollover(); // Returns current systat RT task HW Timer rollover count

     // ST monitor
     // Kernel scheduler load min/max
     uint8_t k_systat_load_st_task_min();     // Returns current systat K-ST stack min (Kernel stats)
     uint8_t k_systat_load_st_task_max();     // Returns current systat K-ST stack max (Kernel stats)
     uint8_t k_systat_load_st_task_avail();   // Returns stack size for K-ST tasks storage (Kernel stats)

     // User space scheduler load min/max
     uint8_t u_systat_load_st_task_min();     // Returns current systat U-ST stack min (User space stats)
     uint8_t u_systat_load_st_task_max();     // Returns current systat U-ST stack max (User space stats)
     uint8_t u_systat_load_st_task_avail();   // Returns stack size for U-ST tasks storage (User space stats)

     // Reset k_systat values
     inline void k_systat_init(void);

     // Displays above values
     inline void k_systat(void);


     // Idle task(s)
     void k_idle(void);   // Update monitoring values and if needed custom code
     void k_systat(void); // Update monitoring values

     // Fault, Alarm, Status, Control
     inline char k_fault();          // Returns fault word
     inline void k_fault_clr();      // Clear fault word
     inline char k_alarm();          // Returns alarm word
     inline void k_alarm_clr();      // Clear alarm word
     inline char k_status();         // Returns status word
     inline char k_command(char);    // Sends command to K-RT
     inline char k_command_user(char); // Custom implementation

     /*
     * Fault and Alarm words record the value (they do not self-clear).
     * To clear the fault and alarm words, use accessors (_clr).
     * Status is an actual value.
     */

     // Serial communications

     // Kernel read/write

     char k_getchar(void);       // Reads single char from serial input
     char k_putchar(char);       // Sends single char through serial input
     char k_printf(const char * , ...); // printf to serial output

     // Read buffer
     #define RX_BUF_SIZE 8
     char rx_buffer[RX_BUF_SIZE];
     int rx_content = 0;

     // char k_getchar_read_buff(); // Directly in k_getchar(), ok same file
     char k_getchar_write_buff(char);

     // Write buffer
     #define TX_BUF_SIZE 16
     char tx_buffer[TX_BUF_SIZE];
     int tx_content = 0;

     char k_putchar_read_buff();
     char k_putchar_write_buff(char);

     // Remote shell
     // Via SP software serial com

     // Definitions (common with SP)

     // The MCU Manufacturer and type as defined in KRT/MSPLC specification
     #define HW_MCU_MFR   0x30 // Microchip
     #define HW_MCU_TYPE  0x09 // PIC16F628/628A

     /*
     * All M* definitions: See correspondance table.
     * Related functions:
     * Command bits k_rt_command() and Server::decode_response()
     */

     // M1...M64 : Kernel base commands
     #define CMD_RET_ST_AL_FLT_WORDS        64	// Return Status, Alarm and Fault words
     #define CMD_RUN_U_ST				    32	// Run U - ST(User Space scheduler)
     #define CMD_STOP_U_ST				    16	// Stop U - ST(User Space scheduler)
     #define CMD_CLR_ST_AL_FLT_WORDS        8	// Clear Fault and Alarm Words
     #define CMD_RET_SYSTAT_RESET		    4	// Read - back systat then reset min / max
     #define CMD_RUN_K_ST				    2	// Run K - ST(Kernel Space scheduler)
     #define CMD_STOP_K_ST				    1	// Stop K - ST(Kernel Space scheduler)

     // M8...M134 : General commands

     // GPIO Configuration
     // Message number M8 ... M31

     #define CMD_GEN_GNCI  				  128	// g[n]ci	Set GPIO [n] as digital input
     #define CMD_GEN_GNCO  				  129	// g[n]co	Set GPIO [n] as digital output
     #define CMD_GEN_GNCA  				  130	// g[n]ca	Set GPIO [n] as analog input. Server responds with PWM resolution in bits.
     #define CMD_GEN_GNCP  				  131	// g[n]cp	Set GPIO [n] as PWM output. Server responds with PWM resolution in bits.
     #define CMD_GEN_GNCR                 135	// g[n]cr	Read-back GPIO [n] Configuration

     #define CMD_GEN_GNCOD  				136	// g[n]cod	Configure GPIO [n] Open Drain (High Z)
     #define CMD_GEN_GNCPU  				137	// g[n]cpu	Configure GPIO [n] Pull Up
     #define CMD_GEN_GNCPD  				138	// g[n]cpd	Configure GPIO [n] Pull Down

     #define CMD_GEN_GNCPP  				140	// g[n]cpp	Configure GPIO [n] Push-Pull

     #define CMD_GEN_GNCSLOW				144	// g[n]cslow	Configure GPIO [n] Slew rate Slow
     #define CMD_GEN_GNCMEDI				145	// g[n]cmed		Configure GPIO [n] Slew rate Medium
     #define CMD_GEN_GNCFAST				146	// g[n]cfast	Configure GPIO [n] Slew rate Fast

     // GPIO Manipulation
     // Message number M32 ... M55
     // Multiples not yet implemented

     #define CMD_GEN_GNRD				152	// g[n]rd		  Read GPIO [n]. Returns a boolean (8 bit with boolean at LSB)
     #define CMD_GEN_GNRD8 				153	// g[n]rd8		Read GPIO [n] through [n+7]. Returns an 8 bit result LSB = [n] MSB = [n+7]
     #define CMD_GEN_GNRD16 			154	// g[n]rd16		Read GPIO [n] through [n+15]. Returns an 8 bit result LSB = [n] MSB = [n+15]
     #define CMD_GEN_GNRD32				155	// g[n]rd32		Read GPIO [n] through [n+31]. Returns an 8 bit result LSB = [n] MSB = [n+31]
     // #define CMD_GEN_GNRDM  		156	// g[n]rd[m]rd…	Read list of digital inputs ex. g1r2r4r8r reads DI 1,2,4,8. Packed bits payload received.

     #define CMD_GEN_GNRA  			  160	// g[n]ra		    Read analog input [n]. Returns N bytes depending on ADC configuration.
     // #define CMD_GEN_GNRAM		  161	// g[n]ra[m]ra…	Read list of analog inputs ex. g1a2a4a8a reads Analog channels 1,2,4,8. Packed payload received.

     #define CMD_GEN_GNWD               164	// g[n]wd[v]		Write [v] to GPIO [n]. Payload 1 byte, bits 7-1 = address, bit 0 = value
     // #define CMD_GEN_GNWDM		 	165	// g[n1]wd[v1]m[n2]wd[v2]	Write multiple GPIO : Payload of [i] GPIO to setup. Payload 1 byte, bits 7-1 = address, bit 0 = value

     #define CMD_GEN_GNWP 				168	// g[n]wp[v]		For a PWM output, writes unscaled value (with of the PWM received at configuration). Send the payload appropriately.
     // #define CMD_GEN_GNWPM           169	// g[n1]wp[v1]m[n2]wp[v2]	Set many PWM ([i]). Payload is the packed PWM configuration. Ex

     // Payload
     // Message number M176 ... M191

     #define CMD_PAYLOAD_8_BITS		  176	// 8 bit payload
     #define CMD_PAYLOAD_16_BITS		177	// 16 bit payload
     #define CMD_PAYLOAD_24_BITS		178	// 24 bit payload
     #define CMD_PAYLOAD_32_BITS		179	// 32 bit payload
     #define CMD_PAYLOAD_40_BITS		180	// 40 bit payload
     #define CMD_PAYLOAD_48_BITS		181	// 48 bit payload
     #define CMD_PAYLOAD_56_BITS		182	// 56 bit payload
     #define CMD_PAYLOAD_64_BITS		183	// 64 bit payload
     #define CMD_PAYLOAD_128_BITS		184	// 128 bit payload
     #define CMD_PAYLOAD_192_BITS		185	// 192 bit payload
     #define CMD_PAYLOAD_256_BITS		186	// 256 bit payload
     #define CMD_PAYLOAD_512_BITS		187	// 512 bit payload
     // M188...M191 are provional for Payload

     // CRC
     // Message number M192 ... M199
     // M195...M199 are provional for CRC

     #define CMD_CRC_8_BITS				  192 // 8 bit CRC
     #define CMD_CRC_16_BITS				193 // 16 bit CRC
     #define CMD_CRC_32_BITS				194 // 32 bit CRC

     // M200 ... M223: Not defined
     // ...

     #define CMD_INF_HWMCU				  224 // ihwmcu
     #define CMD_INF_PING				    225 // iping
     #define CMD_INF_RTC_RD				  226 // irtcrd
     #define CMD_INF_RTC_WR				  227 // irtcwr
     #define CMD_INF_RTC_DEV_RD			228 // irtcdevrd
     #define CMD_INF_RTC_DEV_WR			229 // irtcdevwr
     #define CMD_INF_BATT				    230 // ibatt
     #define CMD_INF_SUPPLY				  231 // isup
     #define CMD_INF_CONSO				  232 // icons
     #define CMD_MCU_SLEEP				  240 // csleep
     #define CMD_MCU_WAKE				    241 // cwake
     #define CMD_FRAME_HEADER			  248 // mhead

     #define CMD_NOP						  253 // Nop
     #define CMD_CLR_SERIAL				254 // Clear serial decoder states

     // Extended commands

     // M255 : Extended commands
     #define CMD_EXTENDED			    255	// Extension. This is followed by the
     // Payload command M56...M67 and an arbitrary Payload. User defined meaning.



#ifdef	__cplusplus
}
#endif

#endif	/* K_RT_USER_H */


// TODO
// MSP/KRT:
// Universal Sequential Read/Writes:
// Adr 0 WV 0 (write first in command)
// Adr 1 WV 1 (...)
// Adr 2 WV 2 (...)
// Not dependant of Endianness which are HW-Sender/Receiver defined.

// Register read (rr)
// rr16adr[v1:0 - 0xffff]b[v2:byte(s)]  		Read back Registrer Value at 16 bit Address[v1].Read[v2] byte(s).
// rr32adr[v1:0 - 0xffff'ffff]b[v2:byte(s)]  Read back Registrer Value at 32 bit Address[v1].Read[v2] byte(s).
// Register write (rw)
// rw16adr[v1:0 - 0xffff]b[v2:byte(s)]v[v3_array:Byte, (Byte), (Byte), (Byte), ...] + 0xfe
// rw32adr[v1:0 - 0xffff'ffff]
// v1, v3 and v3_array
// Only accept hexadecimal notation 0x...
// Example()s:
// rr160x03b0x01 // Readback Register 0x03 (Status on PIC16F628), MCU reads-back 1 byte.
// rr320x1ffb0x04 // MCU readback 4 bytes adr 0x1ff
// rw160x100b0x04v0x010x020x030x04 // Write 0x01 0x02 0x03 0x04 at adr 0x100 (16 bit adr mode)
// rw320x01000100b0x040x010x020x030x04 // Write 0x01 0x02 0x03 0x04 at adr 0x0100'0100 (32 bit adr mode)
// (rw320x0100'0100 Ideally, this notation shoud also be supported)
