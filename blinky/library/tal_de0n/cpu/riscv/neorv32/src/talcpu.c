/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2023 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
**************************************************************************/
#define __TALCPU_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdlib.h>
#include "tal.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static uint32_t dHiResPeriod = 0;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  SysTick_Handler                                                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void SysTick_Handler (void)
{
   TAL_CPU_IRQ_ENTER();

   /* clear/ack pending FIRQ */
   neorv32_cpu_csr_write(CSR_MIP, ~(1<<GPTMR_FIRQ_PENDING));

   OS_TimerCallback();

   TAL_CPU_IRQ_EXIT();
} /* SysTick_Handler */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  tal_CPUInit                                                          */
/*                                                                       */
/*  "Initialize" the CPU.                                                */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUInit (void)
{
   int             rc = 0;
   static uint8_t bInitDone = 0;

   if (0 == bInitDone)
   {
      bInitDone = 1;

      /*
       * Capture all exceptions and give debug info via UART
       * this is not required, but keeps us safe.
       */
      neorv32_rte_setup();

      /* Enable and configure primary UART (UART0). */
      neorv32_uart_setup(NEORV32_UART0, TERM_COM_SPEED, 0);

      /* Check if GPIO unit is implemented at all */
      if (0 == neorv32_gpio_available())
      {
         rc = -1;
         neorv32_uart_puts(NEORV32_UART0, "\r\nError! No GPIO unit synthesized!\r\n");
      }

      /* Check if GPTMR unit is implemented at all */
      if (0 == neorv32_gptmr_available())
      {
         rc = -1;
         neorv32_uart_puts(NEORV32_UART0, "\r\nERROR! General purpose timer not implemented!\r\n");
      }

      if (rc != 0)
      {
         /* Error */
         while (1)
         {
            __asm__ volatile ("nop");
         }
      }
   }

} /* tal_CPUInit */

/*************************************************************************/
/*  tal_CPUSysTickStart                                                  */
/*                                                                       */
/*  Start the SysTick.                                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUSysTickStart (void)
{
   uint32_t ticks;

   /* Install GPTMR interrupt handler */
   neorv32_rte_handler_install(GPTMR_RTE_ID, SysTick_Handler);

   ticks = ((NEORV32_SYSINFO->CLK / 2) / OS_TICKS_PER_SECOND);

   /*
    * The divider for the CPU was chosen so that
    * the ticks are no larger than 16 bits.
    */
   dHiResPeriod = ticks;

   /* Configure timer for 1000Hz ticks in continuous mode (with clock divisor = 2) */
   neorv32_gptmr_setup(CLK_PRSC_2, 1, ticks - 1);

   /*
    * Enable interrupt
    */

   /* Enable GPTMR FIRQ channel */
   neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);

   /* Enable machine-mode interrupts */
   neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

} /* tal_CPUSysTickStart */

/*************************************************************************/
/*  tal_CPUStatGetHiResPeriod                                            */
/*                                                                       */
/*  Return the HiResPeriod value.                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: HiResPeriod                                                  */
/*************************************************************************/
uint32_t tal_CPUStatGetHiResPeriod (void)
{
   return(dHiResPeriod);
} /* tal_CPUStatGetHiResPeriod */

/*************************************************************************/
/*  tal_CPUStatGetHiResCnt                                               */
/*                                                                       */
/*  Return the HiRes counter.                                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: HiRes counter                                                */
/*************************************************************************/
uint32_t tal_CPUStatGetHiResCnt (void)
{
   uint32_t dValue;

   /* Get milliseconds */
   dValue = (OS_TimeGet() << 16);

   /*
    * The COUNT register of the GPTMR is incrementing up to the threshold
    * register (THRES). And the threshold was chosen to fit in a 16 bit
    * register. Therefore the count register will fit in 16 bit too.
    */
   dValue |= (uint16_t)(NEORV32_GPTMR->COUNT & 0x0000FFFF);

   return(dValue);
} /* tal_CPUStatGetHiResCnt */

/*************************************************************************/
/*  tal_CPUGetFrequencyCPU                                               */
/*                                                                       */
/*  Return the clock frequency of the CPU in MHz.                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Frequency                                                    */
/*************************************************************************/
uint32_t tal_CPUGetFrequencyCPU (void)
{
   return(NEORV32_SYSINFO->CLK);
} /* tal_CPUGetFrequencyCPU */

/*************************************************************************/
/*  tal_CPUInitHWDog                                                     */
/*                                                                       */
/*  Initialize the Hardware Watchdog.                                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUInitHWDog (void)
{
   uint32_t       dTimeoutSec;
   static uint8_t bInitDone = 0;

   if ((0 == bInitDone) && (neorv32_wdt_available() != 0))
   {
      bInitDone = 1;

      dTimeoutSec = 1 * (NEORV32_SYSINFO->CLK / 4096);
      neorv32_wdt_setup(dTimeoutSec, 0, 0, 1, 1);
   }

} /* tal_CPUInitHWDog */

/*************************************************************************/
/*  Name  : tal_CPUTriggerHWDog                                          */
/*                                                                       */
/*  Trigger the Hardware Watchdog here.                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUTriggerHWDog (void)
{
   // TODO...
} /* tal_CPUTriggerHWDog */


/*************************************************************************/
/*  tal_CPUReboot                                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUReboot (void)
{
#if defined(__DEBUG__)
   term_printf("\r\n*** Reboot ***\r\n");
   OS_TimeDly(500);
#endif

   /*
    * Init watchdog, if not done before
    */
   tal_CPUInitHWDog();

   /*
    * Wait for watchdog reset
    */
   TAL_CPU_DISABLE_ALL_INTS();
   while (1)
   {
      __asm__ volatile ("nop");
   }
   TAL_CPU_ENABLE_ALL_INTS(); /*lint !e527*/

} /* tal_CPUReboot */

/*** EOF ***/
