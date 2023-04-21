/**************************************************************************
*  Copyright (c) 2023 by Michael Fischer (www.emb4fun.de)
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
#define __MAIN_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "project.h"
#include "spi_flash.h"
#include "adler32.h"
#include "xboot.h"
#include "neorv32.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define UART0           NEORV32_UART0

#define TERM_COM_SPEED  115200

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static volatile uint32_t dSystemTick = 0;

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
   /* clear/ack pending FIRQ */
   neorv32_cpu_csr_write(CSR_MIP, ~(1<<GPTMR_FIRQ_PENDING));

   /* Handle SysTick counter */
   dSystemTick++;

} /* SysTick_Handler */

/*************************************************************************/
/*  InitSysTick                                                          */
/*                                                                       */
/*  Init the SysTick timer.                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void InitSysTick (void)
{
   uint32_t ticks;

   /* Install GPTMR interrupt handler */
   neorv32_rte_handler_install(GPTMR_RTE_ID, SysTick_Handler);

   ticks = ((NEORV32_SYSINFO->CLK / 2) / TICKS_PER_SECOND);

   /* Configure timer for 1000Hz ticks in continuous mode (with clock divisor = 2) */
   neorv32_gptmr_setup(CLK_PRSC_2, 1, ticks - 1);

   /*
    * Enable interrupt
    */

   /* Enable GPTMR FIRQ channel */
   neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);

   /* Enable machine-mode interrupts */
   neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

} /* InitSysTick */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  OSTimeDly                                                            */
/*                                                                       */
/*  Loop for a specified number of milliseconds,                         */
/*  will not release the CPU.                                            */
/*                                                                       */
/*  In    : TimeoutMs                                                    */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OSTimeDly (uint32_t dTimeoutMs)
{
   uint32_t dTicks;
   uint32_t dTimeEnd;

   dTicks   = OS_MS_2_TICKS(dTimeoutMs);
   dTimeEnd = dSystemTick + dTicks;

   while (dSystemTick <= dTimeEnd)
   {
      __asm__ volatile ("nop");
   }

} /* OSTimeDly */

/*************************************************************************/
/*  OSTimeGet                                                            */
/*                                                                       */
/*  Return the time from the system ticker.                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Systick                                                      */
/*************************************************************************/
uint32_t OSTimeGet (void)
{
   return(dSystemTick);
} /* OSTimeGet */

/*************************************************************************/
/*  SystemInit                                                           */
/*                                                                       */
/*  Initialize the system.                                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void SystemInit (void)
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

      /* Check if XIP module is implemented at all */
      if (0 == neorv32_xip_available())
      {
         rc = -1;
         neorv32_uart_puts(NEORV32_UART0, "\r\nError! XIP module not synthesized!\r\n");
      }

      /* Check if SPI module is implemented at all */
      if (0 == neorv32_spi_available())
      {
         rc = -1;
         neorv32_uart_puts(NEORV32_UART0, "\r\nError! SPI module not synthesized!\r\n");
      }

      /* Warning if i-cache is not implemented */
      if (0 == (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_ICACHE)))
      {
         neorv32_uart_puts(NEORV32_UART0, "\r\nWARNING! No instruction cache implemented!\r\n");
         neorv32_uart_puts(NEORV32_UART0, "The XIP program might run very slow...\r\n");
      }

      /* Init SPI */
      rc = spi_Init();
      if (-1 == rc)
      {
         neorv32_uart_puts(NEORV32_UART0, "\r\nError! Wrong SPI Flash!\r\n");
      }

      /* Check for error */
      if (rc != 0)
      {
         /* Error */
         while (1)
         {
            __asm__ volatile ("nop");
         }
      }

      /* Init SysTick */
      InitSysTick();
   }

} /* SystemInit */

/*************************************************************************/
/*  COMSendBlock                                                         */
/*                                                                       */
/*  Send a block of data.                                                */
/*                                                                       */
/*  In    : pData, wSize                                                 */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void COMSendBlock (uint8_t *pData, uint16_t wSize)
{
   if ((pData != NULL) && (wSize != 0))
   {
      while (wSize > 0)
      {
         neorv32_uart_putc(UART0, *pData);
         pData++;
         wSize--;
      }
   }

} /* COMSendBlock */

/*************************************************************************/
/*  COMSendChar                                                          */
/*                                                                       */
/*  Send one character.                                                  */
/*                                                                       */
/*  In    : bData                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void COMSendChar (uint8_t bData)
{
   neorv32_uart_putc(UART0, bData);

} /* COMSendChar */

/*************************************************************************/
/*  COMGetChar                                                           */
/*                                                                       */
/*  Get one character if available.                                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: EOF / character                                              */
/*************************************************************************/
int COMGetChar (void)
{
   int nData = EOF;

   if (neorv32_uart_char_received(UART0))
   {
      nData = (int)neorv32_uart_char_received_get(UART0);
   }

   return(nData);
} /* COMGetChar */

/*************************************************************************/
/*  term_Task                                                            */
/*                                                                       */
/*  This is the terminal task.                                           */
/*                                                                       */
/*  In    : dActualTime                                                  */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void term_Task (uint32_t dActualTime)
{
   int nData;

   (void)dActualTime;

   nData = COMGetChar();
   if (nData != EOF)
   {
      term_RxCallback((uint8_t)nData);
   }

} /* term_Task */

/*** EOF ***/
