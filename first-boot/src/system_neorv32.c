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
#include "flash.h"
#include "adler32.h"
#include "xboot.h"
#include "neorv32.h"

/*=======================================================================*/
/*  All forward prototypes definitions                                   */
/*=======================================================================*/

static void SysTick_Handler (void);

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/** Flash address bytes */
#define XIP_FLASH_ABYTES   3

/*
 * SPI flash commands
 */
enum SPI_FLASH_CMD_enum {
  SPI_FLASH_CMD_PAGE_PROGRAM  = 0x02, /* Program page */
  SPI_FLASH_CMD_READ          = 0x03, /* Read data */
  SPI_FLASH_CMD_WRITE_DISABLE = 0x04, /* Disallow write access */
  SPI_FLASH_CMD_READ_STATUS   = 0x05, /* Get status register */
  SPI_FLASH_CMD_WRITE_ENABLE  = 0x06, /* Allow write access */
  SPI_FLASH_CMD_WAKE          = 0xAB, /* Wake up from sleep mode */
  SPI_FLASH_CMD_SECTOR_ERASE  = 0xD8  /* Erase complete sector */
};

#define UART0              NEORV32_UART0

#define TERM_COM_SPEED     115200

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static volatile uint32_t dSystemTick = 0;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  __reduced_trap_handler                                               */
/*                                                                       */
/*  Reduced but based on the original __neorv32_rte_core handler.        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void __attribute__((__interrupt__)) __attribute__((aligned(4))) __reduced_trap_handler(void)
{
   void (*handler_pnt)(void);
   uint32_t rte_handler;
   uint32_t rte_mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

   // find according trap handler
   switch (rte_mcause)
   {
      case TRAP_CODE_FIRQ_12:
         rte_handler = (uint32_t)&SysTick_Handler;
         break;

      default:
         rte_handler = -1;
         break;
   }

   // execute handler
   if ((int)rte_handler != -1)
   {
      handler_pnt = (void*)rte_handler;
      (*handler_pnt)();
   }

   // compute return address
   if ((((int32_t)rte_mcause) >= 0) &&       // modify pc only if not interrupt (MSB cleared)
        (rte_mcause != TRAP_CODE_I_ACCESS))  // do not try to load the instruction as this is the reason we are here already
   {
      uint32_t rte_mepc = neorv32_cpu_csr_read(CSR_MEPC);

      // get low half word of faulting instruction
      uint32_t rte_trap_inst = (uint32_t)neorv32_cpu_load_unsigned_half(rte_mepc);

      rte_mepc += 4; // default: faulting instruction is uncompressed
      if (neorv32_cpu_csr_read(CSR_MISA) & (1 << CSR_MISA_C))  // C extension implemented?
      {
         if ((rte_trap_inst & 3) != 3) // faulting instruction is compressed instruction
         {
            rte_mepc -= 2;
         }
      }

      // store new return address
      neorv32_cpu_csr_write(CSR_MEPC, rte_mepc);
   }

} /* __reduced_trap_handler */

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
/*  SystemBootInit                                                       */
/*                                                                       */
/*  Initialize the system for the bootloader                             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void SystemBootInit (void)
{
   /*
    * neorv32_rte_setup
    */

   /* Configure trap handler (bare-metal, no neorv32 rte available) */
   neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&__reduced_trap_handler));

   /* Disable all IRQ channels */
   neorv32_cpu_csr_write(CSR_MIE, 0);

   /* Clear all pending IRQs */
   neorv32_cpu_csr_write(CSR_MIP, 0);

   /*******************************************************************/

   /* Enable and configure primary UART (UART0). */
   neorv32_uart_setup(UART0, TERM_COM_SPEED, 0);

   /*  Setup XIP: clock mode 0, bursts enabled */
   if (neorv32_xip_available())
   {
      neorv32_xip_setup(CLK_PRSC_2, 0, 0, SPI_FLASH_CMD_READ);
      neorv32_xip_burst_mode_enable();
      neorv32_xip_start(XIP_FLASH_ABYTES);
   }

   /* Clear all LEDs */
   if (neorv32_gpio_available())
   {
      neorv32_gpio_port_set(0);
   }

   /* Init system ticker */
   if (neorv32_gptmr_available())
   {
      InitSysTick();
   }

} /* SystemBootInit */

/*** EOF ***/
