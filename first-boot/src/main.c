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
*
***************************************************************************
*  History:
*
*  21.08.2017  mifi  First Version.
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

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define PRINT_TEXT(...) neorv32_uart0_puts(__VA_ARGS__)
#define PRINT_XNUM(a)   print_hex_word(a)
#define PRINT_GETC(a)   neorv32_uart0_getc()
#define PRINT_PUTC(a)   neorv32_uart0_putc(a)

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/**********************************************************************//**
 * Print 32-bit number as 8-digit hexadecimal value (with "0x" suffix).
 *
 * @param[in] num Number to print as hexadecimal.
 **************************************************************************/
void print_hex_word (uint32_t num)
{
   static const char hex_symbols[16] = "0123456789abcdef";

   PRINT_PUTC('0');
   PRINT_PUTC('x');

   for (int i=28; i>=0; i-=4)
   {
      PRINT_PUTC(hex_symbols[(num >> i) & 0xf]);
   }
} /* print_hex_word */

/*************************************************************************/
/*  OutputBootMessage                                                    */
/*                                                                       */
/*  Output boot message.                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputBootMessage (void)
{
   const char ResetScreen[] = { 0x1B, 'c', 0 };

   PRINT_TEXT(ResetScreen);
   OSTimeDly(100);

   PRINT_TEXT("\r\n");
   PRINT_TEXT("*********************************\r\n");
   PRINT_TEXT("  Project: " PROJECT_NAME "\r\n");
   PRINT_TEXT("  Board  : " PROJECT_BOARD " \r\n");
   PRINT_TEXT("  Version: v" XSTR(PROJECT_VER_MAJOR) "." XSTR(PROJECT_VER_MINOR_1) XSTR(PROJECT_VER_MINOR_2) "\r\n");
   PRINT_TEXT("  Build  : "__DATE__ " " __TIME__"\r\n");
   PRINT_TEXT("*********************************\r\n");
   PRINT_TEXT("\r\n");

   /* Warning if i-cache is not implemented */
   if (0 == (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_ICACHE)))
   {
      PRINT_TEXT("WARNING! No instruction cache implemented!\r\n");
      PRINT_TEXT("The XIP program might run very slow...\r\n");
      PRINT_TEXT("\r\n");
      OSTimeDly(2000);
   }

} /* OutputBootMessage */

/*************************************************************************/
/*  OutputsystemInfo                                                     */
/*                                                                       */
/*  Output system info.                                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputsystemInfo (void)
{
  PRINT_TEXT("HWV:  ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MIMPID));
  PRINT_TEXT("\nCLK:  ");
  PRINT_XNUM(NEORV32_SYSINFO->CLK);
  PRINT_TEXT("\nMISA: ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MISA));
  PRINT_TEXT("\nXISA: ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MXISA));
  PRINT_TEXT("\nSOC:  ");
  PRINT_XNUM(NEORV32_SYSINFO->SOC);

  PRINT_TEXT("\nIMEM: ");
  PRINT_XNUM((uint32_t)(1 << NEORV32_SYSINFO->MEM[SYSINFO_MEM_IMEM]) & 0xFFFFFFFCUL);
  PRINT_TEXT(" bytes @ ");
  PRINT_XNUM(0x00000000);
  
  PRINT_TEXT("\nDMEM: ");
  PRINT_XNUM((uint32_t)(1 << NEORV32_SYSINFO->MEM[SYSINFO_MEM_DMEM]) & 0xFFFFFFFCUL);
  PRINT_TEXT(" bytes @");
  PRINT_XNUM(0x80000000);

  PRINT_TEXT("\n\n");

} /* OutputsystemInfo */

/*************************************************************************/
/*  GetInfoRequest                                                       */
/*                                                                       */
/*  Check if KEY[0] was pressed.                                         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TRUE / FALSE                                                 */
/*************************************************************************/
static uint8_t GetInfoRequest (void)
{
   uint8_t  bRequest = FALSE;

   /* Check if KEY0 is pressed */
   if (neorv32_gpio_pin_get(0))
   {
      bRequest = TRUE;
   }

   return(bRequest);
} /* GetInfoRequest */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  main                                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
int main (void)
{
   uint32_t dStartAddr;

   /*
    * Initialize system, com and terminal functionality
    */
   SystemBootInit();

   /* Output startup messages */
   OutputBootMessage();

   /* Check if a info request is available */
   if (TRUE == GetInfoRequest())
   {
      OutputsystemInfo();

      while (GetInfoRequest())
      {
         __asm__ volatile ("nop");
      }
   }

   /* Check if an image is available */
   dStartAddr = FlashIsImageAvailable(FLASH_BOOT_START_ADDR);

   /* Check if an image can be started */
   if (0 != dStartAddr)
   {
      FlashStartImage(dStartAddr);
   }
   else
   {
      OutputsystemInfo();

      PRINT_TEXT("Error: 2nd Stage Bootloader not found.\r\n");

      while (1)
      {
         neorv32_gpio_port_set(0);
         OSTimeDly(100);
         neorv32_gpio_port_set(1);
         OSTimeDly(100);
      }
   }


   /*
    * The next code should never be executed
    */
   while (1)
   {
      neorv32_gpio_port_set(0);
      OSTimeDly(100);
      neorv32_gpio_port_set(1);
      OSTimeDly(100);
   }


  /*
   * This return here make no sense.
   * But to prevent the compiler warning:
   * "return type of 'main' is not 'int'
   * we use an int as return :-)
   */
  return(0);
} /* main */

/*** EOF ***/
