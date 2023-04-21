/**************************************************************************
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
#define __MAIN_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdio.h>
#include "tal.h"
#include "terminal.h"
#include "dhrystone.h"
#include "memtest.h"
#include "xmempool.h"

void _main (void);
#define main_core()  _main()

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*
 * Some TASK variables like stack and task control block.
 */
static OS_STACK (StartStack,  TASK_START_STK_SIZE);
static OS_STACK (LEDStack,    TASK_LED_STK_SIZE);

static OS_STACK (AStack,      TASK_A_STK_SIZE);
static OS_STACK (BStack,      TASK_B_STK_SIZE);
static OS_STACK (CStack,      TASK_C_STK_SIZE);

static OS_TCB TCBStartTask;
static OS_TCB TCBLed;

static OS_TCB TCBA;
static OS_TCB TCBB;
static OS_TCB TCBC;

static uint8_t bABCTestRunning = 0;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

#if !defined(__SDRAM__)
/*************************************************************************/
/*  MemTest                                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: 0 = OK / -1 = Error                                          */
/*************************************************************************/
int MemTest (void)
{
   /*
    * The data bus width must be set in memtest.h,
    * datum must be set.
    */
   #define BASE_ADDRESS    (volatile datum *) 0x90000000
   #define NUM_BYTES       (32 * 1024 * 1024)


   if ( (memTestDataBus(BASE_ADDRESS) != 0)                  ||
        (memTestAddressBus(BASE_ADDRESS, NUM_BYTES) != NULL) ||
        (memTestDevice(BASE_ADDRESS, NUM_BYTES) != NULL)     )
   {
      return (-1);
   }
   else
   {
      return (0);
   }

} /* MemTest() */
#endif /* !defined(__SDRAM__) */

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

   term_printf("%s", ResetScreen);
   OS_TimeDly(50);

   term_printf("\r\n");
   term_printf("*********************************\r\n");
   term_printf("  Project: %s\r\n", PROJECT_NAME);
   term_printf("  Board  : %s\r\n", TAL_BOARD);
   term_printf("  Version: v%s\r\n", PROJECT_VER_STRING);
   term_printf("  Build  : "__DATE__ " " __TIME__"\r\n");
   term_printf("*********************************\r\n");
   term_printf("\r\n");

} /* OutputBootMessage */

/*************************************************************************/
/*  OutputVersionInfo                                                    */
/*                                                                       */
/*  Output version information.                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputVersionInfo (void)
{
   uint32_t version;
   uint8_t  v1, v2, v3, v4;

   version = neorv32_cpu_csr_read(CSR_MIMPID);
   v4 = (uint8_t)(version & 0xFF);
   version >>= 8;
   v3 = (uint8_t)(version & 0xFF);
   version >>= 8;
   v2 = (uint8_t)(version & 0xFF);
   version >>= 8;
   v1 = (uint8_t)(version & 0xFF);

   term_printf("*** Version ***\r\n");
   term_printf("Board: %s\r\n", TAL_BOARD);
   if (v4 != 0)
   {
      term_printf("CPU  : %s v%d.%d.%d.%d\r\n", TAL_CPU, v1, v2, v3, v4);
   }
   else
   {
      term_printf("CPU  : %s v%d.%d.%d\r\n", TAL_CPU, v1, v2, v3);
   }
   term_printf("OS   : %s v%s\r\n", OS_NAME, OS_VER_STRING);
   term_printf("TAL  : v%s\r\n", TAL_CORE_VER_STRING);
   term_printf("\r\n");

} /* OutputVersionInfo */

/*************************************************************************/
/*  OutputFrequencyInfo                                                  */
/*                                                                       */
/*  Output frequency information.                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputFrequencyInfo (void)
{
   term_printf("*** Frequency info ***\r\n");
   term_printf("CPU: %3d MHz\r\n", tal_CPUGetFrequencyCPU() / 1000000);
   term_printf("\r\n");

} /* OutputFrequencyInfo */

/*************************************************************************/
/*  OutputCPULoad                                                        */
/*                                                                       */
/*  Output load information.                                             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OutputCPULoad (void)
{
   term_printf("CPU load: %d%%\r\n\r\n", OS_StatGetCPULoad());

} /* OutputCPULoad */

/*************************************************************************/
/*  OutputUsageInfo                                                      */
/*                                                                       */
/*  Output usage information.                                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputUsageInfo (void)
{
   term_printf("*** Usage ***\r\n");
   term_printf("h: output this info\r\n");
   term_printf("?: output this info\r\n");
   term_printf("c: output cpu load info\r\n");
   term_printf("f: output frequency info\r\n");
   term_printf("r: output runtime stack info\r\n");
   term_printf("t: output task info\r\n");
   term_printf("m: output memory info\r\n");
   term_printf("v: output version info\r\n");
   term_printf("x: reboot\r\n");
   term_printf("\r\n");
   term_printf("a: start round robin ABC task wait test\r\n");
   term_printf("A: stop round robin ABC task wait test\r\n");
   term_printf("\r\n");

} /* OutputUsageInfo */

/*************************************************************************/
/*  ATask                                                                */
/*                                                                       */
/*  This is the A task.                                                  */
/*                                                                       */
/*  In    : task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void ATask (void *p)
{
   (void)p;

   while (1 == bABCTestRunning)
   {
      term_putchar('A');
      OS_TimeDly(1000);
   }

   OS_TaskExit();
} /* ATask */

/*************************************************************************/
/*  BTask                                                                */
/*                                                                       */
/*  This is the B task.                                                  */
/*                                                                       */
/*  In    : task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void BTask (void *p)
{
   (void)p;

   while (1 == bABCTestRunning)
   {
      term_putchar('B');
      OS_TimeDly(1000);
   }

   OS_TaskExit();
} /* BTask */

/*************************************************************************/
/*  CTask                                                                */
/*                                                                       */
/*  This is the C task.                                                  */
/*                                                                       */
/*  In    : task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void CTask (void *p)
{
   (void)p;

   while (1 == bABCTestRunning)
   {
      term_putchar('C');
      OS_TimeDly(1000);
   }

   OS_TaskExit();
} /* CTask */

/*************************************************************************/
/*  LEDTask                                                              */
/*                                                                       */
/*  This is the LED task.                                                */
/*                                                                       */
/*  In    : task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void LEDTask (void *p)
{
   (void)p;

   while(1)
   {
      tal_LEDToggle(TAL_LED_CHANNEL_8);
      OS_TimeDly(TASK_LED_DELAY_MS);
   }

} /* LEDTask */

/*************************************************************************/
/*  StartTask                                                            */
/*                                                                       */
/*  This is the Start task.                                              */
/*                                                                       */
/*  In    : task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void StartTask (void *p)
{
   (void)p;

   /*
    * The StartTask will be used to start all other tasks in the system.
    * At the end the priority will be set to "IDLE" priority.
    */

   OS_SysTickStart();   /* Start the System ticker */
   OS_StatEnable();     /* Enable the statistic function  */

   /*******************************************************************/

   term_Start();        /* Start the Terminal functionality */

   /*
    * Output startup messages
    */
   OutputBootMessage();
   OutputFrequencyInfo();
   OutputUsageInfo();

   /* Create the LED task */
   OS_TaskCreate(&TCBLed, LEDTask, NULL, TASK_LED_PRIORITY,
                 LEDStack, sizeof(LEDStack),
                 "LEDTask");


   /*
    * Dhrystone and CoreMark test only runs in the SDRAM configuration
    * because there is not enough memory available in the TCM configuration.
    */
#if defined(__SDRAM__)

#if 0
   term_printf("CoreMark Benchmark started ...\r\n\r\n");
   OS_TimeDly(500);
   main_core();   /* Start CoreMark */
#endif

#if 0
   OS_TimeDly(500);
   main_dhry();   /* Start Dhrystone */
#endif

#endif


   /*
    * The memory test should only used if the SDRAM is
    * not used by the program.
    *
    * !!! The memory test destroys the data structures
    * that are required for the XMEM functionality !!!
    */
#if 0
#if !defined(__SDRAM__)
   term_printf("Memory test... ");
   OS_TimeDly(100);
   if (0 == MemTest())
   {
      term_printf("OK\r\n");
   }
   else
   {
      term_printf("ERROR\r\n");
   }
#endif
#endif


   OS_TaskChangePriority(TASK_START_PRIORITY_IDLE);

   while (1)
   {
      OS_TimeDly(TASK_START_DELAY_MS);
   }

} /* StartTask */

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
   /************************* "Debugger test" *************************/

   uint32_t dStart = 0;
   uint32_t dEnd   = 0;
   uint32_t dSize  = 0;
   dStart = 0x90000000;
   dEnd   = 0x92000000;
   dSize  = (dEnd - dStart) - 1;

   __asm__ volatile ("nop");

   /*******************************************************************/

   /*
    * Init the "Tiny Abstraction Layer"
    */
   tal_Init();

   /*
    * Initialize the memory pool
    */
   xmem_Init();

   /*
    * Create the StartTask.
    * The StartTask is the one and only task which
    * will start all other tasks of the system.
    */
   OS_TaskCreate(&TCBStartTask, StartTask, NULL, TASK_START_PRIORITY,
                 StartStack, sizeof(StartStack),
                 "StartTask");

   /*
    * OSStart must be the last function here.
    *
    * Fasten your seatbelt, engine will be started...
    */
   OS_Start();

   /*
    * This return here make no sense.
    * But to prevent the compiler warning:
    *    "return type of 'main' is not 'int'
    * We use an int as return :-)
    */
   return(dSize); /*lint !e527*/
} /* main */

/*************************************************************************/
/*  term_RxCallback                                                      */
/*                                                                       */
/*  Will be called from TermTask in case a char is received.             */
/*                                                                       */
/*  In    : bData                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void term_RxCallback (uint8_t bData)
{
   switch (bData)
   {
      case 'h':
      case '?':
      {
         OutputUsageInfo();
         break;
      }

      case 'c':
      {
         OutputCPULoad();
         break;
      }

      case 'f':
      {
         OutputFrequencyInfo();
         break;
      }

      case 'r':
      {
         OS_OutputRuntimeStackInfo();
         break;
      }

      case 't':
      {
         OS_OutputTaskInfo();
         break;
      }

      case 'm':
      {
         tal_MEMOutputMemoryInfo();
         break;
      }

      case 'v':
      {
         OutputVersionInfo();
         break;
      }

      case 'x':
      {
         tal_CPUReboot();
         break;
      }

      /****************************************************/

      case 'a':
      {
         if (0 == bABCTestRunning)
         {
            bABCTestRunning = 1;

            /* Create the A,B and C task */
            OS_TaskCreate(&TCBA, ATask, NULL, TASK_A_PRIORITY,
                          AStack, sizeof(AStack), "ATask");

            OS_TaskCreate(&TCBB, BTask, NULL, TASK_B_PRIORITY,
                          BStack, sizeof(BStack), "BTask");

            OS_TaskCreate(&TCBC, CTask, NULL, TASK_C_PRIORITY,
                          CStack, sizeof(CStack), "CTask");
         }
         break;
      }

      case 'A':
      {
         if (1 == bABCTestRunning)
         {
            bABCTestRunning = 0;
            OS_TimeDly(1000);
            term_printf("\r\n\r\n");
         }
         break;
      }

      default:
      {
         /* Do nothing */
         break;
      }
   } /* end switch (bData) */

} /* term_RxCallback */

/*** EOF ***/
