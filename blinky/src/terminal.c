/**************************************************************************
*  Copyright (c) 2013-2023 by Michael Fischer (www.emb4fun.de).
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
*  18.06.2013  mifi  First Version, tested with a STM3240G-EVAL board.
*  20.12.2013  mifi  Tested with an ELEKTOR Internet Radio (EIR).
*  26.02.2018  mifi  Tested with an i.MX RT1050-EVK board.
*  08.06.2020  mifi  Tested with an iMX RT1062 Developer’s Kit.
**************************************************************************/
#define __TERMINAL_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "tal.h"
#include "terminal.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#if !defined(PRINTF_BUFFER_SIZE)
#define PRINTF_BUFFER_SIZE    128
#endif

#define RX_RING_BUFFER_SIZE   16
#define TX_RING_BUFFER_SIZE   256

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static OS_STACK (TermStack, TASK_TERM_STK_SIZE);

static OS_TCB           TCBTermTask;

static TAL_COM_DCB      COMPort;
static TAL_COM_SETTINGS COMSettings;

static uint8_t          RxRingBuffer[RX_RING_BUFFER_SIZE];
static uint8_t          TxRingBuffer[TX_RING_BUFFER_SIZE];

static OS_SEMA          PrintfSema;
static uint8_t          PrintfBuffer[PRINTF_BUFFER_SIZE];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  TermTask                                                             */
/*                                                                       */
/*  This is the terminal task.                                           */
/*                                                                       */
/*  In    : arg                                                          */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void TermTask (void *arg)
{
   TAL_RESULT    Error;
   TAL_COM_DCB *pPort = (TAL_COM_DCB *)arg;
   uint8_t      bData;
   
   while (1)
   {
      Error = tal_COMReceiveCharWait(pPort, &bData, TASK_TERM_DELAY_MS);
      if (TAL_OK == Error)
      {
         term_RxCallback(bData);
      }
      else
      {
         if( (TAL_ERR_COM_OVERFLOW       == Error) || 
             (TAL_ERR_COM_OVERFLOW_EMPTY == Error) )
         {
            tal_COMClearOverflow(pPort);
         }             
      } /* end if (TAL_OK == Error) */ 
   } /* end while (1) */

} /* TermTask */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  term_Start                                                           */
/*                                                                       */
/*  Start the terminal task.                                             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void term_Start (void)
{
   TAL_RESULT Error;
   
   /* Init the Device Control Block */      
   tal_COMInitDCB(&COMPort, TERM_COM_PORT);
   
   /* Set Rx and Tx ring buffer */
   tal_COMSetRingBuffer(&COMPort, TAL_COM_BUFFER_RX, RxRingBuffer, sizeof(RxRingBuffer)); 
   tal_COMSetRingBuffer(&COMPort, TAL_COM_BUFFER_TX, TxRingBuffer, sizeof(TxRingBuffer)); 
      
   /* Prepare communication settings */
   COMSettings.dBaudrate = TERM_COM_SPEED;
   COMSettings.eLength   = TAL_COM_LENGTH_8;
   COMSettings.eParity   = TAL_COM_PARITY_NONE;
   COMSettings.eStop     = TAL_COM_STOP_1_0;

   /* Now, the port can be opened */   
   Error = tal_COMOpen(&COMPort, &COMSettings);
   if (TAL_OK == Error)
   {
      OS_RES_CREATE(&PrintfSema);
   
      OS_TaskCreate(&TCBTermTask, TermTask, &COMPort, TASK_TERM_PRIORITY,
                    TermStack, sizeof(TermStack), 
                    "Terminal");
   }

} /* term_Start */

/*************************************************************************/
/*  term_printf                                                          */
/*                                                                       */
/*  This is a printf like output function.                               */
/*  The function is based on a CrossWorks for ARM example.               */
/*  Use "Customizing putchar" in the help function of CrossWorks.        */
/*                                                                       */
/*  In    : fmt                                                          */
/*  Out   : none                                                         */
/*  Return: Number of characters transmitted                             */
/*************************************************************************/
int term_printf (const char *fmt, ...)
{
   int     n = 0; /* Number of characters transmitted */
   va_list ap;

   if (TAL_TRUE == tal_COMIsOpen(&COMPort))
   {
      OS_RES_LOCK(&PrintfSema);
  
      va_start(ap, fmt);
      n = vsnprintf((char*)PrintfBuffer, sizeof(PrintfBuffer), fmt, ap);
      va_end(ap);
      (void)ap;
  
      tal_COMSendBlock(&COMPort, PrintfBuffer, (uint16_t)n);
  
      OS_RES_FREE(&PrintfSema);
   }
  
   return(n);
} /* term_printf */

/*************************************************************************/
/*  term_puts                                                            */
/*                                                                       */
/*  Send a string over the UART.                                         */
/*                                                                       */
/*  In    : string                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void term_puts (char *string)
{
   if (TAL_TRUE == tal_COMIsOpen(&COMPort))
   {
      OS_RES_LOCK(&PrintfSema);
  
      tal_COMSendBlock(&COMPort, (uint8_t*)string, (uint16_t)strlen(string));
      tal_COMSendChar(&COMPort, '\n');
        
      OS_RES_FREE(&PrintfSema);
   }
} /* term_puts */

/*************************************************************************/
/*  term_putchar                                                         */
/*                                                                       */
/*  Send a character over the UART.                                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Return the char which was written or EOF                     */
/*************************************************************************/
int term_putchar (int ch)
{
   TAL_RESULT Error = TAL_ERROR;
   int        r = EOF;

   if (TAL_TRUE == tal_COMIsOpen(&COMPort))
   {
      OS_RES_LOCK(&PrintfSema);
   
      Error = tal_COMSendChar(&COMPort, (char)ch);
      if ('\n' == ch)
      {
         Error = tal_COMSendChar(&COMPort, '\r');
      }
   
      if (TAL_OK == Error)
      {
         r = ch;
      }
      else
      {
         r = EOF;
      }

      OS_RES_FREE(&PrintfSema);
   }
   
   return(r);
} /* term_putchar */

/*** EOF ***/


