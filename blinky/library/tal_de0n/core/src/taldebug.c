/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2014-2023 by Michael Fischer (www.emb4fun.de).
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
#define __TALDEBUG_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include "talos.h"
#include "terminal.h"

/*=======================================================================*/
/*  All extern data                                                      */
/*=======================================================================*/

//extern int vsnprintf(char *__s, size_t __n, const char *__format, __va_list __arg);
//extern int snprintf(char *__s, size_t __n, const char *__format, ...);

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#if !defined(DEBUG_BUFFER_SIZE)
#define DEBUG_BUFFER_SIZE  128
#endif

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/   

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

#if !defined(TAL_NO_DEBUG)
static OS_SEMA    DebugSema;
static char       DebugBuffer[DEBUG_BUFFER_SIZE];
static uint8_t   bDebugInitDone = 0;
#endif

static uint32_t  dDebugMask = 0xFFFFFFFF;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  DebugInit                                                            */
/*                                                                       */
/*  Init the debug system.                                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
#if !defined(TAL_NO_DEBUG)
static void DebugInit (void)
{ 
   /* Create our semaphore */
   OS_RES_CREATE(&DebugSema);
} /* DebugInit */
#endif /* !defined(TAL_NO_DEBUG) */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  TAL_FATAL                                                            */
/*                                                                       */
/*  This function is called on fatal errors.                             */
/*                                                                       */
/*  In    : func, file, line, expected                                   */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
#if !defined(TAL_NO_ASSERT)
void TAL_FATAL (const char *func, const char *file, int line, const char *expected)
{
   if (expected != NULL)
   {
      term_printf("%s:%d: Assert: Expected %s in %s\n", file, line, expected, func);
   }
   else
   {
      term_printf("%s:%d: Failed: in %s\n", file, line, func);
   }  
   
   while (1)
   {
      __asm__ ("nop");
   }
   
} /* TAL_FATAL */
#endif /* !defined(TAL_NO_ASSERT) */

/*************************************************************************/
/*  tal_Debug                                                            */
/*                                                                       */
/*  This function is like a printf, but with output mask.                */
/*                                                                       */
/*  In    : level, fmt, ...                                              */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
#if !defined(TAL_NO_DEBUG)
void tal_Debug (uint32_t dMask, const char *fmt, ...)
{
   va_list ap;
   int      n;
   uint32_t Time;

   /* Check if we must init the debug system */   
   if (0 == bDebugInitDone)
   {
      bDebugInitDone = 1;
      DebugInit();
   }
   
   /* Check if the mask is enabled */
   if (dDebugMask & dMask)
   {
      OS_RES_LOCK(&DebugSema);
      
      /* Get current time */
      Time = OS_TimeGet();
      
      n = snprintf(DebugBuffer, sizeof(DebugBuffer), "%d.%03d ",
                   Time/1000, Time%1000);

      va_start(ap, fmt);
      vsnprintf(&DebugBuffer[n], sizeof(DebugBuffer)-n, fmt, ap); /*lint !e737*/ 
      va_end(ap);
      (void)ap;
      
      term_puts(DebugBuffer);
      
      OS_RES_FREE(&DebugSema);
   }      
} /* tal_Debug */
#endif /* !defined(TAL_NO_DEBUG) */

/*************************************************************************/
/*  tal_DebugMaskSet                                                     */
/*                                                                       */
/*  Set the output mask of the TAL_DEBUG function.                       */
/*                                                                       */
/*  In    : level                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_DebugMaskSet (uint32_t dMask)
{
   dDebugMask |= dMask;   
} /* tal_DebugMaskSet */

/*************************************************************************/
/*  tal_DebugMaskClr                                                     */
/*                                                                       */
/*  Clear the output mask of the TAL_DEBUG function.                     */
/*                                                                       */
/*  In    : level                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_DebugMaskClr (uint32_t dMask)
{
   dDebugMask &= ~dMask;   
} /* tal_DebugMaskClr */

/*** EOF ***/
