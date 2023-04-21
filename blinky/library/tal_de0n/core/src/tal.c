/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
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
**************************************************************************/
#define __TAL_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include "tal.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#if defined(RTOS_TCTS)
#define _OS_Init()   OS_Init()
#else
#define _OS_Init()
#endif

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  OS_TCTS_Init                                                         */
/*                                                                       */
/*  If TinyCTS is not used, this function makes the linker happy.        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void __attribute__((weak)) OS_TCTS_Init (void)
{
   /* Only an empty function */
} /* OS_TCTS_Init */

/*************************************************************************/
/*  tal_CANInit                                                          */
/*                                                                       */
/*  If the CPU does not support CAN, talcan.c will not included in the   */
/*  project. But the function "tal_CANInit" is still needed.             */ 
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void __attribute__((weak)) tal_CANInit (void)
{
   /* Only an empty function */
} /* tal_CANInit */

/*************************************************************************/
/*  tal_MEMInit                                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void __attribute__((weak)) tal_MEMInit (void)
{
   /* Only an empty function */
} /* tal_MEMInit */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  tal_Init                                                             */
/*                                                                       */
/*  Init the "Tiny Abstraction Layer".                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_Init (void)
{
   TAL_CPU_DISABLE_ALL_INTS();

   tal_CPUInit();    /* Init the CPU module */
   OS_TCTS_Init();   /* Init the OS module  */
   tal_GPIOInit();   /* Init the GPIO module  */
   tal_LEDInit();    /* Init the LED module */
   tal_COMInit();    /* Init the COM module */
   tal_CANInit();    /* Init the CAN module */    /*lint !e522*/
   tal_MEMInit();    /* Init the memory module */ /*lint !e522*/

   TAL_CPU_ENABLE_ALL_INTS();
} /* tal_Init */

/*** EOF ***/
