/****************************************************************************
*  Copyright (c) 2019-2022 by Michael Fischer (www.emb4fun.de).
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
****************************************************************************/
#define __DHRY_PORT_C__

/****************************************************************************
*   I N C L U D E
****************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include "tal.h"
#include "tcts.h"

/****************************************************************************
*   D E F I N E 
****************************************************************************/

/****************************************************************************
*   M A C R O   D E F I N I T I O N S
****************************************************************************/

/****************************************************************************
*   G L O B A L 
****************************************************************************/

/****************************************************************************
*   S T A T I C 
****************************************************************************/

/****************************************************************************
*   L O C A L   F U N C T I O N S
****************************************************************************/

/****************************************************************************
*   G L O B A L  F U N C T I O N S
****************************************************************************/

/***************************************************************************/
/*  Name  : dhry_HWInit                                                    */
/*                                                                         */
/*  Init the CPU / hardware.                                               */
/*                                                                         */
/*  In    : none                                                           */
/*  Out   : none                                                           */
/*  Return: none                                                           */
/***************************************************************************/
void dhry_HWInit (void)
{
} /* dhry_HWInit */

/***************************************************************************/
/*  Name  : dhry_GetCPUFreqMHz                                             */
/*                                                                         */
/*  Return the CPU frequency in MHz.                                       */
/*                                                                         */
/*  In    : none                                                           */
/*  Out   : none                                                           */
/*  Return: Frequency                                                      */
/***************************************************************************/
float dhry_GetCPUFreqMHz (void)
{
   float Frequency;
   
   Frequency = (float)tal_CPUGetFrequencyCPU() / 1000000.0;   
   
   return(Frequency);
} /* dhry_GetCPUFreqMHz */

/***************************************************************************/
/*  Name  : dhry_ftime                                                     */
/*                                                                         */
/*  Return the actual time in seconds and setup the timer functinality.    */
/*                                                                         */
/*  In    : none                                                           */
/*  Out   : none                                                           */
/*  Return: Time                                                           */
/***************************************************************************/
float dhry_ftime (void)
{
   float Time;
   
   Time = (float)OS_TimeGet() / OS_TICKS_PER_SECOND;
   
   return(Time);   
} /* dhry_ftime */

/*** EOF ***/
