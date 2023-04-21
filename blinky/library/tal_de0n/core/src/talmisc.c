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
#define __TALMISC_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include "tal.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  tal_MISCRingSetup                                                    */
/*                                                                       */
/*  Setup the ring buffer.                                               */
/*                                                                       */
/*  In    : pRing, pBuffer, wElementSize, wElementCount                  */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_MISCRingSetup (TAL_MISC_RING *pRing, 
                        uint8_t       *pBuffer, 
                        uint16_t       wElementSize,
                        uint16_t       wElementCount)
{
   pRing->pBuffer          = pBuffer;
   pRing->wElementSize     = wElementSize;
   pRing->wMaxElementCount = wElementCount; 
   pRing->wInIndex         = 0;
   pRing->wOutIndex        = 0;
   pRing->wCount           = 0;    
} /* tal_MISCRingSetup */

/*************************************************************************/
/*  tal_MISCRingReset                                                    */
/*                                                                       */
/*  Reset the ring buffer.                                               */
/*                                                                       */
/*  In    : pRing                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_MISCRingReset (TAL_MISC_RING *pRing)
{
   TAL_CPU_DISABLE_ALL_INTS();

   pRing->wInIndex  = 0;
   pRing->wOutIndex = 0;
   pRing->wCount    = 0;
   
   TAL_CPU_ENABLE_ALL_INTS();
} /* tal_MISCRingReset */

/*************************************************************************/
/*  tal_MISCRingGetFreeCount                                             */
/*                                                                       */
/*  Return the free count of the ring buffer.                            */
/*                                                                       */
/*  In    : pRing                                                        */
/*  Out   : none                                                         */
/*  Return: Count                                                        */
/*************************************************************************/
uint16_t tal_MISCRingGetFreeCount (TAL_MISC_RING *pRing)
{
   uint16_t wCount;
   
   TAL_CPU_DISABLE_ALL_INTS();

   wCount = pRing->wMaxElementCount - pRing->wCount;
   
   TAL_CPU_ENABLE_ALL_INTS();
   
   return(wCount);
} /* tal_MISCRingGetFreeCount */

/*************************************************************************/
/*  tal_MISCRingGetUseCount                                              */
/*                                                                       */
/*  Return the used count of the ring buffer.                            */
/*                                                                       */
/*  In    : pRing                                                        */
/*  Out   : none                                                         */
/*  Return: Count                                                        */
/*************************************************************************/
uint16_t tal_MISCRingGetUseCount (TAL_MISC_RING *pRing)
{
   uint16_t wCount;
   
   TAL_CPU_DISABLE_ALL_INTS();

   wCount = pRing->wCount;
   
   TAL_CPU_ENABLE_ALL_INTS();
   
   return(wCount);
} /* tal_MISCRingGetUseCount */

/*************************************************************************/
/*  tal_MISCRingAdd                                                      */
/*                                                                       */
/*  Add a new element to the ring buffer.                                */
/*                                                                       */
/*  In    : pRing, pData                                                 */
/*  Out   : none                                                         */
/*  Return: TAL_OK / TAL_ERROR                                           */
/*************************************************************************/
TAL_RESULT tal_MISCRingAdd (TAL_MISC_RING *pRing, uint8_t *pData)
{
   TAL_RESULT Error = TAL_ERR_RING_FULL;
   uint16_t  wIndex;

   TAL_CPU_DISABLE_ALL_INTS();
   
   /* Check if a free entry is available */
   if (pRing->wCount != pRing->wMaxElementCount)
   {
      pRing->wCount++;
      
      wIndex = (uint16_t)(pRing->wInIndex * pRing->wElementSize);
      
      memcpy(&pRing->pBuffer[wIndex], pData, pRing->wElementSize);
      
      pRing->wInIndex++;
      
      if (pRing->wInIndex == pRing->wMaxElementCount)
      {
         pRing->wInIndex = 0;
      }
      
      Error = TAL_OK;
   }

   TAL_CPU_ENABLE_ALL_INTS();
   
   return(Error);
} /* tal_MISCRingAdd */

/*************************************************************************/
/*  tal_MISCRingGet                                                      */
/*                                                                       */
/*  Get an element from the ring buffer.                                 */
/*                                                                       */
/*  In    : pRing, pData                                                 */
/*  Out   : pData                                                        */
/*  Return: TAL_OK / TAL_ERROR                                           */
/*************************************************************************/
TAL_RESULT tal_MISCRingGet (TAL_MISC_RING *pRing, uint8_t *pData)
{
   TAL_RESULT Error = TAL_ERR_RING_EMPTY;
   uint16_t  wIndex;

   TAL_CPU_DISABLE_ALL_INTS();
   
   /* Check if an entry is available */
   if (pRing->wCount != 0)
   {
      pRing->wCount--;
      
      wIndex = (uint16_t)(pRing->wOutIndex * pRing->wElementSize);
      
      memcpy(pData, &pRing->pBuffer[wIndex], pRing->wElementSize);
      
      pRing->wOutIndex++;
      
      if (pRing->wOutIndex == pRing->wMaxElementCount)
      {
         pRing->wOutIndex = 0;
      }
      
      Error = TAL_OK;
   }

   TAL_CPU_ENABLE_ALL_INTS();

   return(Error);   
} /* tal_MISCRingGet */

/*** EOF ***/
