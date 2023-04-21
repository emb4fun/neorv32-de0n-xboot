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
#define __TALCAN_C__

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

/*************************************************************************/
/*  GetRxObject                                                          */
/*                                                                       */
/*  Get an object if available.                                          */
/*                                                                       */
/*  In    : pDCB, pData                                                  */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
static TAL_RESULT GetRxObject (TAL_CAN_DCB *pDCB, TAL_CAN_OBJECT *pCANObject)
{
   TAL_RESULT Error;

   Error = tal_MISCRingGet(&pDCB->RxRing, (uint8_t*)pCANObject);
   if (TAL_ERR_RING_EMPTY == Error)
   {
      /* Ring is empty */
      if (TAL_FALSE == pDCB->bRxOverflow)
      {
         Error = TAL_ERR_CAN_EMPTY;
      }
      else
      {
         Error = TAL_ERR_CAN_OVERFLOW_EMPTY;
      }
   }
   else if (TAL_OK == Error)
   {
      /* Ring OK */
      if (TAL_FALSE == pDCB->bRxOverflow)
      {
         Error = TAL_OK;
      }
      else
      {
         Error = TAL_ERR_CAN_OVERFLOW;
      }   
      
      /* Check for valid DLC */
      if (pCANObject->bDLC > 8)
      {
         pCANObject->bDLC = 8;
      }
   }
   
   return(Error);
} /* GetRxObject */ 

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  tal_CANInit                                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CANInit (void)
{
   /* Nothing to do here */
} /* tal_CANInit */

/*************************************************************************/
/*  tal_CANInitDCB                                                       */
/*                                                                       */
/*  Initialize the Device Control Block.                                 */
/*                                                                       */
/*  In    : pDCB, ePort                                                  */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANInitDCB (TAL_CAN_DCB *pDCB, TAL_CAN_PORT ePort)
{
   TAL_RESULT Error = TAL_ERR_NULL_POINTER;
   
   if (pDCB != 0) 
   {   
      /* Clear DCB first */
      memset(pDCB, 0x00, sizeof(TAL_CAN_DCB));

      /* Store Magic and Port information */
      pDCB->eMagic = TAL_MAGIC_CAN; 
      pDCB->ePort  = ePort;
      
      /* Init the CAN hardware layer */
      Error = cpu_CANInit(pDCB);
      if (TAL_OK == Error)
      {
         OS_RES_CREATE(&pDCB->Sema);
         OS_RES_CREATE(&pDCB->TxSema);
         
         OS_SemaCreate(&pDCB->RxRdySema, 0, OS_SEMA_COUNTER_MAX);
         
         pDCB->bDCBInitDone = TAL_TRUE;
      }
   }
   
   return(Error);
} /* tal_CANInitDCB */

/*************************************************************************/
/*  tal_CANSetRingBuffer                                                 */
/*                                                                       */
/*  Set the ring buffer values.                                          */
/*                                                                       */
/*  In    : pDCB, eBuffer, pBuffer, wBufferSize                          */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANSetRingBuffer (TAL_CAN_DCB    *pDCB, 
                                 TAL_CAN_BUFFER  eBuffer,
                                 TAL_CAN_OBJECT *pBuffer, 
                                 uint16_t        wBufferSize)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid parameters */
   if ( (pDCB          != NULL)               &&
        (pBuffer       != NULL)               &&
        (wBufferSize   != 0)                  &&  
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      OS_RES_LOCK(&pDCB->Sema);
      
      if (TAL_CAN_BUFFER_RX == eBuffer)
      {
         tal_MISCRingSetup(&pDCB->RxRing, 
                           (uint8_t*)pBuffer, 
                           sizeof(TAL_CAN_OBJECT), 
                           wBufferSize / sizeof(TAL_CAN_OBJECT));  
         
         Error = TAL_OK;
      }

      if (TAL_CAN_BUFFER_TX == eBuffer)
      {
         tal_MISCRingSetup(&pDCB->TxRing, 
                           (uint8_t*)pBuffer, 
                           sizeof(TAL_CAN_OBJECT), 
                           wBufferSize / sizeof(TAL_CAN_OBJECT));  
         
         Error = TAL_OK;
      }
      
      OS_RES_FREE(&pDCB->Sema);
   }
   
   return(Error);
} /* tal_CANSetRingBuffer */

/*************************************************************************/
/*  tal_CANSetTimestampFunc                                              */
/*                                                                       */
/*  Set the GetTimestamp function.                                       */
/*                                                                       */
/*  In    : pDCB, GetRxTimestamp                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANSetTimestampFunc (TAL_CAN_DCB *pDCB, GET_TIMESTAMP GetRxTimestamp)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid parameters */
   if ( (pDCB           != NULL)               &&
        (GetRxTimestamp != NULL)               &&
        (TAL_TRUE       == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN  == pDCB->eMagic)       )
   {
      pDCB->GetRxTimestamp = GetRxTimestamp;
      
      Error = TAL_OK;   
   } /* end if "test parameters" */        
   
   return(Error);
} /* tal_CANSetTimestampFunc */

/*************************************************************************/
/*  tal_CANClearOverflow                                                 */
/*                                                                       */
/*  Clear the ring buffer.                                               */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANClearOverflow (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid conditions */
   if ( (pDCB          != NULL)               &&
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_TRUE == pDCB->bIsOpen)
      {
         OS_RES_LOCK(&pDCB->Sema);
         
         tal_MISCRingReset(&pDCB->RxRing);
         
         /* Clear counting semaphore */
         OS_SemaDelete(&pDCB->RxRdySema);
         OS_SemaCreate(&pDCB->RxRdySema, 0, OS_SEMA_COUNTER_MAX);
         
         OS_RES_FREE(&pDCB->Sema);
         
         Error = TAL_OK;
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_OPEN;
      }
   }        

   return(Error);
} /* tal_CANClearOverflow */

/*************************************************************************/
/*  tal_CANOpen                                                          */
/*                                                                       */
/*  Open the CAN port with the given settings.                           */
/*                                                                       */
/*  In    : pDCB, pSettings                                              */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANOpen (TAL_CAN_DCB *pDCB, TAL_CAN_SETTINGS *pSettings)
{
   TAL_RESULT  Error = TAL_ERR_PARAMETER;
   
   /* Check for valid parameters */
   if ( (pDCB          != NULL)               &&
        (pSettings     != NULL)               &&
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_FALSE == pDCB->bIsOpen)
      {
         /* Check if RX ring buffer is available */
         if (NULL == pDCB->RxRing.pBuffer)
         {
            Error = TAL_ERR_CAN_NO_RX_BUFF;
            goto CANOpenEnd;  /*lint !e801*/
         }

         /* Check if TX ring buffer is available */
         if (NULL == pDCB->TxRing.pBuffer)
         {
            Error = TAL_ERR_CAN_NO_TX_BUFF;
            goto CANOpenEnd;  /*lint !e801*/
         }

         OS_RES_LOCK(&pDCB->Sema);

         /* Reset ring buffer */         
         tal_MISCRingReset(&pDCB->RxRing);
         tal_MISCRingReset(&pDCB->TxRing);
      
         /* Clear counting semaphore */
         OS_SemaDelete(&pDCB->RxRdySema);
         OS_SemaCreate(&pDCB->RxRdySema, 0, OS_SEMA_COUNTER_MAX);
      
         /* Copy settings */
         pDCB->Settings = *pSettings;
         
         Error = cpu_CANOpen(pDCB);
         if (TAL_OK == Error)
         {
            pDCB->bIsOpen = TAL_TRUE;
         }
         
         OS_RES_FREE(&pDCB->Sema);
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_CLOSED;
      }
   }   
   
CANOpenEnd:   
   return(Error);
} /* tal_CANOpen */

/*************************************************************************/
/*  tal_CANClose                                                         */
/*                                                                       */
/*  Close the CAN port.                                                  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANClose (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid conditions */
   if ( (pDCB          != NULL)               &&
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_TRUE == pDCB->bIsOpen)
      {
         OS_RES_LOCK(&pDCB->Sema);

         Error = cpu_CANClose(pDCB);
         if (TAL_OK == Error)
         {
            pDCB->bIsOpen = TAL_FALSE;
         }
         
         OS_RES_FREE(&pDCB->Sema);
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_OPEN;
      }
   }        

   return(Error);
} /* tal_CANClose */

/*************************************************************************/
/*  tal_CANEnableSilentMode                                              */
/*                                                                       */
/*  Enable "Silent" mode.                                                */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANEnableSilentMode (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid conditions */
   if ( (pDCB          != NULL)               &&
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_TRUE == pDCB->bIsOpen)
      {
         OS_RES_LOCK(&pDCB->Sema);
      
         Error = cpu_CANEnableSilentMode(pDCB);
         
         OS_RES_FREE(&pDCB->Sema);
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_OPEN;
      }
   }        

   return(Error);
} /* tal_CANEnableSilentMode */

/*************************************************************************/
/*  tal_CANDisableSilentMode                                             */
/*                                                                       */
/*  Disable "Silent" mode.                                               */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANDisableSilentMode (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid conditions */
   if ( (pDCB          != NULL)               &&
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_TRUE == pDCB->bIsOpen)
      {
         OS_RES_LOCK(&pDCB->Sema);
      
         Error = cpu_CANDisableSilentMode(pDCB);
         
         OS_RES_FREE(&pDCB->Sema);
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_OPEN;
      }
   }        

   return(Error);
} /* tal_CANDisableSilentMode */

/*************************************************************************/
/*  tal_CANIdentRegister                                                 */
/*                                                                       */
/*  Register a CAN identifier.                                           */
/*                                                                       */
/*  In    : pDCB, dIdentifier, Type                                      */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANIdentRegister (TAL_CAN_DCB   *pDCB,
                                 uint32_t       dIdentifier,
                                 TAL_CAN_TYPE_ID Type)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid conditions */
   if ( (pDCB          != NULL)               &&
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_TRUE == pDCB->bIsOpen)
      {
         OS_RES_LOCK(&pDCB->Sema);
      
         Error = cpu_CANIdentRegister(pDCB, dIdentifier, Type);
         
         OS_RES_FREE(&pDCB->Sema);
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_OPEN;
      }
   }        

   return(Error);
} /* tal_CANIdentRegister */                                 

/*************************************************************************/
/*  tal_CANIdentDeRegister                                               */
/*                                                                       */
/*  DeRegister a CAN identifier.                                         */
/*                                                                       */
/*  In    : pDCB, dIdentifier, Type                                      */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANIdentDeRegister (TAL_CAN_DCB   *pDCB,
                                   uint32_t       dIdentifier,
                                   TAL_CAN_TYPE_ID Type)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid conditions */
   if ( (pDCB          != NULL)               &&
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_TRUE == pDCB->bIsOpen)
      {
         OS_RES_LOCK(&pDCB->Sema);
      
         Error = cpu_CANIdentDeRegister(pDCB, dIdentifier, Type);
         
         OS_RES_FREE(&pDCB->Sema);
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_OPEN;
      }
   }        

   return(Error);
} /* tal_CANIdentDeRegister */                                 

/*************************************************************************/
/*  tal_CANMsgSend                                                       */
/*                                                                       */
/*  Put a message in the ring buffer and start send it if needed.        */
/*                                                                       */
/*  In    : pDCB, pCANObject                                             */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANMsgSend (TAL_CAN_DCB *pDCB, TAL_CAN_OBJECT *pCANObject)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid conditions */
   if ( (pDCB          != NULL)               &&
        (pCANObject    != NULL)               && 
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_TRUE == pDCB->bIsOpen)
      {
         OS_RES_LOCK(&pDCB->TxSema);

#if 0      
         /* Add data to the ring buffer */
         Error = tal_MISCRingAdd(&pDCB->TxRing, (uint8_t*)pCANObject);
         if (TAL_OK == Error)
         {
            /* 
             * The data was added start the transmitter.
             */
            cpu_CANStartTx(pDCB);
         }
         else
         {
            /* Error, no free ring buffer entry */
            Error = TAL_ERR_CAN_MSG_NOT_SEND;
         }
#else
         /* Add data to the ring buffer */
         Error = tal_MISCRingAdd(&pDCB->TxRing, (uint8_t*)pCANObject);
         if (Error != TAL_OK)
         {
            /* 
             * The data was not added to the ring buffer!
             *
             * Ring buffer is full, wait for an free entry. 
             */
            while ( 0 == tal_MISCRingGetFreeCount(&pDCB->TxRing) )
            {
               /* Check if the transmitter is still runing */
               if (TAL_ERROR == cpu_CANTxIsRunning(pDCB))
               {
                  /* No, start the transmitter */
                  cpu_CANStartTx(pDCB);
               }                     
               OS_TimeDly(1);
            }
            
            /*
             * A free entry is available, we can add
             * the data to the ring buffer now.
             */
            tal_MISCRingAdd(&pDCB->TxRing, (uint8_t*)pCANObject);
         }
         
         /* 
          * The data was added start the transmitter.
          */
         cpu_CANStartTx(pDCB);
#endif         

         OS_RES_FREE(&pDCB->TxSema);
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_OPEN;
      }
   }        

   return(Error);
} /* tal_CANMsgSend */

/*************************************************************************/
/*  tal_CANMsgGet                                                        */
/*                                                                       */
/*  Get a CAN message from the ring buffer if available.                 */
/*                                                                       */
/*  In    : pDCB, pCANObject                                             */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANMsgGet (TAL_CAN_DCB *pDCB, TAL_CAN_OBJECT *pCANObject)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   
   /* Check for valid conditions */
   if ( (pDCB          != NULL)               &&
        (pCANObject    != NULL)               && 
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_TRUE == pDCB->bIsOpen)
      {
         OS_RES_LOCK(&pDCB->Sema);
         
         Error = GetRxObject(pDCB, pCANObject);
         
         OS_RES_FREE(&pDCB->Sema);
         
         if ((TAL_OK == Error) || (TAL_ERR_COM_OVERFLOW == Error))
         {
            /* Data was available, RxRdySema must be decrease */
            OS_SemaWait(&pDCB->RxRdySema, 1);
         }
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_OPEN;
      }      
   }        

   return(Error);
} /* tal_CANMsgGet */

/*************************************************************************/
/*  tal_CANMsgGetWait                                                    */
/*                                                                       */
/*  Get a CAN message from the ring buffer if available,                 */
/*  but with timeout.                                                    */
/*                                                                       */
/*  In    : pDCB, pCANObject, dTimeoutMs                                 */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_CANMsgGetWait (TAL_CAN_DCB *pDCB, TAL_CAN_OBJECT *pCANObject, uint32_t dTimeoutMs)
{
   TAL_RESULT Error = TAL_ERR_PARAMETER;
   int        rc;
   
   /* Check for valid conditions */
   if ( (pDCB          != NULL)               &&
        (pCANObject    != NULL)               && 
        (TAL_TRUE      == pDCB->bDCBInitDone) &&
        (TAL_MAGIC_CAN == pDCB->eMagic)       )
   {
      if (TAL_TRUE == pDCB->bIsOpen)
      {
         OS_RES_LOCK(&pDCB->Sema);
         
         /* Check if data is available */         
         Error = GetRxObject(pDCB, pCANObject);
         if ((TAL_OK == Error) || (TAL_ERR_COM_OVERFLOW == Error))
         {
            /* Data was available, RxRdySema must be decrease */
            OS_SemaWait(&pDCB->RxRdySema, 1);
         }
         else
         {
            /* Data is not available, wait for it */
            rc = OS_SemaWait(&pDCB->RxRdySema, dTimeoutMs);
            if (OS_RC_OK == rc)
            {
               /* Data is available */
               Error = GetRxObject(pDCB, pCANObject);
            }
         }
         
         OS_RES_FREE(&pDCB->Sema);
      }
      else
      {
         Error = TAL_ERR_CAN_NOT_OPEN;
      }      
   }        

   return(Error);
} /* tal_CANMsgGetWait */

/*** EOF ***/
