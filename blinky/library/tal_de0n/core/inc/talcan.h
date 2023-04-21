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
#if !defined(__TALCAN_H__)
#define __TALCAN_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include "talboard.h"
#include "taltypes.h"
#include "talmisc.h"
#include "talcpu_can.h"

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * Some magic CAN identifier
 */
#define TAL_CAN_ID_RX_ALL  0xFFFFFFFF
#define TAL_CAN_ID_STATUS  0xFFFFFFFE 
 

/*
 * BitRate defines
 */
#define TAL_CAN_BR_1000K   1000000L
#define TAL_CAN_BR_500K    500000L
#define TAL_CAN_BR_250K    250000L
#define TAL_CAN_BR_125K    125000L
#define TAL_CAN_BR_100K    100000L
#define TAL_CAN_BR_50K     50000L
#define TAL_CAN_BR_20K     20000L
#define TAL_CAN_BR_10K     10000L
#define TAL_CAN_BR_5K      5000L


/*
 * Type of CAN identifier
 */
typedef enum _tal_can_id_
{
   TAL_CAN_TYPE_STD_ID = 0,
   TAL_CAN_TYPE_EXT_ID
} TAL_CAN_TYPE_ID;

/*
 * Enum for using the ring buffer
 */
typedef enum _tal_can_buffer_
{
   TAL_CAN_BUFFER_RX = 0,
   TAL_CAN_BUFFER_TX
} TAL_CAN_BUFFER;


/*
 * Communiction settings
 */
typedef struct _tal_can_settings_
{
  uint32_t  dBitRate;
} TAL_CAN_SETTINGS;

/*
 * Timestamp
 */
typedef uint32_t (*GET_TIMESTAMP)(void);

/*
 * CAN device control block
 */
typedef struct _tal_can_dcb_
{
   /* "Header" */
   uint8_t          bDCBInitDone;
   TAL_MAGIC        eMagic;
   TAL_CAN_PORT     ePort;
   uint8_t          bIsOpen;
   uint8_t          bRxOverflow;
   
   /* Synchronisation */
   OS_SEMA           Sema;
   OS_SEMA           TxSema;
   OS_SEMA           RxRdySema;   

   /* BitRate and so on... */   
   TAL_CAN_SETTINGS  Settings;
   
   /* Rx and Tx ring buffer */   
   TAL_MISC_RING     RxRing;
   TAL_MISC_RING     TxRing;

   /* Timestamp function */
   GET_TIMESTAMP     GetRxTimestamp;
   
   /* HW information from talcpu_can.h */
   TAL_CAN_HW        HW;
} TAL_CAN_DCB;


/*
 * Here comes the CAN object
 */
#define TAL_CAN_OBJECT_FLAGS_STD_ID    0x01
#define TAL_CAN_OBJECT_FLAGS_EXT_ID    0x02
#define TAL_CAN_MAX_OBJECT_DATA        8

typedef struct _tal_can_object_
{
   uint32_t dTimestamp;
   uint32_t dIdentifier;
   uint8_t  bFlags;
   uint8_t  bDLC;
   uint8_t   Data[TAL_CAN_MAX_OBJECT_DATA];
} TAL_CAN_OBJECT;

/**************************************************************************
*  Macro Definitions
**************************************************************************/

#define tal_CANIsOpen(_a)  (_a)->bIsOpen

/**************************************************************************
*  Functions Definitions
**************************************************************************/

void       tal_CANInit (void);

TAL_RESULT tal_CANInitDCB (TAL_CAN_DCB *pDCB, TAL_CAN_PORT ePort);

TAL_RESULT tal_CANSetRingBuffer (TAL_CAN_DCB    *pDCB, 
                                 TAL_CAN_BUFFER  eBuffer,
                                 TAL_CAN_OBJECT *pBuffer,
                                 uint16_t        wBufferSize);

TAL_RESULT tal_CANSetRxTimestampFunc (TAL_CAN_DCB *pDCB, 
                                      GET_TIMESTAMP GetRxTimestamp);                                 
TAL_RESULT tal_CANClearOverflow (TAL_CAN_DCB *pDCB); 

TAL_RESULT tal_CANOpen (TAL_CAN_DCB *pDCB, TAL_CAN_SETTINGS *pSettings);
TAL_RESULT tal_CANClose (TAL_CAN_DCB *pDCB);

TAL_RESULT tal_CANEnableSilentMode (TAL_CAN_DCB *pDCB);
TAL_RESULT tal_CANDisableSilentMode (TAL_CAN_DCB *pDCB);

TAL_RESULT tal_CANMsgSend (TAL_CAN_DCB *pDCB, TAL_CAN_OBJECT *pCANObject);
TAL_RESULT tal_CANMsgGet (TAL_CAN_DCB *pDCB, TAL_CAN_OBJECT *pCANObject);
TAL_RESULT tal_CANMsgGetWait (TAL_CAN_DCB *pDCB, TAL_CAN_OBJECT *pCANObject, uint32_t dTimeoutMs);

TAL_RESULT tal_CANIdentRegister (TAL_CAN_DCB   *pDCB,
                                 uint32_t       dIdentifier,
                                 TAL_CAN_TYPE_ID Type); 
                                      
TAL_RESULT tal_CANIdentDeRegister (TAL_CAN_DCB   *pDCB,
                                   uint32_t       dIdentifier,
                                   TAL_CAN_TYPE_ID Type); 

/*************************************************************************/

TAL_RESULT cpu_CANInit (TAL_CAN_DCB *pDCB);
TAL_RESULT cpu_CANOpen (TAL_CAN_DCB *pDCB);
TAL_RESULT cpu_CANClose (TAL_CAN_DCB *pDCB);
TAL_RESULT cpu_CANStartTx (TAL_CAN_DCB *pDCB);
TAL_RESULT cpu_CANTxIsRunning (TAL_CAN_DCB *pDCB);

TAL_RESULT cpu_CANEnableSilentMode (TAL_CAN_DCB *pDCB);
TAL_RESULT cpu_CANDisableSilentMode (TAL_CAN_DCB *pDCB);

TAL_RESULT cpu_CANIdentRegister (TAL_CAN_DCB   *pDCB,
                                 uint32_t       dIdentifier,
                                 TAL_CAN_TYPE_ID Type); 
                                      
TAL_RESULT cpu_CANIdentDeRegister (TAL_CAN_DCB   *pDCB,
                                   uint32_t       dIdentifier,
                                   TAL_CAN_TYPE_ID Type); 

void       cpu_CANGetSpeed (TAL_CAN_PORT ePort, uint32_t *pSpeed);

#endif /* !__TALCAN_H__ */

/*** EOF ***/
