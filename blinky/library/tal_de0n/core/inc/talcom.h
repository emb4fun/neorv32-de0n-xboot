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
#if !defined(__TALCOM_H__)
#define __TALCOM_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include "talboard.h"
#include "taltypes.h"
#include "talmisc.h"
#include "talcpu_com.h"

/**************************************************************************
*  Global Definitions
**************************************************************************/

typedef enum _tal_com_buffer_
{
   TAL_COM_BUFFER_RX = 0,
   TAL_COM_BUFFER_TX
} TAL_COM_BUFFER;

typedef enum _tal_com_length_
{
   TAL_COM_LENGTH_8 = 0,
   TAL_COM_LENGTH_9
} TAL_COM_LENGTH;

typedef enum _tal_com_parity_
{
   TAL_COM_PARITY_NONE = 0,
   TAL_COM_PARITY_EVEN,
   TAL_COM_PARITY_ODD
} TAL_COM_PARITY;

typedef enum _tal_com_stop_
{
   TAL_COM_STOP_0_5 = 0,
   TAL_COM_STOP_1_0,
   TAL_COM_STOP_1_5,
   TAL_COM_STOP_2_0
} TAL_COM_STOP;

typedef struct _tal_com_settings_
{
  uint32_t       dBaudrate;
  TAL_COM_LENGTH eLength;
  TAL_COM_PARITY eParity;
  TAL_COM_STOP   eStop;
} TAL_COM_SETTINGS;

typedef enum _tal_com_ioctl_
{
   TAL_COM_IOCTL_NONE = 0,
   TAL_COM_IOCTL_RS485_MODE,
   TAL_COM_IOCTL_RS485_TX_ENABLE_FUNC,
   TAL_COM_IOCTL_RS485_TX_DISABLE_FUNC   
} TAL_COM_IOCTL;

/*
 * COM device control block
 */
typedef struct _tal_com_dcb_
{
   /* "Header" */
   uint8_t          bDCBInitDone;
   TAL_MAGIC        eMagic;
   TAL_COM_PORT     ePort;
   uint8_t          bIsOpen;
   uint8_t          bRxOverflow;

   /* Synchronisation */
   OS_SEMA           Sema;
   OS_SEMA           TxSema;
   OS_SEMA           RxRdySema;

   /* Baudrate, Bits, Parity and so on... */   
   TAL_COM_SETTINGS  Settings;
   
   /* Rx and Tx ring buffer */   
   TAL_MISC_RING     RxRing;
   TAL_MISC_RING     TxRing;
   
   /* HW information from talcpu_com.h */
   TAL_COM_HW        HW;
} TAL_COM_DCB;

/**************************************************************************
*  Macro Definitions
**************************************************************************/

#define tal_COMIsOpen(_a)  (_a)->bIsOpen

/**************************************************************************
*  Functions Definitions
**************************************************************************/

void       tal_COMInit (void);

TAL_RESULT tal_COMInitDCB (TAL_COM_DCB *pDCB, TAL_COM_PORT ePort);
TAL_RESULT tal_COMIoctl (TAL_COM_DCB *pDCB, TAL_COM_IOCTL eFunc, uint32_t *pParam);

TAL_RESULT tal_COMSetRingBuffer (TAL_COM_DCB   *pDCB, 
                                 TAL_COM_BUFFER eBuffer,
                                 uint8_t       *pBuffer,
                                 uint16_t       wBufferSize);
                                 
TAL_RESULT tal_COMClearOverflow (TAL_COM_DCB *pDCB); 

TAL_RESULT tal_COMOpen (TAL_COM_DCB *pDCB, TAL_COM_SETTINGS *pSettings);
TAL_RESULT tal_COMClose (TAL_COM_DCB *pDCB);

TAL_RESULT tal_COMSendBlock (TAL_COM_DCB *pDCB,  uint8_t *pData, uint16_t wSize);
TAL_RESULT tal_COMSendString (TAL_COM_DCB *pDCB, char *pString);
void       tal_COMSendStringASS (TAL_COM_DCB *pDCB, char *pString);
TAL_RESULT tal_COMSendChar (TAL_COM_DCB *pDCB, char cData);

TAL_RESULT tal_COMReceiveChar (TAL_COM_DCB *pDCB, uint8_t *pData);
TAL_RESULT tal_COMReceiveCharWait (TAL_COM_DCB *pDCB, uint8_t *pData, uint32_t dTimeout);
TAL_RESULT tal_COMReceiveCharTest (TAL_COM_DCB *pDCB);
TAL_RESULT tal_COMReceiveCharTestWait (TAL_COM_DCB *pDCB, uint32_t dTimeout);

/*************************************************************************/

TAL_RESULT cpu_COMInit (TAL_COM_DCB *pDCB);
TAL_RESULT cpu_COMIoctl (TAL_COM_DCB *pDCB, TAL_COM_IOCTL eFunc, uint32_t *pParam);
TAL_RESULT cpu_COMOpen (TAL_COM_DCB *pDCB);
TAL_RESULT cpu_COMClose (TAL_COM_DCB *pDCB);
TAL_RESULT cpu_COMStartTx (TAL_COM_DCB *pDCB);
TAL_RESULT cpu_COMTxIsRunning (TAL_COM_DCB *pDCB);

void       cpu_COMSendStringASS (TAL_COM_DCB *pDCB, char *pString);

#endif /* !__TALCOM_H__ */

/*** EOF ***/
