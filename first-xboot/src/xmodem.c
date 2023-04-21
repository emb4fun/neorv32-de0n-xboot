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
*
***************************************************************************
*  History:
*
*  22.07.2017  mifi  First Version.
*                    Reference is made to:
*                    http://pauillac.inria.fr/~doligez/zmodem/ymodem.txt
**************************************************************************/
#define __XMODEM_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "project.h"
#include "xmodem.h"
#include "crc16.h"

/*=======================================================================*/
/*  All extern data                                                      */
/*=======================================================================*/

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define XMODEM_MAX_FRAME_SIZE 128
#define FRAME_BUFFER_SIZE     (2+XMODEM_MAX_FRAME_SIZE+2)

#define SOH    0x01  /* Start of frame 128 */
#define STX    0x02  /* Start of frame 1024 */

#define EOT    0x04
#define ACK    0x06
#define NAK    0x15
#define CAN    0x18
#define EOF    0x1a

typedef enum
{
   STATE_IDLE = 0,
   STATE_RECEIVE,
   STATE_ABORT,
   STATE_END_CAN,
   STATE_END_EOT
} state_t;


/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static uint8_t   FrameBuffer[FRAME_BUFFER_SIZE];
static state_t   RXState = STATE_IDLE;
static uint8_t  bRXPacketNumber;
static uint32_t dTXIndex;

static uint8_t *pImageBuffer = NULL;
static uint32_t dImageBufferSize;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  UartReceive                                                          */
/*                                                                       */
/*  Get one character if available.                                      */
/*                                                                       */
/*  In    : dTimeout                                                     */
/*  Out   : none                                                         */
/*  Return: -1 = Timeout / data                                          */
/*************************************************************************/
static int UartReceive (uint32_t dTimeout)
{
   int      nData = -1;
   uint32_t dEndTime;

   dEndTime = OSTimeGet() + OS_MS_2_TICKS(dTimeout);

   while (OSTimeGet() < dEndTime)
   {
      nData = COMGetChar();
      if (nData != -1)
      {
         break;
      }
   }

   return(nData);
} /* UartReceive */

/*************************************************************************/
/*  UartSend                                                             */
/*                                                                       */
/*  Send one character.                                                  */
/*                                                                       */
/*  In    : bData                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void UartSend (uint8_t bData)
{
   COMSendChar(bData);
} /* UartSend */

/*************************************************************************/
/*  UartClearInput                                                       */
/*                                                                       */
/*  Clear UART input buffer.                                             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void UartClearInput (void)
{
   int nData;

   nData = UartReceive(100);
   while (nData >= 0)
   {
      nData = UartReceive(100);
   }

} /* UartClearInput */

/*************************************************************************/
/*  ReceivePacket                                                        */
/*                                                                       */
/*  Receive the next packet and return an error if available.            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Error = 1 / OK = 0                                           */
/*************************************************************************/
static uint8_t ReceivePacket (uint16_t wFrameSize)
{
   uint8_t   bError = 0;
   uint16_t  wFrameBufferSize;
   uint16_t  wCount;
   uint16_t  wCRC1;
   uint16_t  wCRC2;
   int       nData;

   /* Only support a frame size of XMODEM_MAX_FRAME_SIZE */
   if (wFrameSize != XMODEM_MAX_FRAME_SIZE)
   {
      return(1); /* Error */
   }

   wFrameBufferSize = 2 + wFrameSize + 2;

   for(wCount = 0; wCount < wFrameBufferSize; wCount++)
   {
      nData = UartReceive(1000);
      if (nData < 0)
      {
         UartSend(NAK);
         goto ReceivePacketEnd;  /*lint !e801*/
      }
      else
      {
         FrameBuffer[wCount] = (uint8_t)nData;
      }
   }

   /* Check packet and ~packet */
   if ((FrameBuffer[0] + FrameBuffer[1]) != 0xFF)
   {
      UartClearInput();
      UartSend(NAK);
      goto ReceivePacketEnd;  /*lint !e801*/
   }

   /* Check CRC */
   wCRC1  = ((uint16_t)FrameBuffer[(wFrameBufferSize-2)] << 8) | FrameBuffer[(wFrameBufferSize-1)]; /*lint !e817*/
   wCRC2  = crc16_ccitt(0, &FrameBuffer[2], wFrameSize);
   if (wCRC1 != wCRC2)
   {
      UartClearInput();
      UartSend(NAK);
      goto ReceivePacketEnd;  /*lint !e801*/
   }

   /* Check packet number */
   if (FrameBuffer[0] != bRXPacketNumber)
   {
      /*
       * Synchronizing: If a valid block number is received, it will be: 1) the
       * expected one, in which case everything is fine; or 2) a repeat of the
       * previously received block.  This should be considered OK, and only
       * indicates that the receivers <ack> got glitched, and the sender re-
       * transmitted; 3) any other block number indicates a fatal loss of
       * synchronization, such as the rare case of the sender getting a line-glitch
       * that looked like an <ack>.  Abort the transmission, sending a <can>
       */
      if (FrameBuffer[0] == (bRXPacketNumber - 1))
      {
         /* Packet ok, but the last ACK must be lost by the sender */
         UartSend(ACK);
      }
      else
      {
         /* Out of sequence */
         bError  = 1;
         RXState = STATE_ABORT;
      }
   }
   else
   {
      /* Use data */
      if ((dTXIndex + wFrameSize) < dImageBufferSize)
      {
         memcpy(pImageBuffer, &FrameBuffer[2], wFrameSize);
         pImageBuffer += wFrameSize;
         dTXIndex     += wFrameSize;
      }

      bRXPacketNumber++;
      UartSend(ACK);
   }


ReceivePacketEnd:

   return(bError);
} /* ReceivePacket */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  XMStart                                                              */
/*                                                                       */
/*  In    : pBuffer, dBufferSize                                         */
/*  Out   : none                                                         */
/*  Return: TRUE = OK / FALSE = ERROR                                    */
/*************************************************************************/
int XMStart (uint8_t *pBuffer, uint32_t dBufferSize)
{
   int       nResult = FALSE;
   int       nData;
   uint8_t   bData;

   dTXIndex         = 0;
   pImageBuffer     = pBuffer;
   dImageBufferSize = dBufferSize;

   RXState          = STATE_IDLE;
   bRXPacketNumber  = 1;


   UartClearInput();

   while((STATE_IDLE == RXState) || (STATE_RECEIVE == RXState))
   {
      if (STATE_IDLE == RXState)
      {
         /*
          * Send 'C' until sender response
          */
         while(1)
         {
            UartSend('C');
            nData = UartReceive(1000);
            if (nData > 0)
            {
               bData   = (uint8_t)(nData & 0xFF);
               RXState = STATE_RECEIVE;
               break;
            }
         }
      }
      else
      {
         nData = UartReceive(10000);
         bData = (uint8_t)(nData & 0xFF);
      }

      if (nData < 0)
      {
         RXState = STATE_ABORT;
      }
      else
      {
         switch(bData)
         {
            case SOH: /* Frame size 128 */
            {
               if (ReceivePacket(128))
               {
                  RXState = STATE_ABORT;
               }
               break;
            } /* SOH */

            case STX: /* Frame size 1024 */
            {
               if (ReceivePacket(1024))
               {
                  RXState = STATE_ABORT;
               }
               break;
            } /* STX */

            case CAN:   /* Abort from the transmitter side */
            {
               RXState = STATE_END_CAN;
               break;
            }

            case EOT:
            {
               RXState = STATE_END_EOT;
               break;
            } /* EOT */

            default:
            {
               RXState = STATE_ABORT;
               break;
            }
         }
      }
   }

   /* Check if an ABORT must be send */
   if (STATE_ABORT == RXState)
   {
      UartSend(CAN);
      UartSend(CAN);
      UartSend(CAN);
      UartSend(CAN);
      UartSend(CAN);

      nResult = FALSE;
   }

   /* Check if an ACK must be send */
   if ((STATE_END_EOT == RXState) || (STATE_END_CAN == RXState))
   {
      UartSend(ACK);
      UartSend(ACK);
      UartSend(ACK);
      UartSend(ACK);
      UartSend(ACK);

      if (STATE_END_EOT == RXState)
      {
         nResult = TRUE;
      }
      else
      {
         nResult = FALSE;
      }
   }

   UartClearInput();

   return(nResult);
} /* XMStart */

/*** EOF ***/
