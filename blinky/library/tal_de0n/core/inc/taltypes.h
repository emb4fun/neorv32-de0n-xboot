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
#if !defined(__TALTYPES_H__)
#define __TALTYPES_H__

/**************************************************************************
*  Includes
**************************************************************************/

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * Result of a TAL function
 */
typedef enum _tal_result_
{
   TAL_OK                     = 0,
   TAL_ERROR                  = 1,

   TAL_ERR_NULL_POINTER       = 2,  /* Null pointer was used */
   TAL_ERR_PARAMETER          = 3,  /* Wrong parameter */
   
   TAL_ERR_SEM_SIGNALED       = 4,  /* The semaphore was signaled */

   TAL_ERR_RING_EMPTY         = 10, /* Ring buffer is empty */
   TAL_ERR_RING_FULL          = 11, /* Ring buffer is full */

   TAL_ERR_COM_PORT_NOHW      = 20, /* Port is not supported by the board */
   TAL_ERR_COM_PORT_RANGE     = 21, /* Port is out of range  */
   TAL_ERR_COM_NOT_OPEN       = 22, /* Port is not open */
   TAL_ERR_COM_NOT_CLOSED     = 23, /* Port is not closed */
   TAL_ERR_COM_NO_RX_BUFF     = 24, /* RX ring buffer not set */
   TAL_ERR_COM_NO_TX_BUFF     = 25, /* TX ring buffer not set */
   TAL_ERR_COM_LENGTH         = 26, /* Word length not supported */
   TAL_ERR_COM_PARITY         = 27, /* Parity parameter not supported */
   TAL_ERR_COM_STOP           = 28, /* Stop bit parameter not supported */
   TAL_ERR_COM_BAUDRATE       = 29, /* Baud rate not supported */
   TAL_ERR_COM_EMPTY          = 30, /* No data available */
   TAL_ERR_COM_OVERFLOW       = 31, /* We have an RX overflow, but data available */
   TAL_ERR_COM_OVERFLOW_EMPTY = 32, /* Overflow and no data available */
   TAL_ERR_COM_IOCTL_NUM      = 33, /* IOCTL function number not supported */

   TAL_ERR_CAN_PORT_NOHW      = 40, /* Port is not supported by the board */
   TAL_ERR_CAN_PORT_RANGE     = 41, /* Port is out of range  */
   TAL_ERR_CAN_NOT_OPEN       = 42, /* Port is not open */
   TAL_ERR_CAN_NOT_CLOSED     = 43, /* Port is not closed */
   TAL_ERR_CAN_NO_RX_BUFF     = 44, /* RX ring buffer not set */
   TAL_ERR_CAN_NO_TX_BUFF     = 45, /* TX ring buffer not set */
   TAL_ERR_CAN_BAUDRATE       = 46, /* Baud rate not supported */
   TAL_ERR_CAN_INTERN_ACK     = 47,
   TAL_ERR_CAN_EMPTY          = 48, /* No data available */
   TAL_ERR_CAN_OVERFLOW       = 49, /* We have an RX overflow, but data available */
   TAL_ERR_CAN_OVERFLOW_EMPTY = 50, /* Overflow and no data available */
   TAL_ERR_CAN_ID_USED        = 51, /* ID still registered */
   TAL_ERR_CAN_ID_NOT_USED    = 52, /* ID not registered */
   TAL_ERR_CAN_ID_NO_ENTRY    = 53, /* No free filter entry available */ 
   TAL_ERR_CAN_ID_RANGE       = 54, /* ID is out of range */
   TAL_ERR_CAN_MSG_NOT_SEND   = 55, /* Message was not send */

   TAL_ERR_UNKNOWN
} TAL_RESULT;


/*
 * TRUE, FALSE and NULL
 */
#define TAL_TRUE     1
#define TAL_FALSE    0

#ifndef FALSE
#define FALSE        0
#endif

#ifndef TRUE
#define TRUE         1
#endif

#ifndef NULL
#define NULL         0
#endif


/*
 * Some magic numbers used to check for correct DCBs.
 */
typedef enum _tal_magic_
{
   TAL_MAGIC_ERROR = 0x00,
   TAL_MAGIC_COM,
   TAL_MAGIC_CAN
} TAL_MAGIC;

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

#endif /* !__TALTYPES_H__ */

/*** EOF ***/
