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
**************************************************************************/
#if !defined(__SYSTEM_NEORV32_H__)
#define __SYSTEM_NEORV32_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>
#include <stdarg.h>
#include "neorv32.h"

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * See Stringification:
 * https://gcc.gnu.org/onlinedocs/cpp/Stringification.html
 */
#define XSTR(s)   _STR(s)
#define _STR(s)   #s


/*-----------------------------------------------------------------------*/
/* "OS"                                                                  */
/*-----------------------------------------------------------------------*/
#define TICKS_PER_SECOND   1000
#define OS_MS_2_TICKS(_a)  ((_a * TICKS_PER_SECOND) / 1000)

/*
 * Macro to check timeout
 *
 *    a = actual time
 *    s = start time
 *    t = timeout
 */
#define OS_TEST_TIMEOUT(_a,_s,_t)   (((uint32_t)(_a) - (uint32_t)(_s)) >= (uint32_t)(_t))


/*-----------------------------------------------------------------------*/
/* Misc                                                                  */
/*-----------------------------------------------------------------------*/
#define TRUE   1
#define FALSE  0

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

/*
 * System
 */
void     SystemInit (void);
void     OSTimeDly (uint32_t dTimeoutMs);
uint32_t OSTimeGet (void);


/*
 * COM
 */
void COMSendBlock (uint8_t *pData, uint16_t wSize);
void COMSendChar (uint8_t bData);
int  COMGetChar (void);


/*
 * Terminal
 */
void term_Task (uint32_t dActualTime);
void term_RxCallback (uint8_t bData);
#define term_printf(...)   neorv32_uart_printf(NEORV32_UART0, __VA_ARGS__)
void term_puts (char *string);
int  term_putchar (int ch);

#endif /* !__SYSTEM_NEORV32_H__ */

/*** EOF ***/
