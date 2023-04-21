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
#if !defined(__TALDEBUG_H__)
#define __TALDEBUG_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>
#include <stdarg.h>
#include "tal_conf.h"
#include "terminal.h"

/**************************************************************************
*  All Structures and Common Constants
**************************************************************************/

/*
 * TAL_DEBUG mask, 32 possible masks
 */ 
#define TAL_DBG_SYS     0x00000001
#define TAL_DBG_OS      0x00000002
#define TAL_DBG_TAL     0x00000004
#define TAL_DBG_IP      0x00000008
#define TAL_DBG_DHCP    0x00000010
#define TAL_DBG_DHCPs   0x00000020
#define TAL_DBG_SNTP    0x00000040
#define TAL_DBG_SNTPs   0x00000080
#define TAL_DBG_TNP     0x00000100
#define TAL_DBG_FS      0x00000200
#define TAL_DBG_WEB     0x00000400
#define TAL_DBG_PHY     0x04000000
#define TAL_DBG_NIC     0x80000000


/**************************************************************************
*  Macro Definitions
**************************************************************************/

/*
 * TAL_ASSERT, TAL_FAILED
 */
#if !defined(TAL_NO_ASSERT)
#define TAL_ASSERT(_a)  ((_a) ? (void)0 : TAL_FATAL(__func__, __FILE__, __LINE__, #_a))
#define TAL_FAILED()    TAL_FATAL(__func__, __FILE__, __LINE__, NULL)

void    TAL_FATAL(const char *, const char *, int, const char *);
#else
#define TAL_ASSERT(_a)
#define TAL_FAILED()
#endif

/*
 * TAL_PANIC
 */
#if !defined(TAL_NO_PANIC)
#define TAL_PANIC    term_printf
#else
#define TAL_PANIC(...)
#endif

/*
 * TAL_PRITNF
 */
#define tal_printf   term_printf
 
#if !defined(TAL_NO_PRINTF)
#define TAL_PRINTF   term_printf
#else
#define TAL_PRINTF(...)
#endif

/*
 * TAL_DEBUG
 */
#if !defined(TAL_NO_DEBUG)
void tal_Debug (uint32_t dMask, const char *fmt, ...);
#define TAL_DEBUG    tal_Debug
#else
#define TAL_DEBUG(...)
#endif

/**************************************************************************
*  Functions Definitions
**************************************************************************/

void tal_DebugMaskSet (uint32_t dMask);
void tal_DebugMaskClr (uint32_t dMask);
void tal_Debug (uint32_t dMask, const char *fmt, ...);

#endif /* !__TALDEBUG_H__ */

/*** EOF ***/
