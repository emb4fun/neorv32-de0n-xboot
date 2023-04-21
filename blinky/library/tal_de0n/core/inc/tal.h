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
#if !defined(__TAL_H__)
#define __TAL_H__

#if !defined(__CROSSWORKS_ARM) && !defined(__SES_ARM) && !defined(__SES_RISCV)
   Error: Wrong compiler ist used;
#endif 

/**************************************************************************
*  Includes
**************************************************************************/
#include "project.h"

#include "tal_conf.h"
#include "talboard.h"

#include "taltypes.h"
#include "talmisc.h"
#include "talcpu.h"
#include "talos.h"
#include "talgpio.h"
#include "talled.h"
#include "talcom.h"
#include "talcan.h"
#include "taldebug.h"
#include "talmem.h"

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * TAL version information
 */
#define TAL_CORE_VER_MAJOR    0
#define TAL_CORE_VER_MINOR_1  0
#define TAL_CORE_VER_MINOR_2  2

#define TAL_CORE_VER_NUMBER   (((uint32_t)TAL_CORE_VER_MAJOR   << 16) | \
                               ((uint32_t)TAL_CORE_VER_MINOR_1 <<  8) | \
                                (uint32_t)TAL_CORE_VER_MINOR_2)

#define TAL_CORE_VER_STRING   (XSTR(TAL_CORE_VER_MAJOR) "." \
                               XSTR(TAL_CORE_VER_MINOR_1)   \
                               XSTR(TAL_CORE_VER_MINOR_2))

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

void tal_Init (void);

#endif /* !__TAL_H__ */

/*** EOF ***/
