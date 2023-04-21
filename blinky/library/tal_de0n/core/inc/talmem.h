/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2018-2023 by Michael Fischer (www.emb4fun.de).
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
#if !defined(__TALMEM_H__)
#define __TALMEM_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stddef.h>
#include <stdint.h>
#include "taltypes.h"

/**************************************************************************
*  Global Definitions
**************************************************************************/

typedef enum
{
   XM_ID_HEAP = 0,
   XM_ID_FS   = 1,
   XM_ID_IP   = 2,
   XM_ID_WEB  = 3,
   XM_ID_TLS  = 4,
   XM_ID_MBOX = 5,

   /****************/
   XM_ID_MAX = 16    /* <= Last element, do not use more than 16 */
} tal_mem_id;

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

void    tal_MEMInit (void);
void    tal_MEMAdd (tal_mem_id ID, const char *pName, void *pBuffer, uint32_t dSize);
void    tal_MEMOutputMemoryInfo (void);
char   *tal_MEMInfoGet(uint16_t wIndex, uint32_t *pSize, uint32_t *pUsed, uint32_t *pFree, uint32_t *pPeak, uint16_t *pNext);

int32_t tal_MEMGetUsedRawMemory (tal_mem_id ID);
int32_t tal_MEMGetUsedRawMemoryMax (tal_mem_id ID);
void    tal_MEMClrUsedRawMemoryMax (tal_mem_id ID);

void   *xcalloc (tal_mem_id ID, size_t nobj, size_t size);
void   *xmalloc (tal_mem_id ID, size_t size);
void   *xrealloc (tal_mem_id ID, void *p, size_t size);
void    xfree (void *p);
 
char   *xstrdup(tal_mem_id ID, const char *s1); 
 
#endif /* !__TALMEM_H__ */

/*** EOF ***/
