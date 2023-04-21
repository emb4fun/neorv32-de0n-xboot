/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
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
#if !defined(__TALCPU_COM_H__)
#define __TALCPU_COM_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * The NEORV32 CPU supports up to 2 COM ports.
 */
typedef enum _tal_com_port_
{
   TAL_COM_PORT_1 = 0,  /* Must be start at 0 */
   TAL_COM_PORT_2,

   TAL_COM_PORT_MAX
} TAL_COM_PORT; 

/*
 * Hardware information
 */
typedef struct _tal_com_hw_
{
   uint32_t dBaseAddress;
   
   uint32_t dRXFirqEnable;
   uint32_t dTXFirqEnable;
   
   uint32_t dRXFirqPending;
   uint32_t dTXFirqPending;
   
   uint8_t  bTxFirqEnabled;
} TAL_COM_HW;

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

/*
 * Functions still defined in talcom.h
 */

#endif /* !__TALCPU_COM_H__ */

/*** EOF ***/
