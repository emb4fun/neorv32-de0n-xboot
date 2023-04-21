/**************************************************************************
*  Copyright (c) 2017-2023 by Michael Fischer (www.emb4fun.de).
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
*  19.08.2017  mifi  First Version.
**************************************************************************/
#if !defined(__XBOOT_H__)
#define __XBOOT_H__

/**************************************************************************
*  Includes
**************************************************************************/
#ifdef _MSC_VER
#include <windows.h>
#else
#include <stdint.h>
#endif

/**************************************************************************
*  Global Definitions
**************************************************************************/

#ifdef _MSC_VER
typedef BYTE   uint8_t;
typedef DWORD  uint32_t;
#endif

#define XBOOT_MODE

#define XBOOT_HEADER_MAGIC_1  0x4F4F4258
#define XBOOT_HEADER_MAGIC_2  0x4F425854
#define XBOOT_HEADER_SIZEVER  0x00400001

#define XBOOT_SIZE_OF_CRC32   sizeof(uint32_t)

#define XBOOT_DATA_NAME_SIZE  32

/*
 * Total XBOOT header size should be 64 bytes
 */
typedef struct _xboot_header_
{
   uint32_t dMagic1;
   uint32_t dMagic2;
   uint32_t dSizeVersion;
   uint32_t dStartAddress;
   uint32_t dDataTotalSize;
   uint32_t dDataCreationTime;
   uint8_t   DataName[XBOOT_DATA_NAME_SIZE];
   uint32_t dDataCRC32;
   uint32_t dHeaderCRC32;
} XBOOT_HEADER;

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Funtions Definitions
**************************************************************************/

#endif /* !__XBOOT_H__ */

/*** EOF ***/
