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
#if !defined(__FLASH_H__)
#define __FLASH_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>
#include "spi_flash.h"

/**************************************************************************
*  Global Definitions
**************************************************************************/

#define FLASH_TOTAL_SIZE      SPI_FLASH_TOTAL_SIZE
#define FLASH_START_ADDR      0xE0000000
#define FLASH_END_ADDR        (FLASH_START_ADDR + FLASH_TOTAL_SIZE)

#define FLASH_USER_START_ADDR 0xE0100000
#define FLASH_BOOT_START_ADDR 0xE07E0000


/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

uint32_t FlashIsImageAvailable (uint32_t dAddress);
void     FlashStartImage (uint32_t dAddress);
void     FlashEraseAll (void);
void     FlashEraseArea (uint32_t dAddress, uint32_t dSize);
void     FlashProgramArea (uint32_t dAddress, uint8_t *pBuffer, uint32_t dSize);

#endif /* !__FLASH_H__ */

/*** EOF ***/
