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
*  12.02.2017  mifi  First Version for the BeagleBone Black (B3).
*  15.07.2017  mifi  Only the S25FL116K will be supported.
*                    Reference is made to Cypress datasheet:
*                    S25FL116K, S25FL132K, S25FL164K
*                    Cypress Semiconductor Corporation
*                    Document Number: 002-00497 Rev. *E
*                    Revised June 29, 2016
*  23.03.2023  mifi  Reworked for a DE0-Nano with NEORV32 XIP.
**************************************************************************/
#if !defined(__SPI_FLASH_H__)
#define __SPI_FLASH_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>

/**************************************************************************
*  Global Definitions
**************************************************************************/

#define SPI_FLASH_TOTAL_SIZE     (8 * 1024 * 1024)
#define SPI_FLASH_SECTOR_SIZE    (4 * 1024)
#define SPI_FLASH_PAGE_SIZE      256
#define SPI_FLASH_TOTAL_SECTOR   (SPI_FLASH_TOTAL_SIZE / SPI_FLASH_SECTOR_SIZE)

#define SPI_FLASH_START_ADDR     0x00000000
#define SPI_FLASH_END_ADDR       (SPI_FLASH_START_ADDR + SPI_FLASH_TOTAL_SIZE)

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

int spi_Init (void);

int spi_FlashCheckID (void);
int spi_FlashEraseSector (uint32_t dSector);

int spi_FlashRead (uint32_t dAddress, uint8_t *pBuffer, uint32_t dSize);
int spi_FlashWrite (uint32_t dAddress, uint8_t *pBuffer, uint32_t dSize);

#endif /* !__SPI_FLASH_H__ */

/*** EOF ***/
