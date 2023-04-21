/**************************************************************************
*  Copyright (c) 2023 by Michael Fischer (www.emb4fun.de)
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
#define __FLASH_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "project.h"
#include "spi_flash.h"
#include "flash.h"
#include "xboot.h"
#include "adler32.h"
#include "neorv32.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/** Flash address bytes */
#define XIP_FLASH_ABYTES   3

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static uint8_t PageBuffer[SPI_FLASH_PAGE_SIZE];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  PageVerify                                                           */
/*                                                                       */
/*  Compare the buffer with the flash content.                           */
/*                                                                       */
/*  In    : dAddress, pBuffer                                            */
/*  Out   : none                                                         */
/*  Return: 0 = OK / all other ERROR                                     */
/*************************************************************************/
static int PageVerify (uint32_t dAddress, uint8_t *pBuffer)
{
   int       Error = 0;
   uint32_t dIndex;

   spi_FlashRead(dAddress, PageBuffer, SPI_FLASH_PAGE_SIZE);

   for (dIndex = 0; dIndex < SPI_FLASH_PAGE_SIZE; dIndex++)
   {
      if (PageBuffer[dIndex] != pBuffer[dIndex])
      {
         Error = -1;
         break;
      }
   }

   return(Error);
} /* PageVerify */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  FlashIsImageAvailable                                                */
/*                                                                       */
/*  Check if a image is available.                                       */
/*                                                                       */
/*  In    : dAddress                                                     */
/*  Out   : none                                                         */
/*  Return: Start address / FLASH_ERROR = Error                          */
/*************************************************************************/
uint32_t FlashIsImageAvailable (uint32_t dAddress)
{
   uint32_t    dCRC;
   XBOOT_HEADER Header;

   /* Convert address to SPI address */
   if ((dAddress >= FLASH_START_ADDR) && (dAddress <= FLASH_END_ADDR))
   {
      dAddress = dAddress - FLASH_START_ADDR;

      /* Read XBOOT header */
      spi_FlashRead(dAddress, (uint8_t*)&Header, sizeof(Header));
   }
   else
   {
      memcpy(&Header, (void*)dAddress, sizeof(Header));
   }

   /* Check header */
   if ((XBOOT_HEADER_MAGIC_1 == Header.dMagic1) &&
       (XBOOT_HEADER_MAGIC_2 == Header.dMagic2) &&
       (XBOOT_HEADER_SIZEVER == Header.dSizeVersion))
   {
      /* Check CRC of the header */
      dCRC = adler32(ADLER_START_VALUE, (const uint8_t *)&Header, sizeof(XBOOT_HEADER) - XBOOT_SIZE_OF_CRC32);
      if (dCRC == Header.dHeaderCRC32)
      {
         /* Valid CRC */
         dAddress = Header.dStartAddress;
      }
   }
   else
   {
      dAddress = FLASH_ERROR;
   }

   return(dAddress);
} /* FlashIsImageAvailable */

/*************************************************************************/
/*  FlashStartImage                                                      */
/*                                                                       */
/*  Start the flash image.                                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void FlashStartImage (uint32_t dAddress)
{
   #define SPI_FLASH_CMD_READ 0x03

   void (*AppEntry)(void);

   AppEntry = (void (*)(void))(dAddress + sizeof(XBOOT_HEADER));

   /* Disable machine-mode interrupts */
   neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

   /* Disable GPTMR FIRQ channel */
   neorv32_cpu_csr_clr(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);

   /*
    * Check for XIP start
    */
   if ((0 == NEORV32_XIP->CTRL) && (dAddress >= FLASH_USER_START_ADDR) && (dAddress < FLASH_END_ADDR))
   {
      // reset XIP module and configure basic SPI properties
      // * clock prescaler: CLK_PRSC_2
      // * clock mode 0 (cpol = 0, cpha = 0)
      // * flash read command = SPI_FLASH_CMD_READ
      // -> this function will also send 64 dummy clock cycles via the XIP's SPI port (with CS disabled)
      neorv32_xip_setup(CLK_PRSC_2, 0, 0, SPI_FLASH_CMD_READ);

      // Most SPI flash memories support "incremental read" operations - the read command and the start address
      // is only transferred once and after that consecutive data is sampled with each new transferred byte.
      // This can be sued by the XIP burst mode, which accelerates data fetch by up to 50%.
      neorv32_xip_burst_mode_enable(); // this has to be called right before starting the XIP mode by neorv32_xip_start()

      neorv32_xip_start(XIP_FLASH_ABYTES, FLASH_START_ADDR);
   }

   AppEntry(); /* Start application */

} /* FlashStartImage */

/*************************************************************************/
/*  FlashEraseAll                                                        */
/*                                                                       */
/*  Erase the flash user area.                                           */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void FlashEraseAll (void)
{
   uint16_t wCount;

   term_printf("  Flash erase 1 of %d\r", SPI_FLASH_TOTAL_SECTOR);
   for (wCount = 0; wCount < SPI_FLASH_TOTAL_SECTOR; wCount++)
   {
      spi_FlashEraseSector(wCount);

      term_printf("  Flash erase %d of %d\r", wCount+1, SPI_FLASH_TOTAL_SECTOR);
   }
   term_printf("\r\n");

} /* FlashEraseAll */

/*************************************************************************/
/*  FlashEraseArea                                                       */
/*                                                                       */
/*  Erase the given flash area.                                          */
/*                                                                       */
/*  In    : dAddress, dSize                                              */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void FlashEraseArea (uint32_t dAddress, uint32_t dSize)
{
   uint16_t wCount;
   uint16_t wSectorOffset;
   uint16_t wSectorCount;
   uint16_t wSectorNumber;

   if( (dAddress           >= FLASH_START_ADDR) &&
       ((dAddress + dSize) <= FLASH_END_ADDR)   )
   {
      dAddress      = dAddress - FLASH_START_ADDR;
      wSectorOffset = dAddress / SPI_FLASH_SECTOR_SIZE;
      wSectorCount  = dSize / SPI_FLASH_SECTOR_SIZE;

      for (wCount = 0; wCount < wSectorCount; wCount++)
      {
         wSectorNumber = wSectorOffset + wCount;

         spi_FlashEraseSector(wSectorNumber);

         term_printf(".");
      }
   }

} /* FlashEraseArea */

/*************************************************************************/
/*  FlashProgramArea                                                     */
/*                                                                       */
/*  Program the given flash area.                                        */
/*                                                                       */
/*  In    : dAddress, pBuffer, dSize                                     */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void FlashProgramArea (uint32_t dAddress, uint8_t *pBuffer, uint32_t dSize)
{
   int        Error;
   uint32_t  dIndex;
   uint32_t  dPageCount;

   if( (dAddress           >= FLASH_START_ADDR) &&
       ((dAddress + dSize) <= FLASH_END_ADDR)   )
   {
      dAddress   = dAddress - FLASH_START_ADDR;
      dPageCount = (dSize / SPI_FLASH_PAGE_SIZE);

      for (dIndex = 0; dIndex < dPageCount; dIndex++)
      {
         spi_FlashWrite(dAddress, pBuffer, SPI_FLASH_PAGE_SIZE);
         Error = PageVerify(dAddress, pBuffer);
         if (0 == Error)
         {
            term_printf(".");
         }
         else
         {
            term_printf(" Error ");
            break;
         }

         dAddress += SPI_FLASH_PAGE_SIZE;
         pBuffer  += SPI_FLASH_PAGE_SIZE;
      }
   }

} /* FlashProgramArea */

/*** EOF ***/
