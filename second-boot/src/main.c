/**************************************************************************
*  Copyright (c) 2017-2023 by Michael Fischer (www.emb4fun.de)
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
*  21.08.2017  mifi  First Version.
**************************************************************************/
#define __MAIN_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "project.h"
#include "flash.h"
#include "xmodem.h"
#include "xboot.h"
#include "adler32.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/* Only 1MB for testing */
#define IMAGE_BUFFER_SIZE     (1 * 1024 * 1024)

#define AUTO_BOOT_TIMEOUT_MS  3000

#define TCM_START_ADDR        0x00000000
#define SDRAM_START_ADDR      0x90000000

typedef struct _image_list_
{
   char    *pName;
   uint32_t dAddr;
} image_list_t;

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static uint8_t *ImageBuffer = (uint8_t*)0x90000000;

static const image_list_t ImageList[] =
{
   { "XIP",      FLASH_USER_START_ADDR},
   { "TCM",      TCM_START_ADDR},
   { "SDRAM",    SDRAM_START_ADDR},
   { "2nd Boot", FLASH_BOOT_START_ADDR}
};

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  OutputBootMessage                                                    */
/*                                                                       */
/*  Output boot message.                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputBootMessage (void)
{
   const char ResetScreen[] = { 0x1B, 'c', 0 };

   term_printf("%s", ResetScreen);
   OSTimeDly(100);

   term_printf("\r\n");
   term_printf("*********************************\r\n");
   term_printf("  Project: %s\r\n", PROJECT_NAME);
   term_printf("  Board  : %s\r\n", PROJECT_BOARD);
   term_printf("  Version: v%s\r\n", PROJECT_VER_STRING);
   term_printf("  Build  : "__DATE__ " " __TIME__"\r\n");
   term_printf("*********************************\r\n");
   term_printf("\r\n");

} /* OutputBootMessage */

/*************************************************************************/
/*  OutputUsageInfo                                                      */
/*                                                                       */
/*  Output usage information.                                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputUsageInfo (void)
{
   term_printf("*** Update Mode ***\r\n");
   term_printf("h: Output this info\r\n");
   term_printf("i: Display image info\r\n");
   term_printf("x: Start XIP image\r\n");
   term_printf("t: Start TCM image\r\n");
   term_printf("s: Start SDRAM image\r\n");
   term_printf("u: Start XModem upload\r\n");
   term_printf("r: Restart\r\n");
   term_printf("E: Erase !!! complete !!! flash\r\n");
   term_printf("F: Erase FPGA image\r\n");
   term_printf("X: Erase XIP image\r\n");
   term_printf("B: Erase 2nd bootloader image\r\n");
   term_printf("\r\n");

} /* OutputUsageInfo */

/*************************************************************************/
/*  DisplayImageInfo                                                     */
/*                                                                       */
/*  Output some image infos.                                             */
/*                                                                       */
/*  In    : dAddress                                                     */
/*  Out   : none                                                         */
/*  Return: TRUE / FALSE                                                 */
/*************************************************************************/
static int DisplayImageInfo (uint32_t dAddress)
{
   int          nRes = FALSE;
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
         term_printf(" StartAddress    : 0x%X\r\n", Header.dStartAddress);
         term_printf(" DataTotalSize   : %d bytes\r\n", Header.dDataTotalSize);
         term_printf(" DataCreationTime: %d\r\n", Header.dDataCreationTime);

         if (strlen((char*)Header.DataName) < XBOOT_DATA_NAME_SIZE)
         {
            term_printf(" DataName        : %s\r\n", Header.DataName);
         }

         nRes = TRUE;
      }
      else
      {
         term_printf(" Header CRC error\r\n");

         nRes = TRUE;
      }
   }
   else
   {
      nRes = FALSE;
   }

   return(nRes);
} /* DisplayImageInfo */

/*************************************************************************/
/*  EraseFlashImage                                                      */
/*                                                                       */
/*  Erase the image in the Flash.                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TRUE / FALSE                                                 */
/*************************************************************************/
static int EraseFlashImage (uint32_t dAddress)
{
   int         nRes = FALSE;
   XBOOT_HEADER Header;

   dAddress = FlashIsImageAvailable(dAddress);
   if (dAddress != FLASH_ERROR)
   {
      nRes = TRUE;

      /* Convert address to SPI address */
      dAddress = dAddress - FLASH_START_ADDR;

      /* Read XBOOT header */
      spi_FlashRead(dAddress, (uint8_t*)&Header, sizeof(Header));

      term_printf("Erase flash image ");
      FlashEraseArea(Header.dStartAddress, Header.dDataTotalSize + sizeof(Header));
      term_printf(" done\r\n");
      term_printf("\r\n");
   }

   return(nRes);
} /* EraseFlashImage */

/*************************************************************************/
/*  CheckingFPGAImage                                                    */
/*                                                                       */
/*  Check if the CRC of the FPGA data is valid.                          */
/*                                                                       */
/*  In    : dDataSize, dDataCRC                                          */
/*  Out   : none                                                         */
/*  Return: TRUE / FALSE                                                 */
/*************************************************************************/
static int CheckingFPGAImage (uint32_t dDataSize, uint32_t dDataCRC)
{
   int      nRes = FALSE;
   uint32_t dAddress;
   uint32_t dCRC;

   term_printf("Checking FPGA image ");

   dAddress = 0;

   dCRC = ADLER_START_VALUE;
   while(dDataSize)
   {
      if (dDataSize >= SPI_FLASH_SECTOR_SIZE)
      {
         spi_FlashRead(dAddress, (uint8_t*)ImageBuffer, SPI_FLASH_SECTOR_SIZE);

         dCRC = adler32(dCRC, (const uint8_t *)ImageBuffer, SPI_FLASH_SECTOR_SIZE);

         dAddress  += SPI_FLASH_SECTOR_SIZE;
         dDataSize -= SPI_FLASH_SECTOR_SIZE;
      }
      else
      {
         spi_FlashRead(dAddress, (uint8_t*)ImageBuffer, dDataSize);

         dCRC = adler32(dCRC, (const uint8_t *)ImageBuffer, dDataSize);

         dAddress  += dDataSize;
         dDataSize -= dDataSize;
      }
      term_printf(".");
   }

   /* Check CRC of the data */
   if (dCRC == dDataCRC)
   {
      nRes = TRUE;
      term_printf(" OK\r\n");
   }
   else
   {
      term_printf(" ERROR\r\n");
   }

   return(nRes);
} /* CheckingFPGAImage */

/*************************************************************************/
/*  CheckingCRCofFlashImage                                              */
/*                                                                       */
/*  Check if the CRC of the Flash image is valid.                        */
/*                                                                       */
/*  In    : dAddress                                                     */
/*  Out   : none                                                         */
/*  Return: TRUE / FALSE                                                 */
/*************************************************************************/
static int CheckingCRCofFlashImage (uint32_t dAddress)
{
   int         nRes = FALSE;
   uint32_t    dCRC;
   XBOOT_HEADER Header;
   uint32_t    dDataSize;

   /* Convert address to SPI address */
   dAddress = dAddress - FLASH_START_ADDR;

   term_printf("Checking flash image");

   /* Read XBOOT header */
   spi_FlashRead(dAddress, (uint8_t*)&Header, sizeof(Header));

   /* Check header */
   if ((XBOOT_HEADER_MAGIC_1 == Header.dMagic1) &&
       (XBOOT_HEADER_MAGIC_2 == Header.dMagic2) &&
       (XBOOT_HEADER_SIZEVER == Header.dSizeVersion))
   {
      /* Check CRC of the header */
      dCRC = adler32(ADLER_START_VALUE, (const uint8_t *)&Header, sizeof(XBOOT_HEADER) - XBOOT_SIZE_OF_CRC32);
      if (dCRC == Header.dHeaderCRC32)
      {
         /* The header is valid, read the data into the ImageBuffer */
         dDataSize = Header.dDataTotalSize;
         dAddress  = dAddress + sizeof(XBOOT_HEADER);

         dCRC = ADLER_START_VALUE;
         while(dDataSize)
         {
            if (dDataSize >= SPI_FLASH_SECTOR_SIZE)
            {
               spi_FlashRead(dAddress, (uint8_t*)ImageBuffer, SPI_FLASH_SECTOR_SIZE);

               dCRC = adler32(dCRC, (const uint8_t *)ImageBuffer, SPI_FLASH_SECTOR_SIZE);

               dAddress  += SPI_FLASH_SECTOR_SIZE;
               dDataSize -= SPI_FLASH_SECTOR_SIZE;
            }
            else
            {
               spi_FlashRead(dAddress, (uint8_t*)ImageBuffer, dDataSize);

               dCRC = adler32(dCRC, (const uint8_t *)ImageBuffer, dDataSize);

               dAddress  += dDataSize;
               dDataSize -= dDataSize;
            }
            term_printf(".");
         }

         /* Check CRC of the data */
         if (dCRC == Header.dDataCRC32)
         {
            nRes = TRUE;
            term_printf(" OK\r\n");
         }
         else
         {
            term_printf(" ERROR\r\n");
         }
      }
   }
   else
   {
      term_printf("...ERROR\r\n");
   }

   return(nRes);
} /* CheckingCRCofFlashImage */

/*************************************************************************/
/*  HandleImageBuffer                                                    */
/*                                                                       */
/*  Check if the image buffer is valid.                                  */
/*  If the buffer is valid, flash the content.                           */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TRUE / FALSE                                                 */
/*************************************************************************/
static int HandleImageBuffer (void)
{
   int           nRes = FALSE;
   uint32_t      dSize = 0;
   uint32_t      dCRC;
   XBOOT_HEADER *pHeader;

   term_printf("Checking image header ... ");

   pHeader = (XBOOT_HEADER*)ImageBuffer;

   /* Check header */
   if ((XBOOT_HEADER_MAGIC_1 == pHeader->dMagic1) &&
       (XBOOT_HEADER_MAGIC_2 == pHeader->dMagic2) &&
       (XBOOT_HEADER_SIZEVER == pHeader->dSizeVersion))
   {
      /* Check CRC of the header */
      dCRC = adler32(ADLER_START_VALUE, (const uint8_t *)pHeader, sizeof(XBOOT_HEADER) - XBOOT_SIZE_OF_CRC32);
      if (dCRC == pHeader->dHeaderCRC32)
      {
         /* Header CRC is ok, check data CRC */
         term_printf("OK\r\n");

         term_printf("Checking image data ... ");
         dCRC = adler32(ADLER_START_VALUE, (const uint8_t *)&ImageBuffer[sizeof(XBOOT_HEADER)], pHeader->dDataTotalSize);
         if (dCRC == pHeader->dDataCRC32)
         {
            term_printf("OK\r\n");
            dSize = sizeof(XBOOT_HEADER) + pHeader->dDataTotalSize;
         }
         else
         {
            term_printf("ERROR\r\n");
         }
      }
      else
      {
         term_printf("ERROR\r\n");
      }
   }
   else
   {
      term_printf("ERROR\r\n");
   }

   if (dSize != 0)
   {
      /* Check for FPGA image update */
      if( (0 == pHeader->dStartAddress)                     && /* <= Image will be start at 0 */
          (strstr((char*)pHeader->DataName, ".rpd") != NULL) ) /* <= It was a .rpd file       */
      {
         term_printf("Erase FPGA image ");
         FlashEraseArea(FLASH_START_ADDR, dSize);
         term_printf(" done\r\n");

         dSize = dSize - sizeof(XBOOT_HEADER);
         term_printf("Program FPGA image ");
         FlashProgramArea(FLASH_START_ADDR, &ImageBuffer[sizeof(XBOOT_HEADER)], dSize);
         term_printf(" done\r\n");

         nRes = CheckingFPGAImage(pHeader->dDataTotalSize, pHeader->dDataCRC32);
         if (TRUE == nRes)
         {
            term_printf("\r\nFPGA ready to start\r\n");
         }
      }
      /* Check for XIP user area */
      else if ((pHeader->dStartAddress >= FLASH_USER_START_ADDR) && (pHeader->dStartAddress <= FLASH_END_ADDR))
      {
         term_printf("Erase flash image ");
         FlashEraseArea(pHeader->dStartAddress, dSize);
         term_printf(" done\r\n");

         term_printf("Program flash image ");
         FlashProgramArea(pHeader->dStartAddress, ImageBuffer, dSize);
         term_printf(" done\r\n");

         nRes = CheckingCRCofFlashImage(pHeader->dStartAddress);
         if (TRUE == nRes)
         {
            /* "Delete" header in SDRAM */
            ImageBuffer[0] = 0;

            term_printf("Image ready to start\r\n");
         }
      }
      /* Check for TCM */
      else if (TCM_START_ADDR == pHeader->dStartAddress)
      {
         nRes = TRUE;

         term_printf("Copy image... ");
         #pragma GCC diagnostic push
         #pragma GCC diagnostic ignored "-Wnonnull"

         memcpy((void*)TCM_START_ADDR, ImageBuffer, dSize);

         /* "Delete" header in SDRAM */
         ImageBuffer[0] = 0;

         #pragma GCC diagnostic pop

         term_printf("OK\r\n");
         term_printf("Image ready to start\r\n");
      }
      /* Check for SDRAM */
      else if (SDRAM_START_ADDR == pHeader->dStartAddress)
      {
         nRes = TRUE;

         /* Do nothing */
         term_printf("Image ready to start\r\n");
      }

   } /* end if (dSize != 0) */

   return(nRes);
} /* HandleImageBuffer */

/*************************************************************************/
/*  GetUpdateRequest                                                     */
/*                                                                       */
/*  Check if KEY[0] was pressed.                                         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TRUE / FALSE                                                 */
/*************************************************************************/
static uint8_t GetUpdateRequest (void)
{
   uint8_t  bRequest = FALSE;

   /* Check if KEY0 is pressed */
   if (neorv32_gpio_pin_get(0))
   {
      bRequest = TRUE;
   }

   return(bRequest);
} /* GetUpdateRequest */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  main                                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
int main (void)
{
   int      nResult;
   uint32_t dActualTime;
   uint32_t dStartAddr;
   uint8_t  bUpdateRequest;
   uint8_t  bStartXModem = FALSE;
   uint32_t dEndTime;

   /*
    * Initialize system, com and terminal functionality
    */
   SystemInit();

   /* Output startup messages */
   OutputBootMessage();

   /* Check if a manual update request is available */
   bUpdateRequest = GetUpdateRequest();

   /* Check if an image is available */
   dStartAddr = FlashIsImageAvailable(FLASH_USER_START_ADDR);

   /* Check if an image can be started */
   if ((FALSE == bUpdateRequest) && (FLASH_ERROR != dStartAddr))
   {
      int nData;

      term_printf("Autoboot in %ds. Press any key to abort.\r\n", (AUTO_BOOT_TIMEOUT_MS/1000));

      dEndTime = OSTimeGet() + AUTO_BOOT_TIMEOUT_MS;
      while (OSTimeGet() < dEndTime)
      {
         nData = COMGetChar();
         if (nData != EOF)
         {
            /* Stop "Autoboot" */
            break;
         }
      }

      /* Check for "Autoboot" */
      if (OSTimeGet() >= dEndTime)
      {
         FlashStartImage(dStartAddr);
      }
   }

   /* Check for update request */
   if (TRUE == bUpdateRequest)
   {
      term_printf("Update request, enter update mode\r\n");
   }
   else
   {
      /* Check for image error */
      if (FLASH_ERROR == dStartAddr)
      {
         bStartXModem = TRUE;
         term_printf("Image error\r\n");
         term_printf("No image available\r\n");
      }
   }

   term_printf("\r\n");


   /*
    * Check if XModem transfer must be started
    */
   if (TRUE == bStartXModem)
   {
      term_printf("Waiting to receive an image, press RETURN to abort\r\n");
      nResult = XMStart(ImageBuffer, IMAGE_BUFFER_SIZE);
      term_printf("\r\n");
      if (TRUE == nResult)
      {
         term_printf("\r\nTransfer finished\r\n");
         HandleImageBuffer();
      }
      else
      {
         term_printf("\r\nTransfer aborted\r\n");
      }
      term_printf("\r\n");
   }

   /*
    * Update Mode
    */
   OutputUsageInfo();

   while (1)
   {
      dActualTime = OSTimeGet();

      term_Task(dActualTime);
   }


  /*
   * This return here make no sense.
   * But to prevent the compiler warning:
   * "return type of 'main' is not 'int'
   * we use an int as return :-)
   */
  return(0);
} /* main */

/*************************************************************************/
/*  term_RxCallback                                                      */
/*                                                                       */
/*  Will be called from TermTask in case a char is received.             */
/*                                                                       */
/*  In    : bData                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void term_RxCallback (uint8_t bData)
{
   int nResult;

   switch (bData)
   {
      case 'h':   /* Help */
      {
         OutputUsageInfo();
         break;
      }

      /****************************************************/

      case 'i':   /* Display image info */
      {
         uint8_t bIndex;

         for (bIndex = 0; bIndex < sizeof(ImageList) / sizeof(image_list_t); bIndex++)
         {
            term_printf("%s image info:\r\n", ImageList[bIndex].pName);
            if (FALSE == DisplayImageInfo(ImageList[bIndex].dAddr))
            {
               term_printf(" No %s image available\r\n", ImageList[bIndex].pName);
            }
            term_printf("\r\n");
         }
         break;
      }

      case 'x':   /* Start XIP image */
      {
         uint32_t dAddress = FlashIsImageAvailable(FLASH_USER_START_ADDR);

         if (dAddress != FLASH_ERROR)
         {
            FlashStartImage(dAddress);
         }
         else
         {
            term_printf("No XIP image available\r\n");
         }
         term_printf("\r\n");
         break;
      }

      case 't':   /* Start TCM image */
      {
         uint32_t dAddress = FlashIsImageAvailable(TCM_START_ADDR);

         if (dAddress != FLASH_ERROR)
         {
            FlashStartImage(dAddress);
         }
         else
         {
            term_printf("No TCM image available\r\n");
         }
         term_printf("\r\n");
         break;
      }

      case 's':   /* Start SDRAM image */
      {
         uint32_t dAddress = FlashIsImageAvailable(SDRAM_START_ADDR);

         if (dAddress != FLASH_ERROR)
         {
            FlashStartImage(dAddress);
         }
         else
         {
            term_printf("No SDRAM image available\r\n");
         }
         term_printf("\r\n");
         break;
      }

      case 'u':   /* Start XModem upload */
      {
         term_printf("Waiting to receive an image, press RETURN to abort\r\n");
         nResult = XMStart(ImageBuffer, IMAGE_BUFFER_SIZE);
         term_printf("\r\n");
         if (TRUE == nResult)
         {
            term_printf("\r\nTransfer finished\r\n");
            HandleImageBuffer();
         }
         else
         {
            term_printf("\r\nTransfer aborted\r\n");
         }
         term_printf("\r\n");
         break;
      }

      case 'r':   /* Restart */
      {
         if (neorv32_wdt_available() != 0)
         {
            neorv32_wdt_setup((NEORV32_SYSINFO->CLK / 4096), 0, 0, 1, 1);
         }
         break;
      }

      case 'E':   /* Erase !!! complete !!! flash */
      {
         term_printf("Flash complete erase\r\n");
         FlashEraseAll();
         term_printf("Flash erase done\r\n");
         term_printf("\r\n");
         break;
      }

      case 'F':   /* Erase FPGA image */
      {
         term_printf("Erase FPGA image ");
         FlashEraseArea(FLASH_START_ADDR, FLASH_USER_START_ADDR-FLASH_START_ADDR);
         term_printf(" done\r\n");
         term_printf("\r\n");
         break;
      }

      case 'X':   /* Erase XIP image */
      {
         if (FALSE == EraseFlashImage(FLASH_USER_START_ADDR))
         {
            term_printf("No XIP image available\r\n");
            term_printf("\r\n");
         }
         break;
      }

      case 'B':   /* Erase 2nd bootloader image */
      {
         if (FALSE == EraseFlashImage(FLASH_BOOT_START_ADDR))
         {
            term_printf("No 2nd Boot image available\r\n");
            term_printf("\r\n");
         }
         break;
      }

      default:
      {
         /* Do nothing */
         break;
      }
   } /* end switch (bData) */

} /* term_RxCallback */

/*** EOF ***/
