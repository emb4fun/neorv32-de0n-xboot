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

#define PRINT_TEXT(...) neorv32_uart0_puts(__VA_ARGS__)
#define PRINT_XNUM(a)   print_hex_word(a)
#define PRINT_GETC(a)   neorv32_uart0_getc()
#define PRINT_PUTC(a)   neorv32_uart0_putc(a)


#define IMAGE_AVAILABLE_STR   " image available\r\n"

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
   { "2nd Boot", FLASH_BOOT_START_ADDR}
};

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/**********************************************************************//**
 * Print 32-bit number as 8-digit hexadecimal value (with "0x" suffix).
 *
 * @param[in] num Number to print as hexadecimal.
 **************************************************************************/
void print_hex_word (uint32_t num)
{
   static const char hex_symbols[16] = "0123456789abcdef";

   PRINT_PUTC('0');
   PRINT_PUTC('x');

   for (int i=28; i>=0; i-=4)
   {
      PRINT_PUTC(hex_symbols[(num >> i) & 0xf]);
   }
} /* print_hex_word */

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

   PRINT_TEXT(ResetScreen);
   OSTimeDly(100);

   PRINT_TEXT("\r\n");
   PRINT_TEXT("*********************************\r\n");
   PRINT_TEXT("  Project: " PROJECT_NAME "\r\n");
   PRINT_TEXT("  Board  : " PROJECT_BOARD " \r\n");
   PRINT_TEXT("  Version: v" XSTR(PROJECT_VER_MAJOR) "." XSTR(PROJECT_VER_MINOR_1) XSTR(PROJECT_VER_MINOR_2) "\r\n");
   PRINT_TEXT("  Build  : "__DATE__ " " __TIME__"\r\n");
   PRINT_TEXT("*********************************\r\n");
   PRINT_TEXT("\r\n");

   /* Warning if i-cache is not implemented */
   if (0 == (NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_ICACHE)))
   {
      PRINT_TEXT("WARNING! No instruction cache implemented!\r\n");
      PRINT_TEXT("The XIP program might run very slow...\r\n");
      PRINT_TEXT("\r\n");
      OSTimeDly(2000);
   }

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
   PRINT_TEXT("*** Update Mode ***\r\n");
   PRINT_TEXT("h: Output this info\r\n");
   PRINT_TEXT("i: Display image info\r\n");
   PRINT_TEXT("x: Start XIP image\r\n");
   PRINT_TEXT("u: Start XModem upload\r\n");
   PRINT_TEXT("r: Restart\r\n");
   PRINT_TEXT("E: Erase !!! complete !!! flash\r\n");
   PRINT_TEXT("X: Erase XIP image\r\n");
   PRINT_TEXT("B: Erase 2nd bootloader image\r\n");

   PRINT_TEXT("\r\n");

} /* OutputUsageInfo */

/*************************************************************************/
/*  OutputsystemInfo                                                     */
/*                                                                       */
/*  Output system info.                                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputsystemInfo (void)
{
  PRINT_TEXT("HWV:  ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MIMPID));
  PRINT_TEXT("\nCID:  ");
  PRINT_XNUM(NEORV32_SYSINFO->CUSTOM_ID);
  PRINT_TEXT("\nCLK:  ");
  PRINT_XNUM(NEORV32_SYSINFO->CLK);
  PRINT_TEXT("\nMISA: ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MISA));
  PRINT_TEXT("\nXISA: ");
  PRINT_XNUM(neorv32_cpu_csr_read(CSR_MXISA));
  PRINT_TEXT("\nSOC:  ");
  PRINT_XNUM(NEORV32_SYSINFO->SOC);
  PRINT_TEXT("\nIMEM: ");
  PRINT_XNUM(NEORV32_SYSINFO->IMEM_SIZE); PRINT_TEXT(" bytes @");
  PRINT_XNUM(NEORV32_SYSINFO->ISPACE_BASE);
  PRINT_TEXT("\nDMEM: ");
  PRINT_XNUM(NEORV32_SYSINFO->DMEM_SIZE);
  PRINT_TEXT(" bytes @");
  PRINT_XNUM(NEORV32_SYSINFO->DSPACE_BASE);
  PRINT_TEXT("\n\n");

} /* OutputsystemInfo */

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
         PRINT_TEXT(" Header CRC error\r\n");

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
/*  Return: none                                                         */
/*************************************************************************/
static void EraseFlashImage (uint32_t dAddress)
{
   XBOOT_HEADER Header;

   dAddress = FlashIsImageAvailable(dAddress);
   if (dAddress != 0)
   {
      /* Convert address to SPI address */
      dAddress = dAddress - FLASH_START_ADDR;

      /* Read XBOOT header */
      spi_FlashRead(dAddress, (uint8_t*)&Header, sizeof(Header));

      PRINT_TEXT("Erase flash image ");
      FlashEraseArea(Header.dStartAddress, Header.dDataTotalSize + sizeof(Header));
      PRINT_TEXT(" done\r\n");
   }
   else
   {
      PRINT_TEXT("No image available\r\n");
   }

} /* EraseFlashImage */

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

   PRINT_TEXT("Checking flash image");

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
            PRINT_TEXT(" OK\r\n");
         }
         else
         {
            PRINT_TEXT(" ERROR\r\n");
         }
      }
   }
   else
   {
      PRINT_TEXT("...ERROR\r\n");
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

   PRINT_TEXT("Checking image header ... ");

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
         /* CRC is ok, check some values */
         if ((pHeader->dStartAddress + sizeof(XBOOT_HEADER) + pHeader->dDataTotalSize) < FLASH_END_ADDR)
         {
            /* Valid header */
            PRINT_TEXT("OK\r\n");

            PRINT_TEXT("Checking image data ... ");
            dCRC = adler32(ADLER_START_VALUE, (const uint8_t *)&ImageBuffer[sizeof(XBOOT_HEADER)], pHeader->dDataTotalSize);
            if (dCRC == pHeader->dDataCRC32)
            {
               PRINT_TEXT("OK\r\n");
               dSize = sizeof(XBOOT_HEADER) + pHeader->dDataTotalSize;
            }
            else
            {
               PRINT_TEXT("ERROR\r\n");
            }
         }
         else
         {
            PRINT_TEXT("ERROR\r\n");
         }
      }
      else
      {
         PRINT_TEXT("ERROR\r\n");
      }
   }
   else
   {
      PRINT_TEXT("ERROR\r\n");
   }

   if (dSize != 0)
   {
      if (0 == pHeader->dStartAddress)
      {
         // TODO
      }
      else
      {
         PRINT_TEXT("Erase flash image ");
         FlashEraseArea(pHeader->dStartAddress, dSize);
         PRINT_TEXT(" done\r\n");

         PRINT_TEXT("Program flash image ");
         FlashProgramArea(pHeader->dStartAddress, ImageBuffer, dSize);
         PRINT_TEXT(" done\r\n");

         nRes = CheckingCRCofFlashImage(pHeader->dStartAddress);
         if (TRUE == nRes)
         {
            PRINT_TEXT("Image ready to start\r\n");
         }
      }
   }

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
   SystemBootInit();

   /* Output startup messages */
   OutputBootMessage();

   /* Check if a manual update request is available */
   bUpdateRequest = GetUpdateRequest();
   if (TRUE == bUpdateRequest)
   {
      OutputsystemInfo();
   }

   /* First check if a 2nd Stage Bootloader image is available */
   dStartAddr = FlashIsImageAvailable(FLASH_BOOT_START_ADDR);
   if (0 == dStartAddr)
   {
      /* Check if a XIP image is available */
      dStartAddr = FlashIsImageAvailable(FLASH_USER_START_ADDR);
   }

   /* Check if an image can be started */
   if ((FALSE == bUpdateRequest) && (0 != dStartAddr))
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
      PRINT_TEXT("Update request, enter update mode\r\n");
   }
   else
   {
      /* Check for image error */
      if (0 == dStartAddr)
      {
         bStartXModem = TRUE;
         PRINT_TEXT("Image error\r\n");
         PRINT_TEXT("No image available\r\n");
      }
   }

   PRINT_TEXT("\r\n");


   /*
    * Check if XModem transfer must be started
    */
   if (TRUE == bStartXModem)
   {
      PRINT_TEXT("Waiting to receive an image, press RETURN to abort\r\n");
      nResult = XMStart(ImageBuffer, IMAGE_BUFFER_SIZE);
      PRINT_TEXT("\r\n");
      if (TRUE == nResult)
      {
         PRINT_TEXT("\r\nTransfer finished\r\n");
         HandleImageBuffer();
      }
      else
      {
         PRINT_TEXT("\r\nTransfer aborted\r\n");
      }
      PRINT_TEXT("\r\n");
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
            PRINT_TEXT(ImageList[bIndex].pName);
            PRINT_TEXT(" image info:\r\n");

            if (FALSE == DisplayImageInfo(ImageList[bIndex].dAddr))
            {
               PRINT_TEXT(" No ");
               PRINT_TEXT(ImageList[bIndex].pName);
               PRINT_TEXT(" image available\r\n");
            }
            PRINT_TEXT("\r\n");
         }
         break;
      }

      case 'x':   /* Start XIP image */
      {
         uint32_t dAddress = FlashIsImageAvailable(FLASH_USER_START_ADDR);

         if (dAddress != 0)
         {
            FlashStartImage(dAddress);
         }
         else
         {
            PRINT_TEXT("No image available\r\n");
         }
         PRINT_TEXT("\r\n");
         break;
      }

      case 'u':   /* Start XModem upload */
      {
         PRINT_TEXT("Waiting to receive an image, press RETURN to abort\r\n");
         nResult = XMStart(ImageBuffer, IMAGE_BUFFER_SIZE);
         PRINT_TEXT("\r\n");
         if (TRUE == nResult)
         {
            PRINT_TEXT("\r\nTransfer finished\r\n");
            HandleImageBuffer();
         }
         else
         {
            PRINT_TEXT("\r\nTransfer aborted\r\n");
         }
         PRINT_TEXT("\r\n");
         break;
      }

      case 'r':   /* Restart */
      {
         if (neorv32_wdt_available() != 0)
         {
            neorv32_wdt_setup((NEORV32_SYSINFO->CLK / 4096), 0, 0, 1);
         }
         break;
      }

      case 'E':   /* Erase !!! complete !!! flash */
      {
         PRINT_TEXT("Flash complete erase\r\n");
         FlashEraseAll();
         PRINT_TEXT("Flash erase done\r\n");
         PRINT_TEXT("\r\n");
         break;
      }

      case 'X':   /* Erase XIP image */
      {
         EraseFlashImage(FLASH_USER_START_ADDR);
         PRINT_TEXT("\r\n");
         break;
      }

      case 'B':   /* Erase 2nd bootloader image */
      {
         EraseFlashImage(FLASH_BOOT_START_ADDR);
         PRINT_TEXT("\r\n");
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
