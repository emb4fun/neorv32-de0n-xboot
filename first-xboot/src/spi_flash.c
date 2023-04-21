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
*  14.07.2019  mifi  Added support for B3cape01.
*  23.03.2023  mifi  Reworked for a DE0-Nano with NEORV32 XIP.
**************************************************************************/
#define __SPI_FLASH_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdint.h>
#include <string.h>
#include "system_neorv32.h"
#include "spi_flash.h"
#include "neorv32.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define CS_LOW()                 neorv32_spi_cs_en(0)
#define CS_HIGH()                neorv32_spi_cs_dis()

/*
 * JEDEC ID
 */
#define FLASH_ID_1               0x01
#define FLASH_ID_2               0x40
#define FLASH_ID_3               0x17

/*
 * Flash commands
 */
#define FLASH_CMD_WRITE_STATUS   0x01
#define FLASH_CMD_PAGE_PROGRAM   0x02
#define FLASH_CMD_FAST_READ      0x0B
#define FLASH_CMD_WRITE_ENABLE   0x06
#define FLASH_CMD_READ_STATUS    0x05
#define FLASH_CMD_SECTOR_ERASE   0x20
#define FLASH_CMD_READ_ID        0x9F

/*
 * Flash status register SR1
 */
#define FLASH_STATUS_BUSY                 0x01
#define FLASH_STATUS_BLOCK_PROTECT_MASK   0x1C

/*
 * Flash timeout
 */
#define FLASH_MAX_TIMEOUT_MS     500

/*
 * SendReceiveByte timeout
 */
#define SRB_TIMEOUT_MAX_CNT      1000

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  SendReceiveByte                                                      */
/*                                                                       */
/*  Send a byte by the SPI interface and return the byte received.       */
/*                                                                       */
/*  In    : bTxData, pRxData                                             */
/*  Out   : pRxData                                                      */
/*  Return: 0 = OK / -1 = NOK                                            */
/*************************************************************************/
static int SendReceiveByte (uint8_t bTxData, uint8_t *pRxData)
{
   int        Error = -1;
   uint32_t  dTimeout = 0;
   uint32_t  dStatus;

   NEORV32_SPI->DATA = (uint32_t)bTxData;

   dStatus = NEORV32_SPI->CTRL & (1<<SPI_CTRL_BUSY);
   while( (dStatus != 0) && (dTimeout++ < SRB_TIMEOUT_MAX_CNT) )
   {
      dStatus = NEORV32_SPI->CTRL & (1<<SPI_CTRL_BUSY);
   }

   if (dTimeout < SRB_TIMEOUT_MAX_CNT)
   {
      Error = 0;

      *pRxData = NEORV32_SPI->DATA;;
   }

   return(Error);
} /* SendReceiveByte */

/*************************************************************************/
/*  Name  : FlashWriteStatus                                             */
/*                                                                       */
/*  Write "bStatus" to the status register SR1. See page 67.             */
/*                                                                       */
/*  In    : bStatus                                                      */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void FlashWriteStatus (uint8_t bStatus)
{
   uint8_t bDummy;

   CS_LOW();

   SendReceiveByte(FLASH_CMD_WRITE_STATUS, &bDummy);
   SendReceiveByte(bStatus, &bDummy);

   CS_HIGH();

   (void)bDummy;

} /* FlashWriteStatus */

/*************************************************************************/
/*  Name  : FlashWriteEnable                                             */
/*                                                                       */
/*  Send a Write Enable command to sets the Write Enable Latch (WEL)     */
/*  bit in the Status Register to a 1. See page 66.                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void FlashWriteEnable (void)
{
   uint8_t bDummy;

   CS_LOW();

   SendReceiveByte(FLASH_CMD_WRITE_ENABLE, &bDummy);

   CS_HIGH();

} /* FlashWriteEnable */

/*************************************************************************/
/*  Name  : FlashReadStatus                                              */
/*                                                                       */
/*  Return the status byte SR1. See page 50.                             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: bStatus                                                      */
/*************************************************************************/
static uint8_t FlashReadStatus (void)
{
   uint8_t bStatus;

   CS_LOW();

   SendReceiveByte(FLASH_CMD_READ_STATUS, &bStatus);
   SendReceiveByte(0xFF, &bStatus);

   CS_HIGH();

   return(bStatus);
} /* FlashReadStatus */

/*************************************************************************/
/*  Name  : WaitNotBusy                                                  */
/*                                                                       */
/*  Wait for flash is not busy anymore.                                  */
/*                                                                       */
/*  In    : eDevice, wTimeoutMaxMs                                       */
/*  Out   : none                                                         */
/*  Return: 0 = OK / -1 = NOK                                            */
/*************************************************************************/
static int WaitNotBusy (uint32_t dTimeoutMs)
{
   int        Error = -1;
   uint8_t   bData;
   uint32_t  dTimeout = 0;

   bData = FlashReadStatus();
   while ((bData & FLASH_STATUS_BUSY) && (dTimeout < dTimeoutMs))
   {
      OSTimeDly(2);
      dTimeout = dTimeout + 2;
      bData = FlashReadStatus();
   }

   if (dTimeout < dTimeoutMs)
   {
      Error = 0;
   }

   return(Error);
} /* WaitNotBusy */

/*************************************************************************/
/*  SPIConfigure                                                         */
/*                                                                       */
/*  Configure the SPI interface of the board.                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void SPIConfigure (void)
{
   /* Disable and reset SPI module */
   neorv32_spi_disable();

   /* Enable and configure SPI controller */
   neorv32_spi_setup(CLK_PRSC_2, 1, 0, 0, 0);

} /* SPIConfigure */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  spi_Init                                                             */
/*                                                                       */
/*  Initialize the SPI interface of the board.                           */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: 0 = OK / -1 = NOK                                            */
/*************************************************************************/
int spi_Init (void)
{
   int rc;

   SPIConfigure();

   rc = spi_FlashCheckID();

   return(rc);
} /* spi_Init */

/*************************************************************************/
/*  spi_FlashCheckID                                                     */
/*                                                                       */
/*  Check the ID of the device.                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: 0 = OK / -1 = NOK                                            */
/*************************************************************************/
int spi_FlashCheckID (void)
{
   int        Error = -1;
   uint8_t   bDummy;
   uint8_t   bData1 = 0;
   uint8_t   bData2 = 0;
   uint8_t   bData3 = 0;

   CS_LOW();

   SendReceiveByte(FLASH_CMD_READ_ID, &bDummy);
   SendReceiveByte(0xFF, &bData1);
   SendReceiveByte(0xFF, &bData2);
   SendReceiveByte(0xFF, &bData3);

   CS_HIGH();

   /* Check JEDEC ID */
   if ((FLASH_ID_1 == bData1) && (FLASH_ID_2 == bData2) && (FLASH_ID_3 == bData3))
   {
      Error = 0;
   }

   return(Error);
} /* spi_FlashCheckID */

/*************************************************************************/
/*  spi_FlashEraseSector                                                 */
/*                                                                       */
/*  The Sector Erase command sets all memory within a specified sector   */
/*  (4 kbytes) to the erased state of all 1s (FFh). A Write Enable      */
/*  command must be executed before the device will accept the Sector    */
/*  Erase command (Status Register bit WEL must equal 1).                */
/*  See page 68.                                                         */
/*                                                                       */
/*  In    : dSectorNumber                                                */
/*  Out   : none                                                         */
/*  Return: 0 = OK / -1 = NOK                                            */
/*************************************************************************/
int spi_FlashEraseSector (uint32_t dSectorNumber)
{
   int       Error = -1;
   uint32_t dAddress;
   uint8_t  bData;
   uint8_t  bDummy;

   /*
    * Check parameter
    */
   if (dSectorNumber >= SPI_FLASH_TOTAL_SECTOR)
   {
      goto FlashEraseSectorEnd;
   }

   /*
    * Check if flash is busy
    */
   Error = WaitNotBusy(FLASH_MAX_TIMEOUT_MS);
   if (-1 == Error)
   {
      goto FlashEraseSectorEnd;
   }

   /*
    * Check if a "Block Protect Bit" is set
    */
   bData = FlashReadStatus();
   if (bData & FLASH_STATUS_BLOCK_PROTECT_MASK)
   {
      FlashWriteEnable();
      FlashWriteStatus(0); /* Clear "Block Protect Bit" */

      Error = WaitNotBusy(FLASH_MAX_TIMEOUT_MS);
      if (-1 == Error) goto FlashEraseSectorEnd;
   }

   /*
    * Erase sector
    */
   FlashWriteEnable();

   CS_LOW();

   /* Write Byte 1 */
   Error = SendReceiveByte(FLASH_CMD_SECTOR_ERASE, &bDummy);
   if (Error != 0) goto FlashEraseSectorEnd;

   /* Calculate "Sector Starting Address" */
   dAddress = dSectorNumber * SPI_FLASH_SECTOR_SIZE;

   /* Write Byte 2 */
   bData = (uint8_t)((dAddress >> 16) & 0xFF);
   Error = SendReceiveByte(bData, &bDummy);        /* A23-A16 */
   if (Error != 0) goto FlashEraseSectorEnd;

   /* Write Byte 3 */
   bData = (uint8_t)((dAddress >> 8) & 0xFF);
   Error = SendReceiveByte(bData, &bDummy);        /* A15-A8 */
   if (Error != 0) goto FlashEraseSectorEnd;

   /* Write Byte 4 */
   bData = (uint8_t)(dAddress & 0xFF);
   Error = SendReceiveByte(bData, &bDummy);        /* A7-A0 */
   if (Error != 0) goto FlashEraseSectorEnd;

   CS_HIGH();

   /*
    * Wait for end of erase process.
    *
    * After CS# is driven high, the self-timed Sector Erase command will
    * commence for a time duration of tSE. While the Sector Erase cycle is
    * in progress, the Read Status Register command may still be accessed
    * for checking the status of the BUSY bit. The BUSY bit is a 1 during
    * the Sector Erase cycle and becomes a 0 when the cycle is finished
    * and the device is ready to accept other commands again.
    */
   Error = WaitNotBusy(FLASH_MAX_TIMEOUT_MS);

FlashEraseSectorEnd:

   CS_HIGH();

   return(Error);
} /* spi_FlashEraseSector */

/*************************************************************************/
/*  spi_FlashRead                                                        */
/*                                                                       */
/*  Read data from the device. See page 72.                              */
/*                                                                       */
/*  In    : dAddress, pBuffer, dSize                                     */
/*  Out   : none                                                         */
/*  Return: 0 = OK / -1 = NOK                                            */
/*************************************************************************/
int spi_FlashRead (uint32_t dAddress, uint8_t *pBuffer, uint32_t dSize)
{
   int       Error = -1;
   uint8_t  bData;
   uint8_t  bDummy;

   /*
    * Check parameter
    */
   if ((NULL == pBuffer) || (0 == dSize) || ((dAddress + dSize) > SPI_FLASH_TOTAL_SIZE))
   {
      goto FlashReadEnd;
   }

   /*
    * Check if flash is busy
    */
   Error = WaitNotBusy(FLASH_MAX_TIMEOUT_MS);
   if (-1 == Error)
   {
      goto FlashReadEnd;
   }

   /*
    * Read flash
    */

   CS_LOW();

   /* Write Byte 1 */
   Error = SendReceiveByte(FLASH_CMD_FAST_READ, &bDummy);
   if (Error != 0) goto FlashReadEnd;

   /* Write Byte 2 */
   bData = (uint8_t)((dAddress >> 16) & 0xFF);
   Error = SendReceiveByte(bData, &bDummy);  /* A23-A16 */
   if (Error != 0) goto FlashReadEnd;

   /* Write Byte 3 */
   bData = (uint8_t)((dAddress >> 8) & 0xFF);
   Error = SendReceiveByte(bData, &bDummy);  /* A15-A8 */
   if (Error != 0) goto FlashReadEnd;

   /* Write Byte 4 */
   bData = (uint8_t)(dAddress & 0xFF);
   Error = SendReceiveByte(bData, &bDummy);  /* A7-A0 */
   if (Error != 0) goto FlashReadEnd;

   /* Write Byte 5 */
   Error = SendReceiveByte(0xFF, &bDummy);   /* Dummy */
   if (Error != 0) goto FlashReadEnd;

   /* Read Data1...DataN */
   while (dSize--)
   {
      Error = SendReceiveByte(0xFF, &bData);
      if (Error != 0) goto FlashReadEnd;

      *pBuffer++ = bData;
   }

FlashReadEnd:

   CS_HIGH();

   return(Error);
} /* spi_FlashRead */

/*************************************************************************/
/*  spi_FlashWrite                                                       */
/*                                                                       */
/*  The Page Program command allows from one byte to 256 bytes (a page)  */
/*  of data to be programmed at previously erased (FFh) memory locations.*/
/*  A Write Enable command must be executed before the device will accept*/
/*  the Page Program Command (Status Register bit WEL= 1). See page 68.  */
/*                                                                       */
/*  In    : dAddress, pBuffer, dSize                                     */
/*  Out   : none                                                         */
/*  Return: 0 = OK / -1 = NOK                                            */
/*************************************************************************/
int spi_FlashWrite (uint32_t dAddress, uint8_t *pBuffer, uint32_t dSize)
{
   int       Error = -1;
   uint8_t  bData;
   uint8_t  bDummy;

   /*
    * Check parameter
    */
   if ((NULL == pBuffer) || (0 == dSize) || ((dAddress + dSize) > SPI_FLASH_TOTAL_SIZE))
   {
      goto FlashWriteEnd;
   }

   /*
    * Check special write parameter, only page access
    */
   if ((dSize > SPI_FLASH_PAGE_SIZE) || ((dAddress % SPI_FLASH_PAGE_SIZE) != 0))
   {
      goto FlashWriteEnd;
   }

   /*
    * Check if flash is busy
    */
   Error = WaitNotBusy(FLASH_MAX_TIMEOUT_MS);
   if (-1 == Error)
   {
      goto FlashWriteEnd;
   }

   /*
    * Check if a "Block Protect Bit" is set
    */
   bData = FlashReadStatus();
   if (bData & FLASH_STATUS_BLOCK_PROTECT_MASK)
   {
      FlashWriteEnable();
      FlashWriteStatus(0); /* Clear "Block Protect Bit" */

      Error = WaitNotBusy(FLASH_MAX_TIMEOUT_MS);
      if (-1 == Error) goto FlashWriteEnd;
   }

   /*
    * Page Program
    */
   FlashWriteEnable();

   CS_LOW();

   /* Write Byte 1 */
   Error = SendReceiveByte(FLASH_CMD_PAGE_PROGRAM, &bDummy);
   if (Error != 0) goto FlashWriteEnd;

   /* Write Byte 2 */
   bData = (uint8_t)((dAddress >> 16) & 0xFF);
   Error = SendReceiveByte(bData, &bDummy);  /* A23-A16 */
   if (Error != 0) goto FlashWriteEnd;

   /* Write Byte 3 */
   bData = (uint8_t)((dAddress >> 8) & 0xFF);
   Error = SendReceiveByte(bData, &bDummy);  /* A15-A8 */
   if (Error != 0) goto FlashWriteEnd;

   /* Write Byte 4 */
   bData = (uint8_t)(dAddress & 0xFF);
   Error = SendReceiveByte(bData, &bDummy);  /* A7-A0 */
   if (Error != 0) goto FlashWriteEnd;

   /* Write Data1...DataN */
   while (dSize--)
   {
      Error = SendReceiveByte(*pBuffer++, &bDummy);
      if (Error != 0) goto FlashWriteEnd;
   }

   CS_HIGH();

   /*
    * Wait for end of erase process.
    *
    * After CS# is driven high, the self-timed Page Program command will
    * commence for a time duration of tPP. While the Page Program cycle is
    * in progress, the Read Status Register command may still be accessed
    * for checking the status of the BUSY bit. The BUSY bit is a 1 during
    * the Page Program cycle and becomes a 0 when the cycle is finished
    * and the device is ready to accept other commands again.
    */
   Error = WaitNotBusy(FLASH_MAX_TIMEOUT_MS);

FlashWriteEnd:

   CS_HIGH();

   return(Error);
} /* spi_FlashWrite */


/*** EOF ***/
