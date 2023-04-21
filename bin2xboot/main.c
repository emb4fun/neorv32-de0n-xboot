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
#define __MAIN_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <windows.h>
#include <stdio.h>
#include <io.h>
#include <time.h>

#include "xboot.h"
#include "adler32.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define VERSION         "1.02"
#define MAX_IMAGE_SIZE  (7*1024*1024)

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static char szTemp[_MAX_PATH];

static char szInFile[_MAX_PATH];       /* Input file name */
static char szInFilePath[_MAX_PATH];
static char szInFileName[_MAX_PATH];

static char szOutFile[_MAX_PATH];      /* Output file name */
static DWORD dAlignment  = 0;          /* Alignment */
static DWORD dStartAddr  = 0;          /* Start address */
static DWORD dSectorSize = 0;

static BYTE   InImage[MAX_IMAGE_SIZE];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  OutputStartMessage                                                   */
/*                                                                       */
/*  Output start message.                                                */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputStartMessage (void)
{
   printf("\n");
   printf("bin2xboot v%s compiled "__DATE__" "__TIME__"\n", VERSION);
   printf("(c) 2023 by Michael Fischer (www.emb4fun.de)\n");
   printf("\n");
} /* OutputStartMessage */

/*************************************************************************/
/*  OutputUsage                                                          */
/*                                                                       */
/*  Output "usage" message.                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputUsage (void)
{
  printf("Usage: bin2xboot -i Infile -s Start [-a Alignment] [-o Outfile] [-v]\n");
  printf("\n");
  printf("  -i   Input file, e.g. -i input.bin\n");
  printf("  -s   Start address, e.g. -s 0x40100000\n");
  printf("  -a   Size alignment in KB, e.g. -a 4 (default 4)\n");
  printf("  -o   Output file, e.g. -o:output.xbo\n");
  printf("  -v   Show version information only\n");

} /* OutputUsage */

/*************************************************************************/
/*  CreateOutputImage                                                    */
/*                                                                       */
/*  Create the output image file.                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void CreateOutputImage (void)
{
   FILE        *hInFile;
   FILE        *hOutFile;
   DWORD        dInFileSize;
   DWORD        dDataSize;
   DWORD        dTotalSize;
   DWORD        dWriteCnt;
   XBOOT_HEADER  Header;
   LONG         lUnixtime;


   /*
    * Read InFile image
    */

   /* Check if input file is available */
   hInFile = fopen(szInFile, "rb");
   if (NULL == hInFile)
   {
      printf("Error: Input file <%s> not found\n", szInFile);
      exit(1);
   }

   /* Get size of input file */
   fseek(hInFile, 0, SEEK_END);
   dInFileSize = ftell(hInFile);
   fseek(hInFile, 0, SEEK_SET);

   /* Read input file data */
   if (dInFileSize > MAX_IMAGE_SIZE)
   {
      printf("Error: Input file size > %d\n", MAX_IMAGE_SIZE);
      fclose(hInFile);
      exit(1);
   }

   /* "Clear" InImage */
   memset(InImage, 0xFF, sizeof(InImage));

   /* Read input file */
   fread(InImage, 1, dInFileSize, hInFile);

   /* Close input file */
   fclose(hInFile);


   /*
    * Create XBOOT header
    */

   /* The TotalSize must be SectorSize aligned */
   dTotalSize = sizeof(XBOOT_HEADER) + dInFileSize;
   dTotalSize = (dTotalSize + (dSectorSize - 1)) / dSectorSize;
   dTotalSize = dTotalSize * dSectorSize;

   /* New data size */
   dDataSize = dTotalSize - sizeof(XBOOT_HEADER);

   printf("End of image  : 0x%08X\n", dStartAddr + dTotalSize);
   printf("Sector count  : %d\n", (dTotalSize / dSectorSize) );

   /* Get Unixtime */
   _tzset();
   time(&lUnixtime);

   /* Clear header */
   memset(&Header, 0x00, sizeof(Header));

   Header.dMagic1           = XBOOT_HEADER_MAGIC_1;
   Header.dMagic2           = XBOOT_HEADER_MAGIC_2;
   Header.dSizeVersion      = XBOOT_HEADER_SIZEVER;
   Header.dStartAddress     = dStartAddr;
   Header.dDataTotalSize    = dDataSize;
   Header.dDataCreationTime = lUnixtime;

   memcpy(Header.DataName, szInFileName, XBOOT_DATA_NAME_SIZE);
   Header.DataName[XBOOT_DATA_NAME_SIZE - 1] = 0;

   Header.dDataCRC32        = adler32(ADLER_START_VALUE, InImage, dDataSize);
   Header.dHeaderCRC32      = adler32(ADLER_START_VALUE, (char*)&Header, sizeof(XBOOT_HEADER) - XBOOT_SIZE_OF_CRC32);

   dTotalSize = sizeof(Header);


   /*
    * Write output image
    */

   /* Create output file */
   hOutFile = fopen(szOutFile, "wb");
   if (NULL == hOutFile)
   {
      /* File could not created */
      printf("Error: Could not create \"%s\"\n", szOutFile);
      exit(1);
   }

   dWriteCnt  = fwrite(&Header, sizeof(BYTE), sizeof(XBOOT_HEADER), hOutFile);
   dWriteCnt += fwrite(InImage, sizeof(BYTE), dDataSize, hOutFile);

   fclose(hOutFile);

   /* Check data count written to the disk */
   if (dWriteCnt != (sizeof(XBOOT_HEADER) + dDataSize))
   {
      /* Write error */
      printf("Error: Write error\n");
      DeleteFile(szOutFile);
      exit(1);
   }

   printf("\nOutput file \"%s\" was successful created, %d bytes\n", szOutFile, dWriteCnt);

} /* CreateOutputImage */

/*************************************************************************/
/*  SetDefaultOutFile                                                    */
/*                                                                       */
/*  Set default OutFile name.                                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void SetDefaultOutFile (void)
{
   int nSize;
   int nPos = 0;

   memcpy(szOutFile, szInFile, sizeof(szOutFile));

   /* Find the last '.' position */
   nSize = strlen(szOutFile);
   while(nSize > 0)
   {
      if (szOutFile[nSize] != '.')
      {
         nSize--;
      }
      else
      {
         nPos = nSize;
         break;
      }
   }

   if (0 == nPos)
   {
      /* Input filename has no extension */
      strcat(szOutFile, ".xbo");
   }
   else
   {
      szOutFile[nPos] = 0;
      strcat(szOutFile, ".xbo");
   }

} /* SetDefaultOutFile */

/*************************************************************************/
/*  GetInFileName                                                        */
/*                                                                       */
/*  Get the FileName without the path.                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void GetInFileName (void)
{
   int   nSize;
   char *pStart = NULL;

   memcpy(szTemp, szInFile, sizeof(szTemp));

   nSize = strlen(szTemp);
   while(nSize >= 0)
   {
      if (szTemp[nSize] != '\\')
      {
         nSize--;
      }
      else
      {
         nSize++;
         pStart = &szTemp[nSize];
         break;
      }
   }

   /* Clear szInFileName */
   memset(szInFileName, 0x00, sizeof(szInFileName));

   /* Copy filename if pStart is not NULL */
   if (pStart != NULL)
   {
      nSize = strlen(pStart);
      memcpy(szInFileName, pStart, nSize);
   }
   else
   {
      nSize = strlen(szInFile);
      memcpy(szInFileName, szInFile, nSize);
   }

} /* GetInFileName */

/*************************************************************************/
/*  GetInFilePath                                                        */
/*                                                                       */
/*  Find the InFile path if available.                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void GetInFilePath (void)
{
   int nSize;

   memcpy(szInFilePath, szInFile, sizeof(szInFilePath));

   nSize = strlen(szInFilePath);
   while(nSize >= 0)
   {
      if (szInFilePath[nSize] != '\\')
      {
         szInFilePath[nSize] = 0;
         nSize--;
      }
      else
      {
         break;
      }
   }

} /* GetInFilePath */

/*************************************************************************/
/*  CheckInFile                                                          */
/*                                                                       */
/*  Create if the "InFile" exist.                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void CheckInFile (void)
{
   FILE             *hInFile;
   LONG              hFind;
   struct _finddata_t fileinfo;


   /* Check for wildcard "?" */
   if (strstr(szInFile, "?"))
   {
      /* Wildcard use */
      GetInFilePath();

      hFind = _findfirst(szInFile, &fileinfo);
      if ((hFind > 0) && (fileinfo.attrib & 0x20))
      {
         _snprintf(szInFile, sizeof(szInFile), "%s%s", szInFilePath, fileinfo.name);
         _findclose(hFind);
      }
   }

   /* Check if input file is available */
   hInFile = fopen(szInFile, "rb");
   if (NULL == hInFile)
   {
      printf("Error: Input file <%s> not found\n", szInFile);
      exit(1);
   }

   fclose(hInFile);

   GetInFileName();

} /* CheckInFile */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  main                                                                 */
/*                                                                       */
/*  In    : argc, argv                                                   */
/*  Out   : none                                                         */
/*  Return: 0 / error cause                                              */
/*************************************************************************/
int main (int argc, char **argv)
{
   int   nIndex;
   int   nLen;
   int   nCmdUnknown   = 0;
   int   nCmdVersion   = 0;
   int   nCmdStartAddr = 0;
   char *pString;


   szInFile[0]  = 0;
   szOutFile[0] = 0;
   dAlignment   = 0;

   OutputStartMessage();

   /*
    * Check arguments if available
    */
   if (argc > 1)
   {
      for (nIndex=1; nIndex<argc; nIndex++)
      {
         /* Input file */
         if ((pString = strstr(argv[nIndex], "-i")) != NULL)
         {
            if ((nIndex + 1) < argc)
            {
               nIndex++;
               pString = argv[nIndex];
               nLen    = strlen(pString);
               if (nLen < (sizeof(szInFile)-1))
               {
                  strcpy(szInFile, pString);
               }
               else
               {
                  printf("Error: Input name len > %d\n", (sizeof(szInFile)-1));
                  exit(1);
               }
            }
         }
         /* Output file */
         else if ((pString = strstr(argv[nIndex], "-o")) != NULL)
         {
            if ((nIndex + 1) < argc)
            {
               nIndex++;
               pString = argv[nIndex];
               nLen    = strlen(pString);
               if (nLen < (sizeof(szOutFile)-1))
               {
                  strcpy(szOutFile, pString);
               }
               else
               {
                  printf("Error: Output name len > %d\n", (sizeof(szOutFile)-1));
                  exit(1);
               }
            }
         }
         /* Alignment */
         else if ((pString = strstr(argv[nIndex], "-a")) != NULL)
         {
            if ((nIndex + 1) < argc)
            {
               nIndex++;
               pString = argv[nIndex];
               dAlignment = atoi(pString);
            }
         }
         /* Start address */
         else if ((pString = strstr(argv[nIndex], "-s")) != NULL)
         {
            if ((nIndex + 1) < argc)
            {
               nCmdStartAddr = 1;

               nIndex++;
               pString    = argv[nIndex];
               dStartAddr = strtoul(pString, NULL, 16);
            }
         }
         /* Check version information only */
         else if (0 == strcmp(argv[nIndex], "-v"))
         {
            nCmdVersion = 1;
         }
         else
         {
            /* Ups, unknown command */
            nCmdUnknown = 1;
         }
      } /* end for loop */
   }
   else
   {
      /* Error, no argument */
      nCmdUnknown = 1;
   }

   /* Ups, found an unknown command */
   if (1 == nCmdUnknown)
   {
      OutputUsage();
      exit(0);
   }

   /* Version information requested */
   if (1 == nCmdVersion)
   {
      /* Only OutputStartMessage was needed */
      exit(0);
   }


   /*
    * Done analyzing the commands
    */

   /* Check if start address was set */
   if (0 == nCmdStartAddr)
   {
      printf("Error: No start address specified\n");
      exit(1);
   }

   /* Check if input file was set */
   if (0 == szInFile[0])
   {
      printf("Error: No input file specified\n");
      exit(1);
   }
   else
   {
      CheckInFile();
   }

   /* Check if out file option was set */
   if (0 == szOutFile[0])
   {
      /* Use default outfile name */
      SetDefaultOutFile();
   }

   /* Check alignment */
   if (0 == dAlignment)
   {
      dAlignment = 4;
   }
   else
   {
#if 0
      if ((dAlignment != 64) && (dAlignment != 128))
      {
         printf("Error: Alignment not supported, valid values are 64 or 128\n");
         exit(1);
      }
#endif
   }

   dSectorSize = (dAlignment * 1024);


   /*
    * Here the command line was parsed and no error was available
    */
   printf("Input file    : %s\n", szInFile);
   printf("Output file   : %s\n", szOutFile);
   printf("Alignment     : %d KB\n", dAlignment);
   printf("Start address : 0x%08X\n", dStartAddr);

   CreateOutputImage();

   return(0);
} /* main */

/*** EOF ***/
