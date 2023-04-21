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

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define VERSION         "1.00"
#define MAX_IMAGE_SIZE  (32*1024)

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static char szTemp[_MAX_PATH];

static char szInFile[_MAX_PATH];       /* Input file name */
static char szInFilePath[_MAX_PATH];
static char szInFileName[_MAX_PATH];

static char szOutFile[_MAX_PATH];      /* Output file name */

static BYTE   InImage[MAX_IMAGE_SIZE];

static char szTempStr[1024];

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
   printf("bin2vhd v%s compiled "__DATE__" "__TIME__"\n", VERSION);
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
  printf("Usage: bin2vhd -i Infile [-o Outfile]\n");
  printf("\n");
  printf("  -i   Input file, e.g. -i input.bin\n");
  printf("  -o   Output file, e.g. -o:output.xbo\n");

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
   FILE      *hInFile;
   FILE      *hOutFile;
   DWORD      dInFileSize;
   DWORD      dInIndex;
   DWORD      dTmp;
   LONG       lUnixtime;
   struct tm *pTM;
   BYTE        Buffer[4];
   char        CompileTime[64];
   

   /*
    * Get local time
	*/
   time(&lUnixtime);
   pTM = localtime(&lUnixtime);

   _snprintf(CompileTime, sizeof(CompileTime), "%02d.%02d.%d %02d:%02d:%02d",
             pTM->tm_mday, pTM->tm_mon + 1, pTM->tm_year + 1900,
             pTM->tm_hour, pTM->tm_min, pTM->tm_sec);

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


   sprintf(szTempStr, "--\n"
	                  "-- The NEORV32 RISC-V Processor: https://github.com/stnolting/neorv32\n"
                      "-- Auto-generated memory initialization file (for BOOTLOADER) from source file <%s>\n"
                      "-- Size: %lu bytes\n"
					  "-- Built: %s\n"
					  "--\n"
                      "\n"
                      "-- prototype defined in 'neorv32_package.vhd'\n"
                      "package body neorv32_bootloader_image is\n"
                      "\n"
                      "constant bootloader_init_image : mem32_t := (\n", szInFile, dInFileSize, CompileTime);

   fwrite(szTempStr, sizeof(BYTE), strlen(szTempStr), hOutFile);

   dInIndex    = 0;
   dInFileSize = dInFileSize - 4;
   while (dInIndex < dInFileSize)
   {
      Buffer[0] = InImage[dInIndex + 0];
      Buffer[1] = InImage[dInIndex + 1];
      Buffer[2] = InImage[dInIndex + 2];
      Buffer[3] = InImage[dInIndex + 3];

      dTmp  = (DWORD)(Buffer[0] << 0);
      dTmp |= (DWORD)(Buffer[1] << 8);
      dTmp |= (DWORD)(Buffer[2] << 16);
      dTmp |= (DWORD)(Buffer[3] << 24);
      sprintf(szTempStr, "x\"%08x\",\n", (unsigned int)dTmp);
      fwrite(szTempStr, sizeof(BYTE), strlen(szTempStr), hOutFile);

      dInIndex += 4;
   }

   Buffer[0] = InImage[dInIndex + 0];
   Buffer[1] = InImage[dInIndex + 1];
   Buffer[2] = InImage[dInIndex + 2];
   Buffer[3] = InImage[dInIndex + 3];

   dTmp  = (DWORD)(Buffer[0] << 0);
   dTmp |= (DWORD)(Buffer[1] << 8);
   dTmp |= (DWORD)(Buffer[2] << 16);
   dTmp |= (DWORD)(Buffer[3] << 24);
   sprintf(szTempStr, "x\"%08x\"\n", (unsigned int)dTmp);
   fwrite(szTempStr, sizeof(BYTE), strlen(szTempStr), hOutFile);

   /* End */
   sprintf(szTempStr, ");\n"
                      "\n"
                      "end neorv32_bootloader_image;\n"
                      "\n"
					  "-- *** EOF ***"
                      "\n");

   fwrite(szTempStr, sizeof(BYTE), strlen(szTempStr), hOutFile);

   fclose(hOutFile);

   printf("\nOutput file \"%s\" was successful created\n", szOutFile);

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
      strcat(szOutFile, ".vhd");
   }
   else
   {
      szOutFile[nPos] = 0;
      strcat(szOutFile, ".vhd");
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
   int   nCmdUnknown = 0;
   int   nCmdVersion = 0;
   char *pString;


   szInFile[0]  = 0;
   szOutFile[0] = 0;

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

   /*
    * Here the command line was parsed and no error was available
    */
   printf("Input file    : %s\n", szInFile);
   printf("Output file   : %s\n", szOutFile);

   CreateOutputImage();

   return(0);
} /* main */

/*** EOF ***/
