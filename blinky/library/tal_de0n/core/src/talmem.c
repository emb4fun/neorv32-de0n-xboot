/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2018-2023 by Michael Fischer (www.emb4fun.de).
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
#define __TALMEM_C__

/*
 * How to Allocate Dynamic Memory Safely
 * See: https://barrgroup.com/Embedded-Systems/How-To/Malloc-Free-Dynamic-Memory-Allocation
 *
 * Here the free list "First Fit" allocate and "Address Order" free management is used.
 */

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include "tal.h"

/*=======================================================================*/
/*  All Extern definitions                                               */
/*=======================================================================*/

#if defined(TAL_HEAP_MEM1_START)
extern uint32_t TAL_HEAP_MEM1_START;
extern uint32_t TAL_HEAP_MEM1_END;
#endif

#if defined(TAL_HEAP_MEM2_START)
extern uint32_t TAL_HEAP_MEM2_START;
extern uint32_t TAL_HEAP_MEM2_END;
#endif

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*
 * Memory header
 */
typedef struct _mem_hdr_
{
   struct _mem_hdr_ *pNext;
   uint32_t          dSize;
   uint32_t          dListID;
   uint32_t          dObject;
} mem_hdr_t;   


/*
 * Memory context
 */
typedef struct _mem_ctx_
{
   const char  *pName;
   mem_hdr_t   *pFreelist;
   uint32_t     dSize;
   int32_t       UsedRawMemory;
   int32_t       UsedRawMemoryMax;
} mem_ctx_t;   


#define MEM_LIST_COUNT  16

#define MEM_ALIGN       16

#define OBJ_IS_FREE     0x00123456
#define OBJ_IN_USE      0xFFEEDDCC

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static OS_SEMA   Sema;
static mem_ctx_t MemList[MEM_LIST_COUNT];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  MEMMalloc                                                            */
/*                                                                       */
/*  Allocate memory of the given size.                                   */
/*  Returns NULL if memory cannot be allocated.                          */
/*                                                                       */
/*  In    : ID, dSize                                                    */
/*  Out   : none                                                         */
/*  Return: p / NULL                                                     */
/*************************************************************************/
static void *MEMMalloc (tal_mem_id ID, uint32_t dSize)
{
   mem_hdr_t *pFreelist = MemList[ID].pFreelist;
   void      *p = NULL;
   mem_hdr_t *pPrev;
   mem_hdr_t *pMem = NULL;
   mem_hdr_t *pTmp;

   TAL_CPU_DISABLE_ALL_INTS();
   
   if (pFreelist != NULL)
   {
      /* Align size to 16 */
      dSize  = (dSize + 15) & ~15;
      
      /* Added header info */
      dSize += sizeof(mem_hdr_t);
   
      /* Check first element for the same size */
      if (pFreelist->dSize == dSize)
      {
         pMem = pFreelist;
         pFreelist = pFreelist->pNext;
      }
      /* Check first element with bigger size */
      else if (pFreelist->dSize > (dSize + sizeof(mem_hdr_t)))
      {
         pMem = pFreelist;
         
         pFreelist = (mem_hdr_t*)((uint32_t)pFreelist + dSize);
         pFreelist->pNext = pMem->pNext;
         pFreelist->dSize = pMem->dSize - dSize;
      }
      else
      {
         /* Check free list chain */
      
         pPrev = pFreelist;
         pMem  = pFreelist->pNext;
         
         while (pMem != NULL)
         {
            /* Check for same size */
            if (pMem->dSize == dSize)
            {
               pPrev->pNext = pMem->pNext;
               break;
            }
            
            /* Check for bigger size */
            if (pMem->dSize > (dSize + sizeof(mem_hdr_t)))
            {
               pTmp = (mem_hdr_t*)((uint32_t)pMem + dSize);
               pTmp->pNext = pMem->pNext;
               pTmp->dSize = pMem->dSize - dSize;
               
               pPrev->pNext = pTmp;
               break;
            }
         
            pPrev = pMem;
            pMem  = pMem->pNext;
         }
      }
      
      
      if (pMem != NULL)
      {
         pMem->pNext = NULL;
         pMem->dSize = dSize;
         p = (void*)((uint32_t)pMem + sizeof(mem_hdr_t));
      }
   }
   
   if (p != NULL)
   {
      MemList[ID].UsedRawMemory += (int32_t)dSize;
      
      if (MemList[ID].UsedRawMemory > MemList[ID].UsedRawMemoryMax)
      {
         MemList[ID].UsedRawMemoryMax = MemList[ID].UsedRawMemory;
      }
   }

   MemList[ID].pFreelist = pFreelist;

   /* Set ID */
   if (p != NULL)
   {
      pMem = (mem_hdr_t*)((uint32_t)p - sizeof(mem_hdr_t));   
      
      pMem->dListID = (uint32_t)ID;
      pMem->dObject = OBJ_IN_USE;
   }

   TAL_CPU_ENABLE_ALL_INTS();
   
   return(p);
} /* MEMMalloc */

/*************************************************************************/
/*  MEMCalloc                                                            */
/*                                                                       */
/*  Allocate memory of the given size, and initialize them to zero.      */
/*  Returns NULL if memory cannot be allocated.                          */
/*                                                                       */
/*  In    : ID, dSize                                                    */
/*  Out   : none                                                         */
/*  Return: p / NULL                                                     */
/*************************************************************************/
static void *MEMCalloc (tal_mem_id ID, uint32_t dSize)
{
   void *p;
   
   p = MEMMalloc(ID, dSize);
   if (p != NULL)
   {
      memset(p, 0, dSize);
   }
   
   return(p);
} /* MEMCalloc */

/*************************************************************************/
/*  MEMFree                                                              */
/*                                                                       */
/*  Frees the allocated memory.                                          */
/*                                                                       */
/*  In    : pBuffer                                                      */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void MEMFree (void *pBuffer)
{
   mem_hdr_t *pFreelist;
   mem_hdr_t *pPrev;
   mem_hdr_t *pNext;
   mem_hdr_t *pMem;
   uint32_t   dListID;

   TAL_CPU_DISABLE_ALL_INTS();

   if (pBuffer != NULL)
   {
      /* Get memory header */
      pMem = (mem_hdr_t*)((uint32_t)pBuffer - sizeof(mem_hdr_t));

      /* Check for a valid memory object */
      if (pMem->dObject != OBJ_IN_USE)
      {
         /* This is not a memory object which is in used */
         goto MEMFreeEnd;  /*lint !e801*/
         //return;
      }

      /* Get list ID */
      dListID = pMem->dListID;
      if (dListID >= MEM_LIST_COUNT)
      {
         /* Wrong list ID */
         goto MEMFreeEnd;  /*lint !e801*/
         //return;
      }

      /* Set free marker */   
      pMem->dObject = OBJ_IS_FREE;

      pFreelist = MemList[dListID].pFreelist;

      MemList[dListID].UsedRawMemory -= (int32_t)pMem->dSize;

      /* Check empty free list */
      if (NULL == pFreelist)
      {
         pFreelist = pMem;
         pFreelist->pNext = NULL;
      }
      /* Check in front of the free list, without space, mem + list */
      else if (((uint32_t)pMem + pMem->dSize) == (uint32_t)pFreelist)
      {
         pMem->pNext = pFreelist->pNext;
         pMem->dSize = pFreelist->dSize + pMem->dSize;
         pFreelist = pMem; 
      }
      /* Check in front of the free list, with space, mem |space| list */
      else if (((uint32_t)pMem + pMem->dSize) < (uint32_t)pFreelist)
      {
         pMem->pNext = pFreelist;
         pFreelist   = pMem;
      }
      else
      {
         /* Find correct position */
         pPrev = pFreelist;
         pNext = pFreelist->pNext;
         
         while ((pNext != NULL) && ((uint32_t)pNext < (uint32_t)pMem))
         {
            pPrev = pNext;
            pNext = pNext->pNext;
         }
         
         if (pNext != NULL)
         {
            /* Added in between pPrev and pNext */

            /* Check for prev + mem + next */
            if( (((uint32_t)pPrev + pPrev->dSize) == (uint32_t)pMem)  &&
                (((uint32_t)pMem  + pMem->dSize)  == (uint32_t)pNext) )
            {
               pPrev->pNext = pNext->pNext;
               pPrev->dSize = pPrev->dSize + pMem->dSize + pNext->dSize;
            }
            /* Check for prev + mem |space| next */                   
            else if (((uint32_t)pPrev + pPrev->dSize) == (uint32_t)pMem)
            {
               pPrev->pNext = pNext;
               pPrev->dSize = pPrev->dSize + pMem->dSize;
            }
            /* Check for prev |space| mem + next */                   
            else if (((uint32_t)pMem + pMem->dSize) == (uint32_t)pNext)
            {
               pPrev->pNext = pMem;
               pMem->pNext  = pNext->pNext;
               pMem->dSize  = pMem->dSize + pNext->dSize;
            }
            else
            {
               /* prev |space| mem |space| next */
               pMem->pNext  = pPrev->pNext;
               pPrev->pNext = pMem;
            }
         }
         else
         {
            /* Added at the end of the free list, prev */
            
            /* Check for prev + mem */                   
            if (((uint32_t)pPrev + pPrev->dSize) == (uint32_t)pMem)
            {
               pPrev->pNext = NULL;
               pPrev->dSize = pPrev->dSize + pMem->dSize;
            }
            else
            {
               /* prev |space| mem */
               pPrev->pNext = pMem;
               pMem->pNext  = NULL;
            }
         }
      }

      MemList[dListID].pFreelist = pFreelist;

   } /* end if (pBuffer != NULL) */
   
MEMFreeEnd:   
   
   TAL_CPU_ENABLE_ALL_INTS();

} /* MEMFree */

/*************************************************************************/
/*  MEMAdd                                                               */
/*                                                                       */
/*  Add the memroy to the dynamic memory system.                         */
/*                                                                       */
/*  In    : ID, pBuffer, dSize                                           */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void MEMAdd (tal_mem_id ID, void *pBuffer, uint32_t dSize)
{
   uint32_t   dAddress;
   uint32_t   dRest;
   mem_hdr_t *pMem;
   
   /* Align address to 16 */
   dAddress  = (uint32_t)pBuffer;
   dRest     = dAddress % MEM_ALIGN;
   
   if ((dRest != 0) && (dSize > 32))
   {
      dRest     = MEM_ALIGN - dRest;
      dAddress += dRest;
      dSize    -= dRest;
   }
   
   if (dSize > 32)
   {
      /* Align size to 16 */
      dRest  = dSize % MEM_ALIGN;
      dSize -= dRest;

      MemList[ID].dSize += dSize;
   
      pMem = (mem_hdr_t*)dAddress;
      pMem->pNext = NULL;
      
      pMem->dSize   = dSize;
      pMem->dListID = (uint32_t)ID;
      pMem->dObject = OBJ_IN_USE;
      
      pMem = (mem_hdr_t*)((uint32_t)pMem + sizeof(mem_hdr_t));

      MEMFree(pMem);
   }      

} /* MEMAdd */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  tal_MEMInit                                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_MEMInit (void)
{
   uint32_t dAddr;
   uint32_t dSize;

   /* 
    * Clear context list first 
    */
   memset(MemList, 0x00, sizeof(MemList));

   /* 
    * Check for correct memory header 
    */   
    
   /*lint -save -e506 -e774 */
   if (sizeof(mem_hdr_t) != 16) return;
   /*lint -restore */ 
   
   /* 
    * Add memory #1 if available
    */
#if defined(TAL_HEAP_MEM1_START)
   /* Get memory pointer */ 
   dAddr = (uint32_t)&TAL_HEAP_MEM1_START;
   dSize = ((uint32_t)(&TAL_HEAP_MEM1_END) - (uint32_t)(&TAL_HEAP_MEM1_START)) - 1;
   if (dSize != (uint32_t)-1)
   {
      MEMAdd(XM_ID_HEAP, (uint8_t*)dAddr, dSize);
   }
#endif

   /* 
    * Add memory #2 if available
    */

#if defined(TAL_HEAP_MEM2_START)
   /* Get memory pointer */ 
   dAddr = (uint32_t)&TAL_HEAP_MEM2_START;
   dSize = ((uint32_t)(&TAL_HEAP_MEM2_END) - (uint32_t)(&TAL_HEAP_MEM2_START)) - 1;
   
   MEMAdd(XM_ID_HEAP, (uint8_t*)dAddr, dSize);
#endif   
   
   MemList[XM_ID_HEAP].pName = "Heap";
   MemList[XM_ID_HEAP].UsedRawMemory = 0;

   OS_SemaCreate(&Sema, 1, 1);
   
   (void)dAddr;
   (void)dSize;

} /* tal_MEMInit */

/*************************************************************************/
/*  tal_MEMAdd                                                           */
/*                                                                       */
/*  Add the memroy to the dynamic memory system.                         */
/*                                                                       */
/*  In    : ID, pName, pBuffer, dSize                                    */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_MEMAdd (tal_mem_id ID, const char *pName, void *pBuffer, uint32_t dSize)
{
   if ((ID < XM_ID_MAX) && (NULL == MemList[ID].pFreelist))
   {
      MEMAdd(ID, pBuffer, dSize);

      MemList[ID].pName = pName;
      MemList[ID].UsedRawMemory = 0;
   }

} /* tal_MEMAdd */

/*************************************************************************/
/*  tal_MEMOutputMemoryInfo                                              */
/*                                                                       */
/*  Output the dynamic memory information from the system.               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_MEMOutputMemoryInfo (void)
{
   uint8_t  wIndex;
   uint32_t dSize;
   uint32_t dUsed;
   uint32_t dFree;
   uint32_t dPeak;
   
   TAL_PRINTF("*** Dynamic Memory Info ***\n");
   TAL_PRINTF("\n");
   TAL_PRINTF("ID                     Size        Used        Free        Peak\n");
   TAL_PRINTF("================================================================\n");

   for (wIndex = 0; wIndex < MEM_LIST_COUNT; wIndex++)
   {
      if (MemList[wIndex].dSize != 0)
      {
         if (MemList[wIndex].pName != NULL)
         {
            TAL_PRINTF("%-16s  ", MemList[wIndex].pName);
         }
         else
         {
            TAL_PRINTF("%-16s  ", "----------------");
         }

         dSize = MemList[wIndex].dSize;
         dUsed = (uint32_t)MemList[wIndex].UsedRawMemory;
         dFree = dSize - dUsed;
         dPeak = (uint32_t)MemList[wIndex].UsedRawMemoryMax;
         tal_MEMClrUsedRawMemoryMax((tal_mem_id)wIndex);         
         
         TAL_PRINTF("%9d   %9d   %9d   %9d\r\n", dSize, dUsed, dFree, dPeak);
      }
   }

   TAL_PRINTF("\r\n");

} /* tal_MEMOutputMemoryInfo */

/*************************************************************************/
/*  tal_MEMInfoGet                                                       */
/*                                                                       */
/*  Output the dynamic memory information from the system for the index. */
/*                                                                       */
/*  In    : wIndex, pSize, pUsed, pFree, pPeak, pNext                    */ 
/*  Out   : pSize, pUsed, pFree, pPeak, pNext                            */ 
/*  Return: none                                                         */
/*************************************************************************/
char *tal_MEMInfoGet (uint16_t wIndex, uint32_t *pSize, uint32_t *pUsed, uint32_t *pFree, uint32_t *pPeak, uint16_t *pNext)
{
   char *pName = NULL;
   
   *pNext = 0;
   
   if (wIndex < MEM_LIST_COUNT)
   {
      *pNext = 1;
      
      if (MemList[wIndex].dSize != 0)
      {
         pName  = (char*)MemList[wIndex].pName;
         *pSize = MemList[wIndex].dSize;
         *pUsed = (uint32_t)MemList[wIndex].UsedRawMemory;
         *pFree = *pSize - *pUsed;
         *pPeak = (uint32_t)MemList[wIndex].UsedRawMemoryMax;
         tal_MEMClrUsedRawMemoryMax((tal_mem_id)wIndex);         
      }
   }
   
   return(pName);
} /* tal_MEMInfoGet */

/*************************************************************************/
/*  tal_MEMGetUsedRawMemory                                              */
/*                                                                       */
/*  Return the raw used memory count.                                    */
/*                                                                       */
/*  In    : ID                                                           */
/*  Out   : none                                                         */
/*  Return: UsedRawMemory                                                */
/*************************************************************************/
int32_t tal_MEMGetUsedRawMemory (tal_mem_id ID)
{
   int32_t UsedRawMemory = -1;

   if (ID < XM_ID_MAX)
   {
      UsedRawMemory = MemList[ID].UsedRawMemory;
   }

   return(UsedRawMemory);
} /* tal_MEMGetUsedRawMemory */

/*************************************************************************/
/*  tal_MEMGetUsedRawMemoryMax                                           */
/*                                                                       */
/*  Return the maximum raw used memory count.                            */
/*                                                                       */
/*  In    : ID                                                           */
/*  Out   : none                                                         */
/*  Return: UsedRawMemoryMax                                             */
/*************************************************************************/
int32_t tal_MEMGetUsedRawMemoryMax (tal_mem_id ID)
{
   int32_t UsedRawMemoryMax = -1;

   if (ID < XM_ID_MAX)
   {
      UsedRawMemoryMax = MemList[ID].UsedRawMemoryMax;
   }

   return(UsedRawMemoryMax);
} /* tal_MEMGetUsedRawMemoryMax */

/*************************************************************************/
/*  tal_MEMClrUsedRawMemoryMax                                           */
/*                                                                       */
/*  Clear the maximum raw used memory count.                             */
/*                                                                       */
/*  In    : ID                                                           */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_MEMClrUsedRawMemoryMax (tal_mem_id ID)
{
   if (ID < XM_ID_MAX)
   {
      MemList[ID].UsedRawMemoryMax = 0;
   }

} /* tal_MEMClrUsedRawMemoryMax */

/*************************************************************************/
/*  calloc                                                               */
/*                                                                       */
/*  Allocate memory of the given size, and initialize them to zero.      */
/*  Returns NULL if memory cannot be allocated.                          */
/*                                                                       */
/*  In    : nobj, size                                                   */
/*  Out   : none                                                         */
/*  Return: p / NULL                                                     */
/*************************************************************************/
void *calloc (size_t nobj, size_t size)
{
   void     *p = NULL;
   uint32_t dSize = nobj * size;

   if (dSize != 0)
   {
      OS_SemaWait(&Sema, OS_WAIT_INFINITE);
   
      p = MEMCalloc(XM_ID_HEAP, dSize);
   
      OS_SemaSignal(&Sema);
   }      
   
   return(p);
} /* calloc */

/*************************************************************************/
/*  malloc                                                               */
/*                                                                       */
/*  Allocate memory of the given size.                                   */
/*  Returns NULL if memory cannot be allocated.                          */
/*                                                                       */
/*  In    : dSize                                                        */
/*  Out   : none                                                         */
/*  Return: p / NULL                                                     */
/*************************************************************************/
void *malloc (size_t size)
{
   void *p = NULL;
   
   if (size != 0)
   {
      OS_SemaWait(&Sema, OS_WAIT_INFINITE);
   
      p = MEMMalloc(XM_ID_HEAP, size);

      OS_SemaSignal(&Sema);
   }      
   
   return(p);
} /* malloc */

/*************************************************************************/
/*  realloc                                                              */
/*                                                                       */
/*  Resize allocated memory.                                             */
/*                                                                       */
/*  In    : p, size                                                      */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void *realloc (void *p, size_t size)
{
   void *new = NULL;
   
   if (size != 0)
   {
      OS_SemaWait(&Sema, OS_WAIT_INFINITE);

      new = MEMMalloc(XM_ID_HEAP, size);
      if (new != NULL)
      {
         if (p != NULL)
         {
            memcpy(new, p, size);
            MEMFree(p);
         }   
      }

      OS_SemaSignal(&Sema);
   }

   return(new);
} /* realloc */

/*************************************************************************/
/*  free                                                                 */
/*                                                                       */
/*  Frees the allocated memory.                                          */
/*                                                                       */
/*  In    : p                                                            */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void free (void *p)
{
   if (p != NULL)
   {
      OS_SemaWait(&Sema, OS_WAIT_INFINITE);

      MEMFree(p);
   
      OS_SemaSignal(&Sema);
   }      
   
} /* free */

/*************************************************************************/
/*  xcalloc                                                              */
/*                                                                       */
/*  Allocate memory of the given size, and initialize them to zero.      */
/*  Returns NULL if memory cannot be allocated.                          */
/*                                                                       */
/*  In    : ID, nobj, size                                               */
/*  Out   : none                                                         */
/*  Return: p / NULL                                                     */
/*************************************************************************/
void *xcalloc (tal_mem_id ID, size_t nobj, size_t size)
{
   void     *p = NULL;
   uint32_t dSize = nobj * size;

   if ((ID < XM_ID_MAX) && (MemList[ID].dSize != 0) && (dSize != 0))
   {
      OS_SemaWait(&Sema, OS_WAIT_INFINITE);
   
      p = MEMCalloc(ID, dSize);
   
      OS_SemaSignal(&Sema);
   }
   
   return(p);
} /* xcalloc */

/*************************************************************************/
/*  xmalloc                                                              */
/*                                                                       */
/*  Allocate memory of the given size.                                   */
/*  Returns NULL if memory cannot be allocated.                          */
/*                                                                       */
/*  In    : ID, size                                                     */
/*  Out   : none                                                         */
/*  Return: p / NULL                                                     */
/*************************************************************************/
void *xmalloc (tal_mem_id ID, size_t size)
{
   void *p = NULL;

   if ((ID < XM_ID_MAX) && (MemList[ID].dSize != 0) && (size != 0))
   {
      OS_SemaWait(&Sema, OS_WAIT_INFINITE);
   
      p = MEMMalloc(ID, size);
   
      OS_SemaSignal(&Sema);
   }
   
   return(p);
} /* xmalloc */

/*************************************************************************/
/*  xrealloc                                                             */
/*                                                                       */
/*  Resize allocated memory.                                             */
/*                                                                       */
/*  In    : ID, p, size                                                  */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void *xrealloc (tal_mem_id ID, void *p, size_t size)
{
   void *new = NULL;

   if ((ID < XM_ID_MAX) && (MemList[ID].dSize != 0) && (size != 0))
   {
      OS_SemaWait(&Sema, OS_WAIT_INFINITE);

      new = MEMMalloc(ID, size);
      if (new != NULL)
      {
         if (p != NULL)
         {
            memcpy(new, p, size);
            MEMFree(p);
         }   
      }

      OS_SemaSignal(&Sema);
   }

   return(new);
} /* xrealloc */

/*************************************************************************/
/*  xfree                                                                */
/*                                                                       */
/*  Frees the allocated memory.                                          */
/*                                                                       */
/*  In    : p                                                            */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void xfree (void *p)
{
   if (p != NULL)
   {
      OS_SemaWait(&Sema, OS_WAIT_INFINITE);

      MEMFree(p);
   
      OS_SemaSignal(&Sema);
   }      
   
} /* xfree */

/*************************************************************************/
/*  xstrdup                                                              */
/*                                                                       */
/*  xstrdup duplicates the string pointed to by s1 by using malloc to    */
/*  allocate memory for a copy of s and then copying s.                  */
/*                                                                       */
/*  In    : ID, s1                                                       */
/*  Out   : none                                                         */
/*  Return: p = OK / NULL = Error                                        */
/*************************************************************************/
char *xstrdup (tal_mem_id ID, const char *s1)
{
   char  *p = NULL;
   size_t len;

   if ((ID < XM_ID_MAX) && (MemList[ID].dSize != 0) && (s1 != NULL))
   {
      len = strlen(s1);
      p = xmalloc(ID, len + 1);
      if (p != NULL)
      {
         memcpy(p, s1, len + 1);
      }         
   }
   
   return(p);
} /* xstrdup */

/*** EOF ***/
