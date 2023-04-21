/**************************************************************************
*  This file is part of the TCTS project (Tiny Cooperative Task Scheduler)
*
*  Copyright (c) 2015 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Some functionality comes from the Ethernut (www.ethernut.de) project.
*  Therefore:
*
*  Partial Copyright (c) 2001-2005 by egnite Software GmbH.
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
*  15.11.2015  mifi  First Version for the Nios II.
*  10.01.2016  mifi  Rework OSTaskCreate, schedule the new task if
*                    the scheduler is not locked.
*  12.05.2018  mifi  Added ContextSwitchExit support.
*  16.05.2018  mifi  Check 8 byte stack alignment.
**************************************************************************/
#define __TCTS_NIOS_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include <stdint.h>

#include "tcts.h"

/*=======================================================================*/
/*  All extern data                                                      */
/*=======================================================================*/

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/


#define EnterCritical()    \
{                          \
   alt_irq_disable_all();  \
}

#define ExitCritical()     \
{                          \
   alt_irq_enable_all(1);  \
}


/*
 * NiosII GCC context switch frame layout.
 *
 * This is the layout of the stack after a task's context has been
 * switched-out. The stack pointer is stored in the task control block
 * and points to this structure.
 */
typedef struct _switch_frame_
{
   uint32_t  ra;
   uint32_t  fp;
   uint32_t  r16;
   uint32_t  r17;
   uint32_t  r18;
   uint32_t  r19;
   uint32_t  r20;
   uint32_t  r21;
   uint32_t  r22;
   uint32_t  r23;  
   uint32_t  status;
} switch_frame_t;


/*
 * Forward declaration
 */
static uint32_t GetStackFreeCount (uint8_t *pStart, uint8_t *pEnd);
static OS_TCB  *GetReadyTask (void);
static void TaskListAdd (OS_TCB *pTask);
static __inline__ void AddTaskToReadyList (OS_TCB *pTask);
static void TaskSchedule (void);

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/   

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  OutputRuntimeStackInfo                                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputRuntimeStackInfo (void)
{
} /* OutputRuntimeStackInfo */

/*************************************************************************/
/*  TaskCall                                                             */
/*                                                                       */
/*  Call a new task, simulate a C call.                                  */ 
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void TaskCall (void) __attribute__((__naked__));
static void TaskCall (void)
{
   __asm__ volatile ( "mov   r4, r17\n"   /* r4 = Register Arguments (First 32 bits) */
                      "mov   r5, r16\n"   /* Store thread pointer for the callr,     */
                      "callr r5\n" );     /* and call the thread */
      
   /* Jump to OSTaskExit, in case the task returns. */
   __asm__ volatile ( "br    OSTaskExit" );
} /* TaskCall */

/*************************************************************************/
/*  ContextSwitch                                                        */
/*                                                                       */
/*  Switch to the new task.                                              */ 
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void ContextSwitch (void) __attribute__((__naked__));
static void ContextSwitch (void)
{
   /* r4 = new stack pointer, r5 = old stack pointer */

   /* 
    * Save CPU context. 
    */
   __asm__ volatile ( "addi  sp,  sp, -44\n"       /* Size of 11 x push = 11 * uint32_t = 44 */
                      "stw   ra,   0(sp)\n"        /* push */
                      "stw   fp,   4(sp)\n"        /* push */
                      "stw   r16,  8(sp)\n"        /* push */
                      "stw   r17, 12(sp)\n"        /* push */
                      "stw   r18, 16(sp)\n"        /* push */
                      "stw   r19, 20(sp)\n"        /* push */
                      "stw   r20, 24(sp)\n"        /* push */
                      "stw   r21, 28(sp)\n"        /* push */
                      "stw   r22, 32(sp)\n"        /* push */
                      "stw   r23, 36(sp)\n"        /* push */
                      "rdctl r23, status\n"        /* r23 is not more needed and can */
                      "stw   r23, 40(sp)\n"        /* be used here to store the status, push */
                      "stw   sp,  %[StackPtr]\n"   /* Save old stack pointer */   
                      :: [StackPtr] "o"(RunningTask->StackPtr) : "r5", "memory" );

   /* Set the new task */
   RunningTask = NewTask;
   RunningTask->State = OS_TASK_STATE_RUNNING;

   /*
    * Restore CPU context.
    */
   __asm__ volatile ( "ldw   sp,  %[StackPtr]\n"   /* Restore stack pointer. */
                      "ldw   ra,   0(sp)\n"        /* pop */
                      "ldw   fp,   4(sp)\n"        /* pop */
                      "ldw   r16,  8(sp)\n"        /* pop */
                      "ldw   r17, 12(sp)\n"        /* pop */
                      "ldw   r18, 16(sp)\n"        /* pop */
                      "ldw   r19, 20(sp)\n"        /* pop */
                      "ldw   r20, 24(sp)\n"        /* pop */
                      "ldw   r21, 28(sp)\n"        /* pop */
                      "ldw   r22, 32(sp)\n"        /* pop */
                      "ldw   r23, 36(sp)\n"        /* pop */
                      "ldw   r4,  40(sp)\n"        /* pop, r4 is not more needed and can */
                      "wrctl status, r4\n"         /* be used here to store the status */
                      "addi  sp, sp, 44\n"         /* Size of 11 x pop = 11 * uint32_t = 44 */
                      "ret\n"
                      :: [StackPtr] "m"(RunningTask->StackPtr) : "r4", "memory" ); 

} /* ContextSwitch */

/*************************************************************************/
/*  ContextSwitchExit                                                    */
/*                                                                       */
/*  Switch to the new task, like ContextSwitch.                          */
/*  But without to save the CPU context. Used by OSTaskExit.             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void ContextSwitchExit (void) __attribute__((__naked__));
static void ContextSwitchExit (void)
{
   /* r4 = new stack pointer, r5 = old stack pointer */

   /* Set the new task */
   RunningTask = NewTask;
   RunningTask->State = OS_TASK_STATE_RUNNING;

   /*
    * Restore CPU context.
    */
   __asm__ volatile ( "ldw   sp,  %[StackPtr]\n"   /* Restore stack pointer. */
                      "ldw   ra,   0(sp)\n"        /* pop */
                      "ldw   fp,   4(sp)\n"        /* pop */
                      "ldw   r16,  8(sp)\n"        /* pop */
                      "ldw   r17, 12(sp)\n"        /* pop */
                      "ldw   r18, 16(sp)\n"        /* pop */
                      "ldw   r19, 20(sp)\n"        /* pop */
                      "ldw   r20, 24(sp)\n"        /* pop */
                      "ldw   r21, 28(sp)\n"        /* pop */
                      "ldw   r22, 32(sp)\n"        /* pop */
                      "ldw   r23, 36(sp)\n"        /* pop */
                      "ldw   r4,  40(sp)\n"        /* pop, r4 is not more needed and can */
                      "wrctl status, r4\n"         /* be used here to store the status */
                      "addi  sp, sp, 44\n"         /* Size of 11 x pop = 11 * uint32_t = 44 */
                      "ret\n"
                      :: [StackPtr] "m"(RunningTask->StackPtr) : "r4", "memory" ); 

} /* ContextSwitchExit */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  OSStart                                                              */
/*                                                                       */
/*  Start the "Cooperative Task Scheduler".                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
void OSStart (void)
{
   OS_TCB *pTask;  

   /*
    * Use the StartCallback to start the one and only task
    * which will start all other tasks of the system.
    */
   OSStartCallback();   
   
   /* Disable all interrupts */   
   EnterCritical();
   
   /* Start the SysTick */
   tal_CPUSysTickStart();

   /* Unlock the scheduler */
   bIsSchedLocked = 0;  

   /*  Get a task from the ReadyList */
   pTask = GetReadyTask();   
   /* ExitCritical is not needed here, OSStart will not come back */
   
   
   /*
    * Start the task
    */
   RunningTask = pTask;
   RunningTask->State = OS_TASK_STATE_RUNNING;
   RunningTask->dStatStartTime = tal_CPUStatGetHiResCnt();

   __asm__ volatile ( "ldw   sp,  %[StackPtr]\n"   /* Restore stack pointer. */
                      "ldw   ra,   0(sp)\n"        /* pop */
                      "ldw   fp,   4(sp)\n"        /* pop */
                      "ldw   r16,  8(sp)\n"        /* pop */
                      "ldw   r17, 12(sp)\n"        /* pop */
                      "ldw   r18, 16(sp)\n"        /* pop */
                      "ldw   r19, 20(sp)\n"        /* pop */
                      "ldw   r20, 24(sp)\n"        /* pop */
                      "ldw   r21, 28(sp)\n"        /* pop */
                      "ldw   r22, 32(sp)\n"        /* pop */
                      "ldw   r23, 36(sp)\n"        /* pop */
                      "ldw   r4,  40(sp)\n"        /* pop, r4 is not more needed and can */
                      "wrctl status, r4\n"         /* be used here to store the status */
                      "addi  sp, sp, 44\n"         /* Size of 11 x pop = 11 * uint32_t = 44 */
                      "ret\n"
                      :: [StackPtr] "m"(RunningTask->StackPtr) : "r4", "memory" ); 

} /* OSStart */

/*************************************************************************/
/*  OSTaskCreate                                                         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
void OSTaskCreate (OS_TCB *pTCB, OS_TASK Task, void *pParam, int nPrio,
                   uint8_t *pStack, uint16_t wStackSize, char *pName)
{
   switch_frame_t *sf;
   uint32_t        addr;
   
   /* Check 8 byte alignment of the stack */
   addr = (uint32_t)pStack;
   if ((addr & 0x7) != 0)
   {
      /* Corrected start and size */
      addr += 7;
      addr &= ~7;
      
      wStackSize -= 8;
      
      pStack = (uint8_t*)addr;
   }
                                            
   /* Clear the TCB memory */
   memset(pTCB, 0x00, sizeof(os_tcb_t));
   
   /* Fill the stack memory with a check pattern */
   memset(pStack, 0xCC, wStackSize);
   
   /* Copy name */
   memcpy(pTCB->Name, pName, sizeof(pTCB->Name) - 1); /*lint !e420*/
   
   /* 
    * -- Do not use the macro SET_TASK_STATE here --
    *
    * The macro can be "empty" in case of performance
    * optimisation and will not work.
    */   
   pTCB->State       = OS_TASK_STATE_CREATED;
   
   pTCB->nPrio       = nPrio;
   pTCB->pStackStart = pStack;
   pTCB->wStackSize  = wStackSize;
   
   /*
    * Setup the stack frame, the following layout will be used:
    *
    * Upper memory addresses.
    *
    *              +-----+--------------+ <- Stack top (StackStart + StackSize)
    *              I  T  I              I 
    *              I  a  I              I    ^
    *              I  s  I  Switchframe I    I
    *              I  k  I              I    I  pop moves up
    * sf ->        I     +--------------+ <- Initial stack pointer
    *              I  S  I              I    I  push moves down
    *              I  t  I Application  I    I
    *              I  a  I Stack        I    V
    *              I  c  I              I
    *              I  k  I              I
    * StackStart-> +-----+--------------+ <- Stack bottom
    *
    * Lower memory addresses.
    */

   sf = (switch_frame_t*)((uintptr_t) &pStack[wStackSize] - sizeof(switch_frame_t));

   /*
    * Setup the switch frame.
    */
   sf->r16    = (uintptr_t)Task;               
   sf->r17    = (uintptr_t)pParam;               
   sf->r18    = 0x18181818;                     
   sf->r19    = 0x19191919;                     
   sf->r20    = 0x20202020;                     
   sf->r21    = 0x21212121;                     
   sf->r22    = 0x22222222;                     
   sf->r23    = 0x23232323;                     
   sf->status = NIOS2_STATUS_PIE_MSK;           
   sf->fp     = 0x28282828;                     
   sf->ra     = (uint32_t)TaskCall;
   
   pTCB->StackPtr = (uintptr_t)sf;

   EnterCritical();
   TaskListAdd(pTCB);         /* Add the task to the TaskList */
   AddTaskToReadyList(pTCB);  /* Add the task to the ReadyList */
   
   /* Schedule the new task if the scheduler is not locked */
   if (0 == bIsSchedLocked)
   {
      AddTaskToReadyList(RunningTask);    
      TaskSchedule();   
   }
   ExitCritical();
   
} /* OSTaskCreate */

/*** EOF ***/
