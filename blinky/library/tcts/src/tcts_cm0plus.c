/**************************************************************************
*  This file is part of the TCTS project (Tiny Cooperative Task Scheduler)
*
*  Copyright (c) 2016 by Michael Fischer (www.emb4fun.de).
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
*  10.06.2016  mifi  First Version, tested with a SAM D20 Xplained Pro board.
*  12.05.2018  mifi  Added ContextSwitchExit support.
*  16.05.2018  mifi  Check 8 byte stack alignment.
**************************************************************************/
#define __TCTS_CM0PLUS_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include <stdint.h>

#include "tcts.h"

/*=======================================================================*/
/*  All extern data                                                      */
/*=======================================================================*/

extern uint8_t __stack_start__[];
extern uint8_t __stack_end__[];

extern uint8_t __stack_process_start__[];
extern uint8_t __stack_process_end__[];

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define EnterCritical()    __disable_irq()
#define ExitCritical()     __enable_irq()


/* For M0, push, pop, ldr etc only work on r0-r7
 * push{<list>} stores highest register at highest stack address
 * pop{<list>}  pops highest register from highest stack address
 */
#define M0_PushContext()                                                        \
   __asm__ volatile ( "@ Save context\n"                                        \
                      "push    {r4-r7,lr}\n"    /* Save lr, r7, r6, r5, r4. */  \
                      "mov     r4, r8 \n"       /* Mov high registers */        \
                      "mov     r5, r9 \n"                                       \
                      "mov     r6, r10\n"                                       \
                      "mov     r7, r11\n"                                       \
                      "push    {r4-r7}\n"       /* Save regs r11, r10, r9, r8*/ \
                      "mrs     r4,  psr\n"      /* Save status. */              \
                      "push    {r4}\n"                                          \
                      "mov     r4, sp\n"                                        \
                      "str     r4, %0\n"                                        \
                      ::"m" (RunningTask->StackPtr))

#define M0_PopContext()                                                           \
   __asm__ volatile ( "@ Load context\n"                                          \
                      "ldr     r4, %0\n"           /* Get new stack address    */ \
                      "mov     sp, r4\n"           /* ... and store as sp      */ \
                      "pop     {r4}\n"             /* Get saved status...      */ \
                      "msr     xpsr_nzcvq, r4\n"   /* ... and restore psr.     */ \
                      "pop     {r4-r7}\n"          /* Pop r11, r10, r9, r8     */ \
                      "mov     r8 , r4\n"          /* and move tohigh registers*/ \
                      "mov     r9 , r5\n"                                         \
                      "mov     r10, r6\n"                                         \
                      "mov     r11, r7\n"                                         \
                      "cpsie   i\n"                /* Enable interrupts        */ \
                      "pop     {r4-r7, pc}\n"      /* Pop PC, r7, r6, r5, r4   */ \
                      ::"m"(RunningTask->StackPtr))


/*
 * Cortex-M GCC context switch frame layout.
 *
 * This is the layout of the stack after a task's context has been
 * switched-out. The stack pointer is stored in the task control block
 * and points to this structure.
 */
typedef struct _switch_frame_
{
   uint32_t csf_cpsr;
   uint32_t csf_r8;
   uint32_t csf_r9;
   uint32_t csf_r10;
   uint32_t csf_r11;
   uint32_t csf_r4;
   uint32_t csf_r5;
   uint32_t csf_r6;
   uint32_t csf_r7;
   uint32_t csf_lr;
} switch_frame_t;


/*
 * Task call frame layout.
 *
 * This is the stack layout being build to call a new task.
 */
typedef struct _call_frame_ 
{
   uint32_t r0;
   uint32_t pc;
} call_frame_t;


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
   uint32_t dSize;
   uint32_t dFree;
   
   TAL_PRINTF("*** Runtime Stack Info ***\n");
   TAL_PRINTF("\n");
   TAL_PRINTF("Stack          Size   Used   Free\n");
   TAL_PRINTF("===================================\n");
   
   dSize = __stack_end__ - __stack_start__;
   dFree = GetStackFreeCount(__stack_start__, __stack_end__);
   TAL_PRINTF("Main Stack     %4d   %4d   %4d\n", dSize, (dSize - dFree), dFree);
   
   dSize = __stack_process_end__ - __stack_process_start__;
   dFree = GetStackFreeCount(__stack_process_start__, __stack_process_end__);
   TAL_PRINTF("Process Stack  %4d   %4d   %4d\n", dSize, (dSize - dFree), dFree);
   
   (void)dSize;
   (void)dFree;
   
   TAL_PRINTF("\n");
} /* OutputRuntimeStackInfo */

/*************************************************************************/
/*  GetIRQStackStart                                                     */
/*                                                                       */
/*  Return the IRQ stack start address.                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: stack start address                                          */
/*************************************************************************/
static uint8_t *GetIRQStackStart (void)
{
   return(__stack_start__);
} /* GetIRQStackStart */

/*************************************************************************/
/*  GetIRQStackEnd                                                       */
/*                                                                       */
/*  Return the IRQ stack end address.                                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: stack start address                                          */
/*************************************************************************/
static uint8_t *GetIRQStackEnd (void)
{
   return(__stack_end__);
} /* GetIRQStackEnd */

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
   /*
    * Load r11 (OSTaskExit) in lr, and set r11.
    * The content of r11 (OSTaskExit) was set in OSTaskCreate.
    * Load argument in r0 and jump to thread entry.
    */   
   __asm__ volatile ( "mov lr, r11\n"
                      "cpsie   i\n"
                      "pop {r0, pc}\n"
                      ::: "r0", "pc" );
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
   /* 
    * Save CPU context. 
    */
   M0_PushContext();


   /* Set the new task */
   RunningTask = NewTask;
   RunningTask->State = OS_TASK_STATE_RUNNING;


   /*
    * Restore CPU context.
    */
   M0_PopContext();
    
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
   /* Set the new task */
   RunningTask = NewTask;
   RunningTask->State = OS_TASK_STATE_RUNNING;


   /*
    * Restore CPU context.
    */
   M0_PopContext();
    
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
    * Check if the CPU is running on the Process stack.
    * If not, this is an error.
    *
    * Process and Main Stack Size must be defined in CrossWorks
    * or Embedded Studio Linker properties. The Main Stack is used
    * for the IRQ and the Process Stack to start the system.
    */
   if (__stack_process_start__ == __stack_process_end__)
   {
      while(1)
      {
         /* Process Stack Size not defined */
         __asm__ volatile ("nop");
      }
   }

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

   M0_PopContext();

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
   call_frame_t   *cf;
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
    *              I     I              I
    *              I  T  I   Callframe  I
    *              I  a  I              I
    * cf ->        I  s  +--------------+
    *              I  k  I              I    ^
    *              I     I  Switchframe I    I
    *              I     I              I    I  pop moves up
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
   
   cf = (call_frame_t*)((uintptr_t) &pStack[wStackSize] - sizeof(call_frame_t));
   sf = (switch_frame_t*)((uintptr_t) cf - sizeof(switch_frame_t));

   /*
    * Setup the call frame to simulate a C function call.
    */
   cf->pc = (uintptr_t)Task;
   cf->r0 = (uintptr_t)pParam;

   /*
    * Setup the switch frame.
    */
   sf->csf_cpsr = 0;
   sf->csf_r4   = 0x04040404;
   sf->csf_r5   = 0x05050505;
   sf->csf_r6   = 0x06060606;
   sf->csf_r7   = 0x07070707;
   sf->csf_r8   = 0x08080808;
   sf->csf_r9   = 0x09090909;
   sf->csf_r10  = 0x10101010;
   sf->csf_r11  = (uintptr_t)OSTaskExit;
   sf->csf_lr   = (uintptr_t)TaskCall;
   
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
