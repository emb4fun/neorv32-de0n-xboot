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
*  15.02.2015  mifi  First Version, extract from cts.c for Cortex-M.
*  10.01.2016  mifi  Rework OSTaskCreate, schedule the new task if
*                    the scheduler is not locked.
*  12.05.2018  mifi  Added ContextSwitchExit support.
*  16.05.2018  mifi  Check 8 byte stack alignment.
**************************************************************************/
#define __TCTS_CM_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include <stdint.h>

#include "tcts.h"

#if !defined(__SOFTFP__) && defined(__FPU_PRESENT)
#define MCU_USE_CORTEX_FPU
#endif

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


/*
 * Cortex-M GCC context switch frame layout.
 *
 * This is the layout of the stack after a task's context has been
 * switched-out. The stack pointer is stored in the task control block
 * and points to this structure.
 */
typedef struct _switch_frame_
{
#if defined(MCU_USE_CORTEX_FPU)
   uint32_t csf_fpscr;
   uint32_t csf_s16;
   uint32_t csf_s17;
   uint32_t csf_s18;
   uint32_t csf_s19;
   uint32_t csf_s20;
   uint32_t csf_s21;
   uint32_t csf_s22;
   uint32_t csf_s23;
   uint32_t csf_s24;
   uint32_t csf_s25;
   uint32_t csf_s26;
   uint32_t csf_s27;
   uint32_t csf_s28;
   uint32_t csf_s29;
   uint32_t csf_s30;
   uint32_t csf_s31;
#endif

   uint32_t csf_cpsr;
   uint32_t csf_r4;
   uint32_t csf_r5;
   uint32_t csf_r6;
   uint32_t csf_r7;
   uint32_t csf_r8;
   uint32_t csf_r9;
   uint32_t csf_r10;
   uint32_t csf_r11;
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
/*  Return the IRQ stack start address, here process stack.              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: stack start address                                          */
/*************************************************************************/
static uint8_t *GetIRQStackStart (void)
{
   return(__stack_process_start__);
} /* GetIRQStackStart */

/*************************************************************************/
/*  GetIRQStackEnd                                                       */
/*                                                                       */
/*  Return the IRQ stack end address, here process stack.                */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: stack start address                                          */
/*************************************************************************/
static uint8_t *GetIRQStackEnd (void)
{
   return(__stack_process_end__);
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
                      "mov r11, 0x11111111\n"
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
   __asm__ volatile ( "stmfd  sp!, {r4-r11, lr}\n"    /* Save registers. */
                      "mrs    r4,  psr\n"             /* Move status. */
                      "stmfd  sp!, {r4}\n"            /* Save status.*/

#if defined(MCU_USE_CORTEX_FPU)        
                      "vpush  {s16-s31}\n"            /* Save registers. */
                      "vmrs   r4,  fpscr\n"           /* Move FPU status. */
                      "stmfd  sp!, {r4}\n"            /* Save FPU status.*/
#endif
        
                      "str    sp, %0"                 /* Save stack pointer. */
                      :: "m" (RunningTask->StackPtr) );


   /* Set the new task */
   RunningTask = NewTask;
   RunningTask->State = OS_TASK_STATE_RUNNING;


   /*
    * Restore CPU context.
    */
   __asm__ volatile ( "ldr    sp,  %0\n"              /* Restore stack pointer. */

#if defined(MCU_USE_CORTEX_FPU)        
                      "ldmfd  sp!, {r4}\n"            /* Get saved FPU status... */            
                      "vmsr   fpscr, r4\n"            /* ...and save back. */
                      "vpop   {s16-s31}\n"            /* Restore FPU registers. */
#endif
        
                      "ldmfd   sp!, {r4}\n"           /* Get saved status... */
                      "msr     xpsr_nzcvq, r4\n"      /* ...and save execution and application status in psr. */
                      "ldmfd   sp!, {r4-r11, pc}\n"   /* Restore registers. */
                      :: "m" (RunningTask->StackPtr) );
    
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
   __asm__ volatile ( "ldr    sp,  %0\n"              /* Restore stack pointer. */

#if defined(MCU_USE_CORTEX_FPU)        
                      "ldmfd  sp!, {r4}\n"            /* Get saved FPU status... */            
                      "vmsr   fpscr, r4\n"            /* ...and save back. */
                      "vpop   {s16-s31}\n"            /* Restore FPU registers. */
#endif
        
                      "ldmfd   sp!, {r4}\n"           /* Get saved status... */
                      "msr     xpsr_nzcvq, r4\n"      /* ...and save execution and application status in psr. */
                      "ldmfd   sp!, {r4-r11, pc}\n"   /* Restore registers. */
                      :: "m" (RunningTask->StackPtr) );
    
} /* ContextSwitchExit */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  OS_Start                                                             */
/*                                                                       */
/*  Start the "Cooperative Task Scheduler".                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
void OS_Start (void)
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

   __asm__ volatile ( "ldr    sp,  %0\n"              /* Restore stack pointer. */

#if defined(MCU_USE_CORTEX_FPU)        
                      "ldmfd  sp!, {r4}\n"            /* Get saved FPU status... */            
                      "vmsr   fpscr, r4\n"            /* ...and save back. */
                      "vpop   {s16-s31}\n"            /* Restore FPU registers. */
#endif
        
                      "ldmfd   sp!, {r4}\n"           /* Get saved status... */
                      "msr     xpsr_nzcvq, r4\n"      /* ...and save execution and application status in psr. */
                      "cpsie   i\n"                   /* ...enable interrupts */
                      "ldmfd   sp!, {r4-r11, pc}\n"   /* Restore registers. */
                      :: "m" (RunningTask->StackPtr) );

} /* OS_Start */

/*************************************************************************/
/*  OS_TaskCreate                                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TaskCreate (OS_TCB *pTCB, OS_TASK Task, void *pParam, int nPrio,
                    uint8_t *pStack, uint16_t wStackSize, char *pName)
{
   switch_frame_t *sf;
   call_frame_t   *cf;
   uint32_t        addr;
   size_t          len;
   
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
   memset(pTCB, 0x00, sizeof(OS_TCB));
   
   /* Fill the stack memory with a check pattern */
   memset(pStack, 0xCC, wStackSize);
   
   /* Copy name */
   pTCB->Name[0] = 0;
   if (pName != NULL)
   {
      len = strlen(pName) + 1;
      if (len < sizeof(pTCB->Name))
      {
         memcpy(pTCB->Name, pName, len);
      }
   }
   
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
   sf->csf_r11  = (uintptr_t)OS_TaskExit;
   sf->csf_lr   = (uintptr_t)TaskCall;
   
#if defined(MCU_USE_CORTEX_FPU)
   sf->csf_fpscr = 0x03000000;
   sf->csf_s16   = 0x16161616;
   sf->csf_s17   = 0x17171717;
   sf->csf_s18   = 0x18181818;
   sf->csf_s19   = 0x19191919;
   sf->csf_s20   = 0x20202020;
   sf->csf_s21   = 0x21212121;
   sf->csf_s22   = 0x22222222;
   sf->csf_s23   = 0x23232323;
   sf->csf_s24   = 0x24242424;
   sf->csf_s25   = 0x25252525;
   sf->csf_s26   = 0x26262626;
   sf->csf_s27   = 0x27272727;
   sf->csf_s28   = 0x28282828;
   sf->csf_s29   = 0x29292929;
   sf->csf_s30   = 0x30303030;
   sf->csf_s31   = 0x31313131;
#endif    
   
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
   
} /* OS_TaskCreate */

/*** EOF ***/
