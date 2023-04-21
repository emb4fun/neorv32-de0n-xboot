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
*  15.02.2015  mifi  First Version, extract from cts.c for ARM.
*  14.11.2015  mifi  Added support for Cortex-A8 and the Cortex-A8 FPU.
*  10.01.2016  mifi  Rework OSTaskCreate, schedule the new task if
*                    the scheduler is not locked.
*  12.05.2018  mifi  Added ContextSwitchExit support.
*  16.05.2018  mifi  Check 8 byte stack alignment.
**************************************************************************/
#define __TCTS_ARM_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include <stdint.h>

#include "tcts.h"

#if defined(SUPERVISOR_START)
   Error: SUPERVISOR_START must NOT be defined;
#endif

#if !defined(__NO_FPU) && !defined(__SOFTFP__)
#define MCU_USE_CORTEX_FPU
#endif

/*=======================================================================*/
/*  All extern data                                                      */
/*=======================================================================*/

extern uint8_t __stack_start__[];
extern uint8_t __stack_end__[];

/* IRQ */
extern uint8_t __stack_irq_start__[];
extern uint8_t __stack_irq_end__[];

/* FIQ */
extern uint8_t __stack_fiq_start__[];
extern uint8_t __stack_fiq_end__[];

/* Supervisor */
extern uint8_t __stack_svc_start__[];
extern uint8_t __stack_svc_end__[];

/* Abort */
extern uint8_t __stack_abt_start__[];
extern uint8_t __stack_abt_end__[];

/* Undefined */
extern uint8_t __stack_und_start__[];
extern uint8_t __stack_und_end__[];

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*
 * Imprecise abort disable bit
 */
#define CPSR_A_BIT      0x100
#define ARM_MODE_SYS    0x1F


#if defined(__ARM_ARCH_7A__)
#define CPSR_VALUE      (CPSR_A_BIT | ARM_MODE_SYS)
#else
#define CPSR_VALUE      ARM_MODE_SYS
#endif


#define EnterCritical()                            \
{                                                  \
   int temp_;                                      \
   __asm__ volatile ( "mrs  %0, cpsr\n"            \
                      "orr  %0, %0, #0xC0\n"       \
                      "msr  cpsr, %0\n"            \
                      : "=r" (temp_) : : "cc" );   \
}

#define ExitCritical()                             \
{                                                  \
   int temp_;                                      \
   __asm__ volatile ( "mrs  %0, cpsr\n"            \
                      "bic  %0, %0, #0xC0\n"       \
                      "msr  cpsr, %0\n"            \
                      : "=r" (temp_) : : "cc");    \
}


/*
 * ARM7TDMI GCC context switch frame layout.
 *
 * This is the layout of the stack after a task's context has been
 * switched-out. The stack pointer is stored in the task control block
 * and points to this structure.
 */
typedef struct _switch_frame_
{
#if defined(MCU_USE_CORTEX_FPU)
   uint32_t fpscr;
   uint32_t s0;
   uint32_t s1;
   uint32_t s2;
   uint32_t s3;
   uint32_t s4;
   uint32_t s5;
   uint32_t s6;
   uint32_t s7;
   uint32_t s8;
   uint32_t s9;
   uint32_t s10;
   uint32_t s11;
   uint32_t s12;
   uint32_t s13;
   uint32_t s14;
   uint32_t s15;
   
#if defined(__ARM_ARCH_VFP3_D32__) || defined(__ARM_ARCH_VFP4_D32__)
   uint32_t s16;   
   uint32_t s17;
   uint32_t s18;
   uint32_t s19;
   uint32_t s20;
   uint32_t s21;
   uint32_t s22;
   uint32_t s23;
   uint32_t s24;
   uint32_t s25;
   uint32_t s26;
   uint32_t s27;
   uint32_t s28;
   uint32_t s29;
   uint32_t s30;
   uint32_t s31;
#endif   

#endif

   uint32_t cpsr;
   uint32_t r4;
   uint32_t r5;
   uint32_t r6;
   uint32_t r7;
   uint32_t r8;
   uint32_t r9;
   uint32_t r10;
   uint32_t r11;
   uint32_t lr;
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
   TAL_PRINTF("Stack              Size   Used   Free\n");
   TAL_PRINTF("=======================================\n");
   
   dSize = __stack_end__ - __stack_start__;
   dFree = GetStackFreeCount(__stack_start__, __stack_end__);
   TAL_PRINTF("User/System Stack  %4d   %4d   %4d\n", dSize, (dSize - dFree), dFree);
   
   dSize = __stack_irq_end__ - __stack_irq_start__;
   dFree = GetStackFreeCount(__stack_irq_start__, __stack_irq_end__);
   TAL_PRINTF("IRQ Stack          %4d   %4d   %4d\n", dSize, (dSize - dFree), dFree);

   dSize = __stack_fiq_end__ - __stack_fiq_start__;
   dFree = GetStackFreeCount(__stack_fiq_start__, __stack_fiq_end__);
   TAL_PRINTF("FIQ Stack          %4d   %4d   %4d\n", dSize, (dSize - dFree), dFree);

   dSize = __stack_svc_end__ - __stack_svc_start__;
   dFree = GetStackFreeCount(__stack_svc_start__, __stack_svc_end__);
   TAL_PRINTF("Supervisior Stack  %4d   %4d   %4d\n", dSize, (dSize - dFree), dFree);

   dSize = __stack_abt_end__ - __stack_abt_start__;
   dFree = GetStackFreeCount(__stack_abt_start__, __stack_abt_end__);
   TAL_PRINTF("Abort Stack        %4d   %4d   %4d\n", dSize, (dSize - dFree), dFree);

   dSize = __stack_und_end__ - __stack_und_start__;
   dFree = GetStackFreeCount(__stack_und_start__, __stack_und_end__);
   TAL_PRINTF("Undefined Stack    %4d   %4d   %4d\n", dSize, (dSize - dFree), dFree);
   
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
   return(__stack_irq_start__);
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
   return(__stack_irq_end__);
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
   /* Enable IRQ / FIQ */
   __asm__ volatile ( "mrs  r0, cpsr\n"
                      "bic  r0, r0, #0xC0\n"
                      "msr  cpsr, r0\n" );

   /* Call the task. */
   __asm__ volatile ( "pop  {r0-r1}\n"    /* Load argument in r0 */
                      "mov  lr, pc\n"     /* Store pc */
                      "bx   r1\n" );      /* Call the task */
      
   /* Jump to OSTaskExit, in case the task returns. */
   __asm__ volatile ( "bl   OS_TaskExit" );
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
   __asm__ volatile ( "push   {r4-r11, lr}\n"      /* Save registers. */
                      "mrs    r0, cpsr\n"          /* Get status... */    
                      "push   {r0}\n"              /* ...and save status. */

#if defined(MCU_USE_CORTEX_FPU)     
#if defined(__ARM_ARCH_VFP3_D32__) || defined(__ARM_ARCH_VFP4_D32__)
                      "vpush  {s16-s31}\n"         /* Save registers. */ 
                      "vpush  {s0-s15}\n"          /* Save registers. */ 
#else
                      "vpush  {s0-s15}\n"          /* Save registers. */ 
#endif                      
                      "vmrs   r0, fpscr\n"         /* Get status... */    
                      "push   {r0}\n"              /* ...and save status. */
#endif                      
                      
                      "str    sp, %[StackPtr]\n"   /* Save stack pointer. */
                      : : [StackPtr] "o"(RunningTask->StackPtr) : "r0", "memory" );

   /* Set the new task */
   RunningTask = NewTask;
   RunningTask->State = OS_TASK_STATE_RUNNING;

   /*
    * Restore CPU context.
    */
   __asm__ volatile ( "ldr    sp, %[StackPtr]\n"   /* Restore stack pointer. */
   
#if defined(MCU_USE_CORTEX_FPU)        
                      "pop    {r0}\n"              /* Get saved status... */
                      "vmsr   fpscr, r0\n"         /* ...and restore status. */
#if defined(__ARM_ARCH_VFP3_D32__) || defined(__ARM_ARCH_VFP4_D32__)
                      "vpop   {s0-s15}\n"          /* Restore registers. */
                      "vpop   {s16-s31}\n"         /* Restore registers. */
#else
                      "vpop   {s0-s15}\n"          /* Restore registers. */
#endif                      
#endif   
   
                      "pop    {r0}\n"              /* Get saved status... */
                      "msr    cpsr, r0\n"          /* ...and restore status. */
                      "pop    {r4-r11, lr}\n"      /* Restore registers. */
                      "mov    pc, lr\n"            /* Jump to last lr. */
                      : : [StackPtr] "m"(RunningTask->StackPtr) : "r0", "memory"); 
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
   __asm__ volatile ( "ldr    sp, %[StackPtr]\n"   /* Restore stack pointer. */
   
#if defined(MCU_USE_CORTEX_FPU)        
                      "pop    {r0}\n"              /* Get saved status... */
                      "vmsr   fpscr, r0\n"         /* ...and restore status. */
#if defined(__ARM_ARCH_VFP3_D32__) || defined(__ARM_ARCH_VFP4_D32__)
                      "vpop   {s0-s15}\n"          /* Restore registers. */
                      "vpop   {s16-s31}\n"         /* Restore registers. */
#else
                      "vpop   {s0-s15}\n"          /* Restore registers. */
#endif                      
#endif   
   
                      "pop    {r0}\n"              /* Get saved status... */
                      "msr    cpsr, r0\n"          /* ...and restore status. */
                      "pop    {r4-r11, lr}\n"      /* Restore registers. */
                      "mov    pc, lr\n"            /* Jump to last lr. */
                      : : [StackPtr] "m"(RunningTask->StackPtr) : "r0", "memory"); 
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

   /* Disable all interrupts */   
   EnterCritical();
   
   /* Start the SysTick */
//   tal_CPUSysTickStart();

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

   __asm__ volatile ( "ldr    sp, %[StackPtr]\n"      /* Restore stack pointer. */

#if defined(MCU_USE_CORTEX_FPU)
                      "pop    {r0}\n"              /* Get saved status... */
                      "vmsr   fpscr, r0\n"         /* ...and restore status. */
#if defined(__ARM_ARCH_VFP3_D32__) || defined(__ARM_ARCH_VFP4_D32__)
                      "vpop   {s0-s15}\n"          /* Restore registers. */
                      "vpop   {s16-s31}\n"         /* Restore registers. */
#else
                      "vpop   {s0-s15}\n"          /* Restore registers. */
#endif                      
#endif
   
                      "pop    {r0}\n"              /* Get saved status... */
                      "msr    cpsr, r0\n"             /* ...and save in cpsr. */
                      "pop    {r4-r11, lr}\n"      /* Restore registers. */
                      "mov    pc, lr\n"               /* Jump to last lr. */
                      :: [StackPtr] "m"(RunningTask->StackPtr) : "r0", "memory" ); 
                      
} /* OS_Start */

/*************************************************************************/
/*  OS_TaskCreate                                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
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
   sf->cpsr = CPSR_VALUE;  /* System mode, interrupts MUST be enabled by TaskEntry later */
   sf->r4   = 0x04040404;
   sf->r5   = 0x05050505;
   sf->r6   = 0x06060606;
   sf->r7   = 0x07070707;
   sf->r8   = 0x08080808;
   sf->r9   = 0x09090909;
   sf->r10  = 0x10101010;
   sf->r11  = 0x11111111;
   sf->lr   = (uintptr_t)TaskCall;

#if defined(MCU_USE_CORTEX_FPU)
   sf->fpscr = 0x00000000;
   sf->s0    = 0x00000000;
   sf->s1    = 0x01010101;
   sf->s2    = 0x02020202;
   sf->s3    = 0x03030303;
   sf->s4    = 0x04040404;
   sf->s5    = 0x05050505;
   sf->s6    = 0x06060606;
   sf->s7    = 0x07070707;
   sf->s8    = 0x08080808;
   sf->s9    = 0x09090909;
   sf->s10   = 0x10101010;
   sf->s11   = 0x11111111;
   sf->s12   = 0x00000000;
   sf->s13   = 0x00000000;
   sf->s14   = 0x00000000;
   sf->s15   = 0x15151515;
   
#if defined(__ARM_ARCH_VFP3_D32__) || defined(__ARM_ARCH_VFP4_D32__)
   sf->s16   = 0x16161616;
   sf->s17   = 0x17171717;
   sf->s18   = 0x00000000;
   sf->s19   = 0x00000000;
   sf->s20   = 0x00000000;
   sf->s21   = 0x00000000;
   sf->s22   = 0x00000000;
   sf->s23   = 0x00000000;
   sf->s24   = 0x00000000;
   sf->s25   = 0x00000000;
   sf->s26   = 0x00000000;
   sf->s27   = 0x00000000;
   sf->s28   = 0x00000000;
   sf->s29   = 0x00000000;
   sf->s30   = 0x30303030;
   sf->s31   = 0x31313131;
#endif   
#endif    

   pTCB->StackPtr = (uintptr_t)sf;


   TAL_CPU_DISABLE_ALL_INTS();
   
   TaskListAdd(pTCB);         /* Add the task to the TaskList */
   AddTaskToReadyList(pTCB);  /* Add the task to the ReadyList */
   
   /* Schedule the new task if the scheduler is not locked */
   if (0 == bIsSchedLocked)
   {
      AddTaskToReadyList(RunningTask);    
      TaskSchedule();   
   }
   
   TAL_CPU_ENABLE_ALL_INTS();
   
} /* OS_TaskCreate */

/*** EOF ***/
