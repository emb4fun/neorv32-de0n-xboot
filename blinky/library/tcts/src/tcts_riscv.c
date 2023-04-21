/**************************************************************************
*  This file is part of the TCTS project (Tiny Cooperative Task Scheduler)
*
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
#define __TCTS_RISCV_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include <stdint.h>

#include "tcts.h"

uint32_t *pStackPtr;

/*=======================================================================*/
/*  All extern data                                                      */
/*=======================================================================*/

extern uint8_t __stack_start__[];
extern uint8_t __stack_end__[];

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define EnterCritical()    neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE)
#define ExitCritical()     neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE)


/*
 * RISC-V GCC context switch frame layout.
 *
 * This is the layout of the stack after a task's context has been
 * switched-out. The stack pointer is stored in the task control block
 * and points to this structure.
 *
 * The next table indicates the role of each integer and floating-point
 * registers in the calling convention.
 * See: https://riscv.org/wp-content/uploads/2015/01/riscv-calling.pdf
 *
 *    Register  | ABI Name  | Description                    | Saver
 *    ===============================================================
 *    x0          zero        Hard-wired zero                  
 *    x1          ra          Return address                   Caller
 *    x2          sp          Stack pointer                    Callee
 *    x3          gp          Global pointer                   
 *    x4          tp          Thread pointer                   
 *    x5-7        t0-2        Temporaries                      Caller
 *    x8          s0/fp       Saved register/frame pointer     Callee
 *    x9          s1          Saved register                   Callee
 *    x10-11      a0-1        Function arguments/return values Caller
 *    x12-17      a2-7        Function arguments               Caller
 *    x18-27      s2-11       Saved registers                  Callee
 *    x28-31      t36         Temporaries                      Caller
 *    ===============================================================
 *    f0-7        ft0-7       FP temporaries                   Caller
 *    f8-9        fs0-1       FP saved registers               Callee
 *    f10-11      fa0-1       FP arguments/return values       Caller
 *    f12-17      fa2-7       FP arguments                     Caller
 *    f18-27      fs2-11      FP saved registers               Callee
 *    f28-31      ft8-11      FP temporaries                   Caller 
 *    ===============================================================
 *
 * In the beginning only the registers were backed up from the "Caller".
 * This worked, but only if the code was not optimized.
 *
 * However, if the code was compiled with an optimization level, it no
 * longer worked. Only when all registers were saved it will work with
 * an optimization too.
 */
typedef struct  __attribute__((__packed__)) _switch_frame_ // Size = 28 * 4 = 112
{
   uint32_t ra;   /* x1  */
   uint32_t t0;   /* x5  */
   uint32_t t1;   /* x6  */
   uint32_t t2;   /* x7  */
   uint32_t s0;   /* x8  */
   uint32_t s1;   /* x9  */
   uint32_t a0;   /* x10 */
   uint32_t a1;   /* x11 */
   uint32_t a2;   /* x12 */
   uint32_t a3;   /* x13 */
   uint32_t a4;   /* x14 */
   uint32_t a5;   /* x15 */
   uint32_t a6;   /* x16 */
   uint32_t a7;   /* x17 */
   uint32_t s2;   /* x18 */
   uint32_t s3;   /* x19 */
   uint32_t s4;   /* x20 */
   uint32_t s5;   /* x21 */
   uint32_t s6;   /* x22 */
   uint32_t s7;   /* x23 */
   uint32_t s8;   /* x24 */
   uint32_t s9;   /* x25 */
   uint32_t s10;  /* x26 */
   uint32_t s11;  /* x27 */
   uint32_t t3;   /* x28 */
   uint32_t t4;   /* x29 */
   uint32_t t5;   /* x30 */
   uint32_t t6;   /* x31 */
} switch_frame_t;


/*
 * Task call frame layout.
 *
 * This is the stack layout being build to call a new task.
 */
typedef struct  __attribute__((__packed__)) _call_frame_   // Size = 4 * 4 = 64 
{
   uint32_t ra;   /* x1  */
   uint32_t t0;   /* x5  */
   uint32_t t1;   /* x6  */
   uint32_t t2;   /* x7  */
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
   return(NULL);
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
   return(NULL);
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
   __asm__ volatile ( "lw   ra,  0(sp)\n"    // sf->ra = (uint32_t)OS_TaskExit;  /* x1  */
                      "lw   t0,  4(sp)\n"    // sf->t0 = (uint32_t)Task;         /* x5  */
                      "lw   t1,  8(sp)\n"    // sf->t1 = 0x06060606;             /* x6  */
                      "lw   t2, 12(sp)\n"    // sf->t2 = 0x07070707;             /* x7  */
                      "addi sp, sp, 16\n"
                      "jalr x0, t0, 0\n" );

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
   __asm__ volatile ( "addi sp, sp, -112\n" 
                      "sw   ra,    0(sp)\n"
                      "sw   t0,    4(sp)\n"
                      "sw   t1,    8(sp)\n"
                      "sw   t2,   12(sp)\n"
                      "sw   s0,   16(sp)\n"
                      "sw   s1,   20(sp)\n"
                      "sw   a0,   24(sp)\n"
                      "sw   a1,   28(sp)\n"
                      "sw   a2,   32(sp)\n"
                      "sw   a3,   36(sp)\n"
                      "sw   a4,   40(sp)\n"
                      "sw   a5,   44(sp)\n"
                      "sw   a6,   48(sp)\n"
                      "sw   a7,   52(sp)\n"
                      "sw   s2,   56(sp)\n"
                      "sw   s3,   60(sp)\n"
                      "sw   s4,   64(sp)\n"
                      "sw   s5,   68(sp)\n"
                      "sw   s6,   72(sp)\n"
                      "sw   s7,   76(sp)\n"
                      "sw   s8,   80(sp)\n"
                      "sw   s9,   84(sp)\n"
                      "sw   s10,  88(sp)\n"
                      "sw   s11,  92(sp)\n"
                      "sw   t3,   96(sp)\n"
                      "sw   t4,  100(sp)\n"
                      "sw   t5,  104(sp)\n"
                      "sw   t6,  108(sp)\n"
                      "sw   sp, %0"                 /* Save stack pointer. */
                      :: "m" (RunningTask->StackPtr) );


   /* Set the new task */
   RunningTask = NewTask;
   RunningTask->State = OS_TASK_STATE_RUNNING;


   /*
    * Restore CPU context.
    */
   __asm__ volatile ( "lw   sp, %0\n"               /* Restore stack pointer. */
                      "lw   ra,    0(sp)\n"
                      "lw   t0,    4(sp)\n"
                      "lw   t1,    8(sp)\n"
                      "lw   t2,   12(sp)\n"
                      "lw   s0,   16(sp)\n"
                      "lw   s1,   20(sp)\n"
                      "lw   a0,   24(sp)\n"
                      "lw   a1,   28(sp)\n"
                      "lw   a2,   32(sp)\n"
                      "lw   a3,   36(sp)\n"
                      "lw   a4,   40(sp)\n"
                      "lw   a5,   44(sp)\n"
                      "lw   a6,   48(sp)\n"
                      "lw   a7,   52(sp)\n"
                      "lw   s2,   56(sp)\n"
                      "lw   s3,   60(sp)\n"
                      "lw   s4,   64(sp)\n"
                      "lw   s5,   68(sp)\n"
                      "lw   s6,   72(sp)\n"
                      "lw   s7,   76(sp)\n"
                      "lw   s8,   80(sp)\n"
                      "lw   s9,   84(sp)\n"
                      "lw   s10,  88(sp)\n"
                      "lw   s11,  92(sp)\n"
                      "lw   t3,   96(sp)\n"
                      "lw   t4,  100(sp)\n"
                      "lw   t5,  104(sp)\n"
                      "lw   t6,  108(sp)\n"
                      "addi sp, sp, 112\n"
                      "ret\n"
                      :: "m" (RunningTask->StackPtr) );

} /* ContextSwitch */

/*************************************************************************/
/*  ContextSwitchExit                                                    */
/*                                                                       */
/*  Switch to the new task, like ContextSwitch.                          */
/*  But without to save the CPU context. Used by OS_TaskExit.            */
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
   __asm__ volatile ( "lw   sp, %0\n"
                      "lw   ra,    0(sp)\n"
                      "lw   t0,    4(sp)\n"
                      "lw   t1,    8(sp)\n"
                      "lw   t2,   12(sp)\n"
                      "lw   s0,   16(sp)\n"
                      "lw   s1,   20(sp)\n"
                      "lw   a0,   24(sp)\n"
                      "lw   a1,   28(sp)\n"
                      "lw   a2,   32(sp)\n"
                      "lw   a3,   36(sp)\n"
                      "lw   a4,   40(sp)\n"
                      "lw   a5,   44(sp)\n"
                      "lw   a6,   48(sp)\n"
                      "lw   a7,   52(sp)\n"
                      "lw   s2,   56(sp)\n"
                      "lw   s3,   60(sp)\n"
                      "lw   s4,   64(sp)\n"
                      "lw   s5,   68(sp)\n"
                      "lw   s6,   72(sp)\n"
                      "lw   s7,   76(sp)\n"
                      "lw   s8,   80(sp)\n"
                      "lw   s9,   84(sp)\n"
                      "lw   s10,  88(sp)\n"
                      "lw   s11,  92(sp)\n"
                      "lw   t3,   96(sp)\n"
                      "lw   t4,  100(sp)\n"
                      "lw   t5,  104(sp)\n"
                      "lw   t6,  108(sp)\n"
                      "addi sp, sp, 112\n"
                      "ret\n"
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

   __asm__ volatile ( "lw   sp, %0\n"        /* Restore stack pointer. */
                      "lw   ra,    0(sp)\n"  // sf->ra  = (uint32_t)OS_TaskExit; /* x1  */
                      "lw   t0,    4(sp)\n"  // sf->t0  = 0x05050505;            /* x5  */
                      "lw   t1,    8(sp)\n"  // sf->t1  = 0x06060606;            /* x6  */
                      "lw   t2,   12(sp)\n"  // sf->t2  = 0x07070707;            /* x7  */
                      "lw   s0,   16(sp)\n"  // sf->s0  = 0x08080808;            /* x8  */
                      "lw   s1,   20(sp)\n"  // sf->s1  = 0x09090909;            /* x9  */
                      "lw   a0,   24(sp)\n"  // sf->a0  = (uint32_t)pParam;      /* x10 */
                      "lw   a1,   28(sp)\n"  // sf->a1  = 0x11111111;            /* x11 */
                      "lw   a2,   32(sp)\n"  // sf->a2  = 0x12121212;            /* x12 */
                      "lw   a3,   36(sp)\n"  // sf->a3  = 0x13131313;            /* x13 */
                      "lw   a4,   40(sp)\n"  // sf->a4  = 0x14141414;            /* x14 */
                      "lw   a5,   44(sp)\n"  // sf->a5  = 0x15151515;            /* x15 */
                      "lw   a6,   48(sp)\n"  // sf->a6  = 0x16161616;            /* x16 */
                      "lw   a7,   52(sp)\n"  // sf->a7  = 0x17171717;            /* x17 */
                      "lw   s2,   56(sp)\n"  // sf->s2  = 0x18181818;            /* x18 */
                      "lw   s3,   60(sp)\n"  // sf->s3  = 0x19191919;            /* x19 */
                      "lw   s4,   64(sp)\n"  // sf->s4  = 0x20202020;            /* x20 */
                      "lw   s5,   68(sp)\n"  // sf->s5  = 0x21212121;            /* x21 */
                      "lw   s6,   72(sp)\n"  // sf->s6  = 0x22222222;            /* x22 */
                      "lw   s7,   76(sp)\n"  // sf->s7  = 0x23232323;            /* x23 */
                      "lw   s8,   80(sp)\n"  // sf->s8  = 0x24242424;            /* x24 */
                      "lw   s9,   84(sp)\n"  // sf->s9  = 0x25252525;            /* x25 */
                      "lw   s10,  88(sp)\n"  // sf->s10 = 0x26262626;            /* x26 */
                      "lw   s11,  92(sp)\n"  // sf->s11 = 0x27272727;            /* x27 */
                      "lw   t3,   96(sp)\n"  // sf->t3  = 0x28282828;            /* x28 */
                      "lw   t4,  100(sp)\n"  // sf->t4  = 0x29292929;            /* x29 */
                      "lw   t5,  104(sp)\n"  // sf->t5  = 0x30303030;            /* x30 */
                      "lw   t6,  108(sp)\n"  // sf->t6  = 0x31313131;            /* x31 */
                      "addi sp, sp, 112\n"
                      "ret\n"
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

   /* Check 16 byte alignment of the stack */
   addr = (uint32_t)pStack;
   if ((addr & 15) != 0)
   {
      /* Corrected start and size */
      addr += 15;
      addr &= ~15;
      
      wStackSize -= 16;
      
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
   /* I do not know why, but this code calculate a wrong sf pointer. */
   cf = (call_frame_t*)((uint32_t)&pStack[wStackSize] - sizeof(call_frame_t));
   sf = (switch_frame_t*)((uint32_t)cf - sizeof(switch_frame_t));

   /*
    * Setup the call frame to simulate a C function call.
    */
   cf->ra = (uint32_t)OS_TaskExit;
   cf->t0 = (uint32_t)Task;
   cf->t1 = 0x06060606;
   cf->t2 = 0x07070707;

   /*
    * Setup the switch frame.
    */
   sf->ra  = (uint32_t)TaskCall; /* x1  */
   sf->t0  = 0x05050505;         /* x5  */
   sf->t1  = 0x06060606;         /* x6  */
   sf->t2  = 0x07070707;         /* x7  */
   sf->s0  = 0x08080808;         /* x8  */
   sf->s1  = 0x09090909;         /* x9  */
   sf->a0  = (uint32_t)pParam;   /* x10 */
   sf->a1  = 0x11111111;         /* x11 */
   sf->a2  = 0x12121212;         /* x12 */
   sf->a3  = 0x13131313;         /* x13 */
   sf->a4  = 0x14141414;         /* x14 */
   sf->a5  = 0x15151515;         /* x15 */
   sf->a6  = 0x16161616;         /* x16 */
   sf->a7  = 0x17171717;         /* x17 */
   sf->s2  = 0x18181818;         /* x18 */
   sf->s3  = 0x19191919;         /* x19 */
   sf->s4  = 0x20202020;         /* x20 */
   sf->s5  = 0x21212121;         /* x21 */
   sf->s6  = 0x22222222;         /* x22 */
   sf->s7  = 0x23232323;         /* x23 */
   sf->s8  = 0x24242424;         /* x24 */
   sf->s9  = 0x25252525;         /* x25 */
   sf->s10 = 0x26262626;         /* x26 */
   sf->s11 = 0x27272727;         /* x27 */
   sf->t3  = 0x28282828;         /* x28 */
   sf->t4  = 0x29292929;         /* x29 */
   sf->t5  = 0x30303030;         /* x30 */
   sf->t6  = 0x31313131;         /* x31 */
   
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
