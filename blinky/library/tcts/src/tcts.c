/**************************************************************************
*  This file is part of the TCTS project (Tiny Cooperative Task Scheduler)
*
*  Copyright (c) 2014-2023 by Michael Fischer (www.emb4fun.de).
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
*  17.01.2014  mifi  First Version.
*  15.02.2015  mifi  Expanded for Cortex-M.
*  27.09.2015  mifi  Corrected AddTaskToWaitList. TCBFifoAddEnd must used
*                    instead of TCBFifoAddPrio for correct task order.
*  14.11.2015  mifi  Expanded for Cortex-A8.
*  15.11.2015  mifi  Expanded for Nios II.
*  04.01.2016  mifi  Added more timezone functionality.
*                    Corrected "WaitList" bug.
*  10.01.2016  mifi  Rework semaphore handling.
*                    Rework OSTaskCreate, schedule new task if
*                    scheduler is not locked.
*                    Added OSTaskGetPriority and OSTimeDlyUntil.
*                    Added event functionality.
*  04.02.2016  mifi  Remove OSSemaBroadcast and OSSemaBroadcastAsync.
*                    Change OSSemaCreate parameter from uint32 to int32.
*  08.02.2016  mifi  Added "Performance optimisation" switch.
*  12.02.2016  mifi  Added the some more functionality:
*                    - OSTaskTerminateRequest and OSTaskShouldTerminate
*                    - OSTimerSet
*                    - OSSemaReset
*  05.05.2016  mifi  Added incomplete Mailbox functionality, only:
*                    - OSMboxCreate
*                    - OSMboxDelete
*                    - OSMboxPostFromInt
*                    - OSMboxWait
*  16.06.2016  mifi  Added Cortex-M0+ support.
*  12.05.2018  mifi  Added TaskScheduleExit support.
*  23.06.2018  mifi  Change version pattern.
*                    Rework OS_WAIT_INFINITE handling.
*                    Added mutex functionality
*                    - OSMutexCreate
*                    - OSMutexSignal
*                    - OSMutexSignalFromInt
*                    - OSMutexWait
*                    Change version to v0.18.1
*  26.05.2020  mifi  Change API from OSxx to OS_xx.
*                    Change version to v0.19.0.
*  24.02.2021  mifi  Change version to v0.20.0.
*                    Added Software Watchdog support.
*  25.02.2021  mifi  Change version to v0.21.0.
*                    Added OS_TaskWakeup and OS_TaskIsSleeping.
*  16.05.2021  mifi  Change version to v0.22.0.
**************************************************************************/
#define __TCTS_C__

#if defined(RTOS_TCTS)

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include <stdint.h>

#include "tcts.h"
#include "tal.h"

/*=======================================================================*/
/*  All extern data                                                      */
/*=======================================================================*/

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*
 * Check if "performacne optimisation" is enabled.
 * If this option is enabled the statistic functionality is limited.
 * No task based time information are available in this case.
 */
#if !defined(TCTS_ENABLE_PERFORMANCE_OPTIMISATION_NO_STATISTIC)
   Error: TCTS_ENABLE_PERFORMANCE_OPTIMISATION_NO_STATISTIC must be defined in tcts_conf.h;
#else
#define _PERF_OPT_NO_STATISTIC   TCTS_ENABLE_PERFORMANCE_OPTIMISATION_NO_STATISTIC
#endif


/*
 * Watchdog SW and HW
 */
#if !defined(TCTS_USE_SWWD)
#define _USE_SWWD    0
#else
#define _USE_SWWD    TCTS_USE_SWWD
#endif

#if !defined(TCTS_USE_HWWD)
#define _USE_HWWD    0
#else
#define _USE_HWWD    TCTS_USE_HWWD
#endif


/*
 * Idle, statistic and SW-Watchdog stack size
 */
#if !defined(OS_IDLE_STACK_SIZE)
#define OS_IDLE_STACK_SIZE    512
#endif

#if !defined(OS_STAT_STACK_SIZE)
#define OS_STAT_STACK_SIZE    512
#endif

#if !defined(OS_SWDOG_STACK_SIZE)
#define OS_SWWD_STACK_SIZE    512
#endif

/*
 * Idle, statistic and SW-Watchdog task priority
 */
#define IDLE_TASK_PRIO        255
#define STAT_TASK_PRIO        254
#define SWDOG_TASK_PRIO       3

/*
 * Test if the fifo is empty
 */
#define IsFifoEmpty(_f)       (((_f)->pIn == NULL) ? 1 : 0)

/*
 * Set state of a task
 */
#if (_PERF_OPT_NO_STATISTIC >= 1)
#define SET_TASK_STATE(_a,_b)
#else
#define SET_TASK_STATE(_a,_b) { _a->State = _b; }
#endif

/*
 * Software Watchdog
 */
#define TASK_SWDOG_DELAY_TIME_MS    500

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*
 * Ide, Statistic and SW-Watchdog task variables
 */
static OS_STACK(IdleStack, OS_IDLE_STACK_SIZE);
static OS_STACK(StatStack, OS_STAT_STACK_SIZE);

#if (_USE_SWWD >= 1)
static OS_STACK(SWDogStack, OS_SWWD_STACK_SIZE);
#endif

static OS_TCB TCBIdle;
static OS_TCB TCBStat;
static OS_TCB TCBSWDog;

/*
 * Scheduler lock status
 */
static uint8_t bIsSchedLocked;

/*
 * Time variables
 */
static volatile uint32_t dSystemTick = 0;
static volatile uint32_t dMilliSecCnt = 0;
static volatile uint32_t dSecTick    = 0;
static volatile uint32_t dUnixtime   = 0;

static int16_t   wTimezoneID           = 0;
static int16_t   wTimezoneOffsetMin    = 0;
static int32_t   nTimezoneOffsetSec    = 0;
static int32_t   dTimezoneDstOffsetSec = 0;

/*
 * Some statistics variables
 */
static uint8_t   bStatEnabled       = 0;
static uint8_t   bStatCPULoad       = 0;
static uint32_t  dStatTotalTaskTime = 0;
static uint32_t  dStatHirResPeriod  = 0;

/*
 * The TaskList contains all the tasks of the system.
 */
static OS_TCB *pTaskList = NULL;

/*
 * The ReadyList contains the tasks which are ready to run.
 */
static os_tcb_fifo_t ReadyList;

/*
 * The WaitList contains the tasks which are waiting
 * with timeout, e.g. OSTimeDly, OSSemaWait or OSEventWait.
 */
static os_tcb_fifo_t WaitList;

/*
 * This is the actual running task.
 */
static OS_TCB *RunningTask = NULL;

/*
 * This is the task which will run after the context switch.
 */
static OS_TCB *NewTask = NULL;

static OS_TIMER_FUNC UserTimer = NULL;
static uint32_t      UserTimerTimeout = 0;

/*
 * Software Watchdog
 */
static uint8_t bSWDogEnabled = 0;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*
 * Check for ARM7 style CPU
 */
#if defined(__ARM_ARCH_4T__) || defined(__ARM_ARCH_7A__)
#include "tcts_arm.c"
#endif

/*
 * Check for Cortex-M style CPU
 */
#if defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7M__)
#include "tcts_cm.c"
#endif

/*
 * Check for Cortex-M0 style CPU
 */
#if defined(__ARM_ARCH_6M__)
#include "tcts_cm0plus.c"
#endif

/*
 * Check for Nios II style CPU
 */
#if defined(__NIOS__)
#include "tcts_nios.c"
#endif

/*
 * Check for RISC-V style CPU
 */
#if defined(__ARCH_RISCV__)
#include "tcts_riscv.c"
#endif


/*************************************************************************/
/*  GetStackFreeCount                                                    */
/*                                                                       */
/*  The stack was initialized with 0xCC.                                 */
/*  Unused entries contains still 0xCC.                                  */
/*                                                                       */
/*  In    : pStart, pEnd                                                 */
/*  Out   : none                                                         */
/*  Return: Free entries of the stack                                    */
/*************************************************************************/
static uint32_t GetStackFreeCount (uint8_t *pStart, uint8_t *pEnd)
{
   uint32_t dFreeCount = 0;

   while (pStart < pEnd)
   {
      if (0xCC == *pStart)
      {
         dFreeCount++;
      }
      else
      {
         break;
      }
      pStart++;
   }

   return(dFreeCount);
} /* GetStackFreeCount */

/*************************************************************************/
/*  TCBFifoAddPrio                                                       */
/*                                                                       */
/*  Add the task to the FIFO, with respect to the priority.              */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pFifo, pTask                                                 */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static __inline__ void TCBFifoAddPrio (os_tcb_fifo_t *pFifo, OS_TCB *pTask)
{
   OS_TCB *pList;
   int      TaskWasAdded = 0;

#if 0 /* TODO: double check if it is a good idea */
   /* The Idle task must not added to any lists */
   if (&TCBIdle == pTask)
   {
      return;
   }
#endif

   /* Check if the fifo is empty */
   if (NULL == pFifo->pIn)
   {
      /* Fifo is empty */
      pTask->pPrev = NULL;    /* Task has no prev */
      pTask->pNext = NULL;    /* and no next */

      pFifo->pIn  = pTask;    /* Fifo in and out */
      pFifo->pOut = pTask;    /* points to the new task */
   }
   else
   {
      pList = pFifo->pIn;
      while (pList != NULL)
      {
         if (pTask->nPrio >= pList->nPrio)
         {
            if (pList->pPrev == NULL)
            {
               /* Insert at the front of the fifo */
               pTask->pPrev = NULL;          /* This is the new first task, therefore no prev. */
               pTask->pNext = pFifo->pIn;    /* Link the new task in front of the fifo. */
               pFifo->pIn->pPrev = pTask;    /* Link the second task (old first) back to the new first task. */
               pFifo->pIn = pTask;           /* Set the fifo->pIn to the new first task. */
            }
            else
            {
               /* Insert in the middle of the fifo */
               pTask->pNext = pList;
               pTask->pPrev = pList->pPrev;

               pList->pPrev->pNext = pTask;
               pList->pPrev        = pTask;
            }
            TaskWasAdded = 1;
            break;
         } /* end if (pTask->Prio >= pList->Prio) */

         pList = pList->pNext;
      } /* end while (pList != NULL) */

      /* Check if task was not added */
      if (0 == TaskWasAdded)
      {
         /* Add Task at the end of the fifo */
         pTask->pNext = NULL;
         pTask->pPrev = pFifo->pOut;

         pFifo->pOut->pNext = pTask;
         pFifo->pOut        = pTask;
      }
   }

} /* TCBFifoAddPrio */

/*************************************************************************/
/*  TCBFifoRemove                                                        */
/*                                                                       */
/*  Remove a task from the FIFO, output side, and return the task.       */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pFifo                                                        */
/*  Out   : none                                                         */
/*  Return: pTask                                                        */
/*************************************************************************/
static __inline__ OS_TCB *TCBFifoRemove (os_tcb_fifo_t *pFifo)
{
   OS_TCB *pTask;

   /* Check if the fifo is empty */
   if (NULL == pFifo->pOut)
   {
      /* Error */
      TAL_FAILED();
      return(NULL);
   }

   /* Check if only one element is in the fifo */
   if (pFifo->pIn == pFifo->pOut)
   {
      /* Only one element is in the fifo */
      pTask = pFifo->pOut;

      /* Clear the fifo in/out */
      pFifo->pIn  = NULL;
      pFifo->pOut = NULL;
   }
   else
   {
      pTask = pFifo->pOut;          /* Get the last task of the fifo */
      pFifo->pOut = pTask->pPrev;   /* Move the last pointer to the task before the end */
      pFifo->pOut->pNext = NULL;    /* Remove the link from the new last element to the next one */
   }

#if 1 /* TODO: Is this realy needed ? */
   /* Clear fifo pointer */
   pTask->pPrev = NULL;
   pTask->pNext = NULL;
#endif

   return(pTask);
} /* TCBFifoRemove */

/*************************************************************************/
/*  TCBFifoRemoveMiddle                                                  */
/*                                                                       */
/*  Remove a task from the FIFO.                                         */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pFifo, pTask                                                 */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static __inline__ void TCBFifoRemoveMiddle (os_tcb_fifo_t *pFifo, OS_TCB *pTask)
{
   /* Check if fifo is empty */
   if (NULL == pFifo->pIn)
   {
      /* Error */
      TAL_FAILED();
   }

   /* Check if only one element is in the list */
   if (pFifo->pIn == pFifo->pOut)
   {
      /* Clear the fifo in/out */
      pFifo->pIn  = NULL;
      pFifo->pOut = NULL;
   }
   else
   {
      /* Check if the task is in the middle of the list */
      if      ((pTask->pPrev != NULL) && (pTask->pNext != NULL))
      {
         pTask->pNext->pPrev = pTask->pPrev;
         pTask->pPrev->pNext = pTask->pNext;
      }
      /* Check for the first element */
      else if (pTask->pPrev == NULL)
      {
         pTask->pNext->pPrev = NULL;
         pFifo->pIn = pTask->pNext;
      }
      /* Check for the last element */
      else if (pTask->pNext == NULL)
      {
         pTask->pPrev->pNext = NULL;
         pFifo->pOut = pTask->pPrev;
      }
   }

   pTask->pNext = NULL;
   pTask->pPrev = NULL;
} /* TCBFifoRemoveMiddle */

/*************************************************************************/
/*  TaskListAdd                                                          */
/*                                                                       */
/*  Add the task to the TaskList.                                        */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pTask                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void TaskListAdd (OS_TCB *pTask)
{
   /* Check if the list is empty */
   if (NULL == pTaskList)
   {
      /* List is empty */
      pTask->pTaskPrev = NULL;
      pTask->pTaskNext = NULL;

      pTaskList = pTask;
   }
   else
   {
      pTask->pTaskPrev = NULL;

      /*
       * Link the new task in front of the fifo.
       */
      pTask->pTaskNext = pTaskList;

      /*
       * Link the old "first task" back to the new task.
       */
      pTaskList->pTaskPrev = pTask;

      /*
       * Set the new fifo->pIn.
       */
      pTaskList = pTask;
   }

} /* TaskListAdd */

/*************************************************************************/
/*  TaskListRemove                                                       */
/*                                                                       */
/*  Remove a task from the TaskList.                                     */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pTask                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void TaskListRemove (OS_TCB *pTask)
{
   /* Check if only one element is in the list */
   if ((NULL == pTaskList->pTaskPrev) && (NULL == pTaskList->pTaskNext))
   {
      /* Clear the list */
      pTaskList = NULL;
   }
   else
   {
      /* Check if the task is in the middle of the list */
      if      ((pTask->pTaskPrev != NULL) && (pTask->pTaskNext != NULL))
      {
         pTask->pTaskNext->pTaskPrev = pTask->pTaskPrev;
         pTask->pTaskPrev->pTaskNext = pTask->pTaskNext;
      }
      /* Check for the first element */
      else if (pTask->pTaskPrev == NULL)
      {
         pTask->pTaskNext->pTaskPrev = NULL;
         pTaskList = pTask->pTaskNext;
      }
      /* Check for the last element */
      else if (pTask->pTaskNext == NULL)
      {
         pTask->pTaskPrev->pTaskNext = NULL;
      }
   }

} /* TaskListRemove */

/*************************************************************************/
/*  AddTaskToWaitList                                                    */
/*                                                                       */
/*  Add the task to the WaitList fifo.                                   */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pTask                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static __inline__ void AddTaskToWaitList (OS_TCB *pTask)
{
   os_tcb_fifo_t *pFifo = &WaitList;

   SET_TASK_STATE(pTask, OS_TASK_STATE_WAITING);

   /*
    * For the correct order of task processing the task must be
    * added at the end. For example, if there exist three tasks, A, B and C
    * with the same priority. And the tasks was started in the ABC order the
    * WaitList looks like A->B->C after all tasks was added to the WaitList
    * now.
    *
    * CheckWaitList will move the tasks later from the WaitList to
    * the ReadyList if the timeout is expired. But this will be
    * done with TCBFifoAddPrio. The ReadyList will look like C->B->A.
    *
    * The task which is started is taken by the scheduler from the
    * end of the ReadyList. Now the order of task processing is correct.
    */

   /* The Idle task must not added to any lists */
   if (&TCBIdle == pTask)
   {
      return;
   }

   /* Check if the fifo is empty */
   if (NULL == pFifo->pIn)
   {
      /* Fifo is empty */
      pTask->pWaitPrev = NULL;         /* Task has no prev */
      pTask->pWaitNext = NULL;         /* and no next */

      pFifo->pIn  = pTask;             /* Fifo in and out */
      pFifo->pOut = pTask;             /* points to the new task */
   }
   else
   {
      /* Fifo is not empty */
      pTask->pWaitNext = NULL;         /* This is the new last task, therefore no next. */
      pTask->pWaitPrev = pFifo->pOut;  /* Link the new task at the end of the fifo. */
      pFifo->pOut->pWaitNext = pTask;  /* Link the old last to the new last. */
      pFifo->pOut = pTask;             /* Set the fifo-pOut to the new last task. */
   }

} /* AddTaskToWaitList */

/*************************************************************************/
/*  WaitListRemove                                                       */
/*                                                                       */
/*  Remove a task from the WaitList fifo.                                */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pTask                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void WaitListRemove (OS_TCB *pTask)
{
   os_tcb_fifo_t *pFifo = &WaitList;

   /* Check if fifo is empty */
   if (NULL == pFifo->pIn)
   {
      /* Error */
      TAL_FAILED();
   }

   /* Check if only one element is in the list */
   if (pFifo->pIn == pFifo->pOut)
   {
      /* Clear the fifo in/out */
      pFifo->pIn  = NULL;
      pFifo->pOut = NULL;
   }
   else
   {
      /* Check if the task is in the middle of the list */
      if      ((pTask->pWaitPrev != NULL) && (pTask->pWaitNext != NULL))
      {
         pTask->pWaitNext->pWaitPrev = pTask->pWaitPrev;
         pTask->pWaitPrev->pWaitNext = pTask->pWaitNext;
      }
      /* Check for the first element */
      else if (pTask->pWaitPrev == NULL)
      {
         pTask->pWaitNext->pWaitPrev = NULL;
         pFifo->pIn = pTask->pWaitNext;
      }
      /* Check for the last element */
      else if (pTask->pWaitNext == NULL)
      {
         pTask->pWaitPrev->pWaitNext = NULL;
         pFifo->pOut = pTask->pWaitPrev;
      }
   }

   pTask->pWaitPrev = NULL;
   pTask->pWaitNext = NULL;
} /* WaitListRemove */

/*************************************************************************/
/*  AddTaskToReadyList                                                   */
/*                                                                       */
/*  Add the task to the Ready list                                       */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pTask                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static __inline__ void AddTaskToReadyList (OS_TCB *pTask)
{
   SET_TASK_STATE(pTask, OS_TASK_STATE_READY);
   TCBFifoAddPrio(&ReadyList, pTask);
} /* AddTaskToReadyList */

/*************************************************************************/
/*  GetReadyTask                                                         */
/*                                                                       */
/*  Get the task which is ready.                                         */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pTask                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static OS_TCB *GetReadyTask (void)
{
   OS_TCB *pTask;

#if 0 /* TODO: double check if it is a good idea */
   /* In case the ReadyList fifo is empty, return the Idle task */
   if (NULL == ReadyList.pIn)
   {
      pTask = &TCBIdle;
   }
   else
#endif
   {
      pTask = TCBFifoRemove(&ReadyList);
   }

   return(pTask);
} /* GetReadyTask */

/*************************************************************************/
/*  HandleWaitList                                                       */
/*                                                                       */
/*  The WaitList is used for handling a timeout or a delay.              */
/*                                                                       */
/*  Go through the WaitList and decrease the WaitCount. Check if the     */
/*  task from the WaitList can be moved to the ReadyList.                */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void HandleWaitList (void)
{
   OS_TCB *pTask = WaitList.pIn;
   OS_TCB *pNext;

   while (pTask != NULL)
   {
      /* Decrease the WaitCount if allowed */
      if ((pTask->dTimeoutTicks > 0) && (pTask->dTimeoutTicks != OS_WAIT_INFINITE))
      {
         pTask->dTimeoutTicks--;
      }

      /* Check if the timeout is expired */
      if (0 == pTask->dTimeoutTicks)
      {
         /*
          * Save the next pointer of the task.
          * pTask->pNext could not be used later, because it wil be changed
          * by the TCBFifoAdd before we can use the Next for the loop.
          */
         pNext = pTask->pWaitNext;

         /* Remove the task from the WaitList */
         WaitListRemove(pTask);

         /* Check if the task was waiting for a semaphore */
         if (pTask->pSemaWait)
         {
            /*
             * This is a timeout of the semaphore,
             * the task must be removed from the semaphore list too.
             */
            TCBFifoRemoveMiddle(&pTask->pSemaWait->Fifo, pTask);
            pTask->pSemaWait   = NULL;
            pTask->nReturnCode = OS_RC_TIMEOUT;  /* Return code for OSSemaWait */
         }

         /* Check if the task was waiting for a mutex */
         if (pTask->pMutexWait)
         {
            /*
             * This is a timeout of the mutex,
             * the task must be removed from the mutex list too.
             */
            TCBFifoRemoveMiddle(&pTask->pMutexWait->Fifo, pTask);
            pTask->pMutexWait  = NULL;
            pTask->nReturnCode = OS_RC_TIMEOUT;  /* Return code for OSMutexWait */
         }

         /* Check if the task was waiting for an event */
         if (pTask->pEventWait)
         {
            /*
             * This is a timeout of the event,
             * the task must be removed from the event list too.
             */
            TCBFifoRemoveMiddle(&pTask->pEventWait->Fifo, pTask);
            pTask->pEventWait  = NULL;
            pTask->nReturnCode = OS_RC_TIMEOUT;  /* Return code for OSEventWait */
         }

         /* Add the task to the ReadyList */
         AddTaskToReadyList(pTask);

         /* Loop through the list */
         pTask = pNext;
      }
      else
      {
         /* Loop through the list */
         pTask = pTask->pWaitNext;
      }
   } /* end while (pTask != NULL) */

} /* HandleWaitList */

/*************************************************************************/
/*  StatUpdateTime                                                       */
/*                                                                       */
/*  Update the statistic time information from a task.                   */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pTask                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void StatUpdateTime (OS_TCB *pTask)
{
   uint16_t wStartTick;
   uint16_t wEndTick;
   uint16_t wStartTimer;
   uint16_t wEndTimer;
   uint16_t wDeltaTick;
   uint16_t wDeltaTimer;

   wStartTimer = pTask->dStatStartTime & 0xFFFF;
   wStartTick  = pTask->dStatStartTime >> 16;
   wEndTimer   = pTask->dStatEndTime & 0xFFFF;
   wEndTick    = pTask->dStatEndTime >> 16;

   wDeltaTick = wEndTick - wStartTick;

   if (wEndTimer > wStartTimer)
   {
      wDeltaTimer = wEndTimer - wStartTimer;
   }
   else
   {
      wDeltaTimer = (uint16_t)(dStatHirResPeriod - wStartTimer) + wEndTimer;
   }

   pTask->dStatTotalTime += (wDeltaTick * dStatHirResPeriod) + wDeltaTimer;

} /* StatUpdateTime */

/*************************************************************************/
/*  TaskSchedule                                                         */
/*                                                                       */
/*  Must be called with interrupts disabled.                             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void TaskSchedule (void)
{
   /*
    * Get new task from the ReadyList.
    */
   NewTask = GetReadyTask();

   /*
    * If NewTask == RunningTask, we does not need a task switch
    */
   if (NewTask != RunningTask)
   {
#if (_PERF_OPT_NO_STATISTIC >= 1)
      /* Context switch needed */
      ContextSwitch();
#else
      if (!bStatEnabled)
      {
         /* Context switch needed */
         ContextSwitch();
      }
      else
      {
         /* Get task end time for statistics */
         RunningTask->dStatEndTime = tal_CPUStatGetHiResCnt();
         StatUpdateTime(RunningTask);

         /* Context switch needed */
         ContextSwitch();

         /* Get task start time for statistics */
         RunningTask->dStatStartTime = tal_CPUStatGetHiResCnt();
      }
#endif /* (_PERF_OPT_NO_STATISTIC >= 1) */
   }

} /* TaskSchedule */

/*************************************************************************/
/*  TaskScheduleExit                                                     */
/*                                                                       */
/*  Must be called with interrupts disabled.                             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void TaskScheduleExit (void)
{
   /*
    * Get new task from the ReadyList.
    */
   NewTask = GetReadyTask();

   /*
    * If NewTask == RunningTask, we does not need a task switch
    */
   if (NewTask != RunningTask)
   {
#if (_PERF_OPT_NO_STATISTIC >= 1)
      /* Context switch needed */
      ContextSwitchExit();
#else
      if (!bStatEnabled)
      {
         /* Context switch needed */
         ContextSwitchExit();
      }
      else
      {
         /* Get task end time for statistics */
         RunningTask->dStatEndTime = tal_CPUStatGetHiResCnt();
         StatUpdateTime(RunningTask);

         /* Context switch needed */
         ContextSwitchExit();

         /* Get task start time for statistics */
         RunningTask->dStatStartTime = tal_CPUStatGetHiResCnt();
      }
#endif /* (_PERF_OPT_NO_STATISTIC >= 1) */
   }

} /* TaskScheduleExit */

#if (_USE_SWWD >= 1)
/*************************************************************************/
/*  InitHWDog                                                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void InitHWDog (void)
{
#if (_USE_HWWD >= 1)
   tal_CPUInitHWDog();
#endif
} /* InitHWDog */

/*************************************************************************/
/*  TriggerHWDog                                                         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void TriggerHWDog (void)
{
#if (_USE_HWWD >= 1)
   tal_CPUTriggerHWDog();
#endif
} /* TriggerHWDog */

/*************************************************************************/
/*  SWDogTask                                                            */
/*                                                                       */
/*  In    : Task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void SWDogTask (void *pParam)
{
   OS_TCB   *pTCB;
   uint32_t  dDelayTicks = OS_MS_2_TICKS(TASK_SWDOG_DELAY_TIME_MS);

   (void)pParam;

   bSWDogEnabled = 1;

   /* Enable the hardware watchdog */
   InitHWDog();

   /* And feed the dog the first time */
   TriggerHWDog();

   while (1)
   {
      OS_TimeDly(dDelayTicks);

      /* Trigger the HW-Dog here */
      TriggerHWDog();

      /* Get task list start */
      pTCB = OS_TaskGetList();

      /* Check timeout of each task */
      while (pTCB != NULL)
      {
         /* Check if task use the SWDog */
         if (pTCB->lSWDogTimeoutMax != 0)
         {
            pTCB->lSWDogTimeout -= (int32_t)dDelayTicks;
            if (pTCB->lSWDogTimeout < 0)
            {
               /* Task timeout, reset system */
               tal_CPUReboot();
            }
         }

         pTCB = pTCB->pTaskNext;
      } /* end while (pTCB != NULL) */

   } /* end hile (1) */

} /* SWDogTask */
#endif /* (_USE_SWWD >= 1) */

/*************************************************************************/
/*  StatTask                                                             */
/*                                                                       */
/*  In    : Task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void StatTask (void *pParam)
{
   OS_TCB   *pTask;
   uint32_t  dIdleTime = 0;
   uint32_t  dStatUsage;

   (void)pParam;

   bStatEnabled      = 1;
   dStatHirResPeriod = tal_CPUStatGetHiResPeriod();

   while (1)
   {
      if (bStatEnabled)
      {
         /* Get total task time */
         dStatTotalTaskTime = 0;

         /* Collect total task time first */
         pTask = pTaskList;
         while (pTask != NULL)
         {
            /* Count total TaskTime */
            dStatTotalTaskTime += pTask->dStatTotalTime;

            /* Find Idle time */
            if (IDLE_TASK_PRIO == pTask->nPrio)
            {
               dIdleTime = pTask->dStatTotalTime;
            }

            pTask->dStatTotalLastTime = pTask->dStatTotalTime;
            pTask->dStatTotalTime     = 0;

            pTask = pTask->pTaskNext;
         }


         if (0 == dStatTotalTaskTime) dStatTotalTaskTime = 1;

         /* Calculate usage in percent */
         pTask = pTaskList;
         while (pTask != NULL)
         {
            dStatUsage = (uint32_t)(((uint64_t)10000 * pTask->dStatTotalLastTime) / dStatTotalTaskTime);
            if (dStatUsage > 10000)
            {
               dStatUsage = 10000;
            }
            pTask->dStatUsage = dStatUsage;

            pTask = pTask->pTaskNext;
         }

         /*
          * Calculate CPU load like:
          *
          *   bStatCPULoad = 100 - (uint8_t)((dIdleTime * 100) / dStatTotalTaskTime)
          *
          * But this makes problems with big counts. Therefore
          * dIdleTime will not multiplied instead dStatTotalTaskTime
          * will be divided by 100.
          *
          *   bStatCPULoad = 100 - (uint8_t)(dIdleTime / (dStatTotalTaskTime / 100))
          */
         if (dStatTotalTaskTime > 100) /* Prevent division by zero */
         {
            bStatCPULoad = 100 - (uint8_t)(dIdleTime / (dStatTotalTaskTime / 100));
         }
         else
         {
            bStatCPULoad = 1;
         }
      }

      OS_TimeDly(500);
   }

} /* StatTask */

/*************************************************************************/
/*  IdleTask                                                             */
/*                                                                       */
/*  In    : Task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void IdleTask (void *pParam)
{
   (void)pParam;

   while (1)
   {
      /* Do not call any other OS function than OSTaskYield here */
      OS_TaskYield();
   }

} /* IdleTask */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  OS_TCTS_Init                                                         */
/*                                                                       */
/*  Init the "Cooperative Task Scheduler".                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TCTS_Init (void)
{
   /* Init the Ready and Wait List */
   memset(&ReadyList, 0x00, sizeof(ReadyList));
   memset(&WaitList,  0x00, sizeof(WaitList));

   bIsSchedLocked = 1;     /* Used in tcts_cm, _arm and _nios */

   /* Create the Idle task */
   OS_TaskCreate(&TCBIdle, IdleTask, NULL, IDLE_TASK_PRIO,
                 IdleStack, sizeof(IdleStack), "[IdleTask]");

   /*
    * Set dStatTotalTaskTime to 1, to prevent
    * division by 0 in case statistic is not enabled.
    */
   dStatTotalTaskTime = 1;

   (void)bIsSchedLocked;   /* Prevent lint warning */
} /* OS_TCTS_Init */

/*************************************************************************/
/*  OS_SysTickStart                                                      */
/*                                                                       */
/*  Start the SysTick.                                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
void OS_SysTickStart (void)
{
   tal_CPUSysTickStart();
} /* OS_SysTickStart */

/*************************************************************************/
/*  OS_OutputRuntimeStackInfo                                            */
/*                                                                       */
/*  Output the runtime stack information.                                */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_OutputRuntimeStackInfo (void)
{
   OutputRuntimeStackInfo();
} /* OS_OutputRuntimeStackInfo */

/*************************************************************************/
/*  OS_OutputTaskInfo                                                    */
/*                                                                       */
/*  Output the Stack and Time information from the tasks.                */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_OutputTaskInfo (void)
{
   OS_TCB   *pTCB;
   uint8_t  *pStart;
   uint8_t  *pEnd;
   uint32_t  dSize;
   uint32_t  dFree;
   uint16_t  wTime1 = 0;
   uint16_t  wTime2 = 0;
   uint8_t   bDog;

   TAL_PRINTF("*** Task Info ***\n");
   TAL_PRINTF("\n");
   TAL_PRINTF("Task              Prio   Size   Used   Free   Dog   CPU %%\n");
   TAL_PRINTF("==========================================================\n");

   /* Display "IRQ" Stack */
   pStart = GetIRQStackStart();
   pEnd   = GetIRQStackEnd();
   dSize  = (uint32_t)pEnd - (uint32_t)pStart;
   if (dSize != 0)
   {
      dFree  = GetStackFreeCount(pStart, pEnd);

      TAL_PRINTF("%-16s  ", "- IRQ -");
      TAL_PRINTF("  --   %4d   %4d   %4d    -   -----\n", dSize, (dSize - dFree), dFree);
   }


   /* Get start of list */
   pTCB = OS_TaskGetList();

   while (pTCB != NULL)
   {
      /* Do not handle Statistic and Idle here, will be handled at the end */
      if ((pTCB != &TCBStat) && (pTCB != &TCBIdle) && (pTCB != &TCBSWDog))
      {
         if (pTCB->Name[0] != 0)
         {
            TAL_PRINTF("%-16s  ", pTCB->Name);
         }
         else
         {
            TAL_PRINTF("%-16s  ", "----------------");
         }

         dFree  = OS_GetStackInfo (pTCB, &dSize);
         wTime1 = (uint16_t)(pTCB->dStatUsage / 100);
         wTime2 = (uint16_t)(pTCB->dStatUsage % 100);

#if (_USE_SWWD >= 1)
         bDog = (pTCB->lSWDogTimeoutMax != 0) ? 'x' : '-';
#else
         bDog = '-';
#endif

         TAL_PRINTF("%4d   %4d   %4d   %4d    %c   %2d.%02d\n", pTCB->nPrio, dSize, (dSize - dFree), dFree, bDog, wTime1, wTime2);
      }

      pTCB = pTCB->pTaskNext;
   }

   /*
    * Display Software Watchdog task info
    */
   if (bSWDogEnabled)
   {
      pTCB   = &TCBSWDog;
      dFree  = OS_GetStackInfo (&TCBSWDog, &dSize);
      wTime1 = (uint16_t)(pTCB->dStatUsage / 100);
      wTime2 = (uint16_t)(pTCB->dStatUsage % 100);

      TAL_PRINTF("%-16s  ", pTCB->Name);
      TAL_PRINTF("%4d   %4d   %4d   %4d    -   %2d.%02d\n", pTCB->nPrio, dSize, (dSize - dFree), dFree, wTime1, wTime2);
   }

   /*
    * Display Statistic task info
    */
   if (bStatEnabled)
   {
      pTCB   = &TCBStat;
      dFree  = OS_GetStackInfo (&TCBStat, &dSize);
      wTime1 = (uint16_t)(pTCB->dStatUsage / 100);
      wTime2 = (uint16_t)(pTCB->dStatUsage % 100);

      TAL_PRINTF("%-16s  ", pTCB->Name);
      TAL_PRINTF("%4d   %4d   %4d   %4d    -   %2d.%02d\n", pTCB->nPrio, dSize, (dSize - dFree), dFree, wTime1, wTime2);
   }

   /*
    * Display Idle task info
    */
   pTCB   = &TCBIdle;
   dFree  = OS_GetStackInfo (&TCBIdle, &dSize);
   wTime1 = (uint16_t)(pTCB->dStatUsage / 100);
   wTime2 = (uint16_t)(pTCB->dStatUsage % 100);

   TAL_PRINTF("%-16s  ", pTCB->Name);
   TAL_PRINTF("%4d   %4d   %4d   %4d    -   %2d.%02d\n", pTCB->nPrio, dSize, (dSize - dFree), dFree, wTime1, wTime2);

   TAL_PRINTF("\n--- CPU load: %d%% ---\n", OS_StatGetCPULoad());

   (void)dSize;
   (void)dFree;
   (void)wTime1;
   (void)wTime2;

   TAL_PRINTF("\n");
} /* OS_OutputTaskInfo */

/*************************************************************************/
/*  OS_OutputSWDogInfo                                                   */
/*                                                                       */
/*  Output the Software Watchdog information from the tasks.             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_OutputSWDogInfo (void)
{
   OS_TCB   *pTCB;
   uint8_t   bDog;

   TAL_PRINTF("*** Software Watchdog Info ***\n");
   TAL_PRINTF("\n");
   TAL_PRINTF("Task              Prio   Dog   Time   Min \n");
   TAL_PRINTF("==========================================\n");

   /* Get start of list */
   pTCB = OS_TaskGetList();

   while (pTCB != NULL)
   {
      /* Do not handle Statistic and Idle here, will be handled at the end */
      if ((pTCB != &TCBStat) && (pTCB != &TCBIdle) && (pTCB != &TCBSWDog))
      {
         if (pTCB->Name[0] != 0)
         {
            TAL_PRINTF("%-16s  ", pTCB->Name);
         }
         else
         {
            TAL_PRINTF("%-16s  ", "----------------");
         }

#if (_USE_SWWD >= 1)
         bDog = (pTCB->lSWDogTimeoutMax != 0) ? 1 : 0;
#else
         bDog = 0;
#endif

         if (1 == bDog)
         {
            TAL_PRINTF("%4d    x   %5d  %5d\n", pTCB->nPrio, pTCB->lSWDogTimeoutMax, pTCB->lSWDogTimeout);
         }
         else
         {
            TAL_PRINTF("%4d    -\n", pTCB->nPrio, bDog);
         }
      }

      pTCB = pTCB->pTaskNext;
   }

   /*
    * Display Software Watchdog task info
    */
   if (bSWDogEnabled)
   {
      pTCB = &TCBSWDog;
      TAL_PRINTF("%-16s  ", pTCB->Name);
      TAL_PRINTF("%4d    -\n", pTCB->nPrio);
   }

   /*
    * Display Statistic task info
    */
   if (bStatEnabled)
   {
      pTCB = &TCBStat;
      TAL_PRINTF("%-16s  ", pTCB->Name);
      TAL_PRINTF("%4d    -\n", pTCB->nPrio);
   }

   /*
    * Display Idle task info
    */
   pTCB = &TCBIdle;
   TAL_PRINTF("%-16s  ", pTCB->Name);
   TAL_PRINTF("%4d    -\n", pTCB->nPrio);

   TAL_PRINTF("\n");
} /* OS_OutputSWDogInfo */

/*************************************************************************/
/*  OS_GetStackInfo                                                      */
/*                                                                       */
/*  Return the info about the stack.                                     */
/*                                                                       */
/*  In    : pTCB, pSize                                                  */
/*  Out   : pSize                                                        */
/*  Return: dFree                                                        */
/*************************************************************************/
uint32_t OS_GetStackInfo (OS_TCB *pTCB, uint32_t *pSize)
{
   uint8_t  *pStart;
   uint8_t  *pEnd;
   uint32_t  dSize;
   uint32_t  dFree;

   pStart = pTCB->pStackStart;
   pEnd   = (uint8_t*)pStart + pTCB->wStackSize;
   dSize  = (uint32_t)pEnd - (uint32_t)pStart;
   dFree  = GetStackFreeCount(pStart, pEnd);

   *pSize = dSize;

   return(dFree);
} /* OS_GetStackInfo */

/*************************************************************************/
/*  OS_StatEnable                                                        */
/*                                                                       */
/*  Enable the statistic functionality.                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_StatEnable (void)
{
#if (_PERF_OPT_NO_STATISTIC >= 1)
   /*
    * Prevent compiler and lint warnings.
    */
   uint32_t dDummy = (uint32_t)StatUpdateTime + (uint32_t)StatTask;
   (void)dDummy;
   (void)StatStack;
   (void)dStatHirResPeriod;
#else
   /* Create the Statistic task */
   OS_TaskCreate(&TCBStat, StatTask, NULL, STAT_TASK_PRIO,
                 StatStack, sizeof(StatStack), "[StatTask]");
#endif
} /* OS_StatEnable */

/*************************************************************************/
/*  OS_StatGetCPULoad                                                    */
/*                                                                       */
/*  Return the CPU load in percent.                                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
uint8_t OS_StatGetCPULoad (void)
{
   return(bStatCPULoad);
} /* OS_StatGetCPULoad */

/*************************************************************************/
/*  OS_StatGetTotalTaskTime                                              */
/*                                                                       */
/*  Return the total task time, of all tasks, from the last second.      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
uint32_t OS_StatGetTotalTaskTime (void)
{
   return(dStatTotalTaskTime);
} /* OS_StatGetTotalTaskTime */

/*************************************************************************/
/*  OS_TaskGetPriority                                                   */
/*                                                                       */
/*  Get the priority of the actual running task.                         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: The task priority                                            */
/*************************************************************************/
int OS_TaskGetPriority (void)
{
   return(RunningTask->nPrio);
} /* OS_TaskGetPriority */

/*************************************************************************/
/*  OS_TaskChangePriority                                                */
/*                                                                       */
/*  Change the priority of the actual running task.                      */
/*                                                                       */
/*  In    : Prio                                                         */
/*  Out   : none                                                         */
/*  Return: The old priority                                             */
/*************************************************************************/
int OS_TaskChangePriority (int nPrio)
{
   int nOldPrio = RunningTask->nPrio;

   RunningTask->nPrio = nPrio;

   return(nOldPrio);
} /* OS_TaskChangePriority */

/*************************************************************************/
/*  OS_TaskYield                                                         */
/*                                                                       */
/*  Give CPU time to the next task from the ReadyList.                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TaskYield (void)
{
   EnterCritical();

   /*
    * Add the actual running task to the ReadyList.
    */
   AddTaskToReadyList(RunningTask);

   /*
    * Schedule the next task
    */
   TaskSchedule();

   ExitCritical();
} /* OS_TaskYield */

/*************************************************************************/
/*  OS_TaskExit                                                          */
/*                                                                       */
/*  Terminate the actual running task.                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TaskExit (void)
{
   EnterCritical();

   /* Remove task from the task list */
   TaskListRemove(RunningTask);

   /*
    * Set state to "not in use".
    *
    *   -- Do not use the macro SET_TASK_STATE here --
    *
    * The macro can be "empty" in case of performance
    * optimisation and will not work.
    */
   RunningTask->State = OS_TASK_STATE_NOT_IN_USE;

   /* Schedule to the next task which is ready */
   TaskScheduleExit();

   ExitCritical();
} /* OS_TaskExit */

/*************************************************************************/
/*  OS_TaskTerminateRequest                                              */
/*                                                                       */
/*  Set the termination flag only.                                       */
/*                                                                       */
/*  In    : pTCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TaskTerminateRequest (OS_TCB *pTCB)
{
   EnterCritical();

   if (pTCB != NULL)
   {
      pTCB->bFlagTermRequest = 1;
   }

   ExitCritical();
} /* OS_TaskTerminateRequest */

/*************************************************************************/
/*  OS_TaskWakeup                                                        */
/*                                                                       */
/*  Wakeup the task be setting the timeout to 0.                         */
/*                                                                       */
/*  In    : pTCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TaskWakeup (OS_TCB *pTCB)
{
   EnterCritical();

   if (pTCB != NULL)
   {
      pTCB->dTimeoutTicks = 0;
   }

   ExitCritical();
} /* OS_TaskWakeup */

/*************************************************************************/
/*  OS_TaskIsSleeping                                                    */
/*                                                                       */
/*  Return the info is the task is sleeping.                             */
/*                                                                       */
/*  In    : pTCB                                                         */
/*  Out   : none                                                         */
/*  Return: 1 = sleeping / 0 = not                                       */
/*************************************************************************/
uint8_t OS_TaskIsSleeping (OS_TCB *pTCB)
{
   uint8_t bSleeping = 0;

   EnterCritical();

   if (pTCB != NULL)
   {
      bSleeping = (pTCB->dTimeoutTicks != 0) ? 1 : 0;
   }

   ExitCritical();

   return(bSleeping);
} /* OS_TaskIsSleeping */

/*************************************************************************/
/*  OS_TaskShouldTerminate                                               */
/*                                                                       */
/*  Return the termination flag of the task.                             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: termination flag                                             */
/*************************************************************************/
uint8_t OS_TaskShouldTerminate (void)
{
   return(RunningTask->bFlagTermRequest);
} /* OS_TaskShouldTerminate */

/*************************************************************************/
/*  OS_TaskGetList                                                       */
/*                                                                       */
/*  Return the first element of the task list.                           */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: first element of the task list                               */
/*************************************************************************/
OS_TCB *OS_TaskGetList (void)
{
   return(pTaskList);
} /* OS_TaskGetList */

/*************************************************************************/
/*  OS_TaskTestStateNotInUsed                                            */
/*                                                                       */
/*  Check if task is in used.                                            */
/*                                                                       */
/*  In    : pTCB                                                         */
/*  Out   : none                                                         */
/*  Return: 0 / 1                                                        */
/*************************************************************************/
int OS_TaskTestStateNotInUsed (OS_TCB *pTCB)
{
   return( (OS_TASK_STATE_NOT_IN_USE == pTCB->State) );
} /* OS_TaskTestStateNotInUsed */

/*************************************************************************/
/*  OS_TaskSetStateNotInUsed                                             */
/*                                                                       */
/*  Change the task state to "not in used".                              */
/*                                                                       */
/*  In    : pTCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TaskSetStateNotInUsed (OS_TCB *pTCB)
{
   if (pTCB != NULL)
   {
      pTCB->State = OS_TASK_STATE_NOT_IN_USE;
   }

} /* OS_TaskSetNotInUsed */

/*************************************************************************/
/*  OS_TimerCallback                                                     */
/*                                                                       */
/*  This function will be called from the Systick interrupt.             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TimerCallback (void)
{

   /*
    * System ticker
    */
   dSystemTick++;

   /*
    * Second ticker
    */
   dMilliSecCnt++;
   if (OS_TICKS_PER_SECOND == dMilliSecCnt)
   {
      dMilliSecCnt = 0;
      dSecTick++;
      dUnixtime++;
   }

   /*
    * Go through the WaitList and decrease the WaitCount.
    */
   HandleWaitList();


   /*
    * Check user timer
    */
   if (UserTimer != NULL)
   {
      if (UserTimerTimeout > 0)
      {
         UserTimerTimeout--;
      }
      else
      {
         UserTimer();
         UserTimer = NULL;
      }
   }

} /* OS_TimerCallback */

/*************************************************************************/
/*  OS_TimerSet                                                          */
/*                                                                       */
/*  Set a user timer and timeout.                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TimerSet (OS_TIMER_FUNC Timer, uint32_t dTimeoutMs)
{
   uint32_t dTicks;

   dTicks = OS_MS_2_TICKS(dTimeoutMs);

   EnterCritical();

   UserTimer        = Timer;
   UserTimerTimeout = dTicks;

   ExitCritical();

} /* OS_TimerSet */

/*************************************************************************/
/*  OS_TimeGet                                                           */
/*                                                                       */
/*  Return the time from the system ticker.                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Systick                                                      */
/*************************************************************************/
uint32_t OS_TimeGet (void)
{
   return(dSystemTick);
} /* OS_TimeGet */

/*************************************************************************/
/*  OS_TimeGetSeconds                                                    */
/*                                                                       */
/*  Return the time from the second ticker.                              */
/*                                                                       */
/*  Out   : none                                                         */
/*  Return: dSecTick                                                     */
/*************************************************************************/
uint32_t OS_TimeGetSeconds (void)
{
   return(dSecTick);
} /* OS_TimeGetSeconds */

/*************************************************************************/
/*  OS_TimeDly                                                           */
/*                                                                       */
/*  Causes the current task to wait for a specified interval or, if the  */
/*  specified interval is zero, to give up the CPU for another task      */
/*  (with higher or same priority).                                      */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : TimeoutMs                                                    */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TimeDly (uint32_t dTimeoutMs)
{
   uint32_t dTicks;

   dTicks = OS_MS_2_TICKS(dTimeoutMs);

   if (0 == dTicks)
   {
      OS_TaskYield();
   }
   else
   {
      EnterCritical();
      RunningTask->dTimeoutTicks = dTicks;
      AddTaskToWaitList(RunningTask);  /* Add the task to the WaitList */
      TaskSchedule();                  /* Schedule the next task */
      ExitCritical();
   }

} /* OS_TimeDly */

/*************************************************************************/
/*  OS_TimeDlyUntil                                                      */
/*                                                                       */
/*  Causes the current task to wait for a specified time.                */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : dNextTime                                                    */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TimeDlyUntil (uint32_t dNextTime)
{
   uint32_t dWaitTime;
   uint32_t dActualTime = OS_TimeGet();

   /* Calculate delta, only if dNextTime is greater than dActualTime */
   if (dNextTime > dActualTime)
   {
      dWaitTime = dNextTime - dActualTime;

      EnterCritical();
      RunningTask->dTimeoutTicks = dWaitTime;
      AddTaskToWaitList(RunningTask);  /* Add the task to the WaitList */
      TaskSchedule();                  /* Schedule the next task */
      ExitCritical();
   }
   else
   {
      OS_TaskYield();
   }

} /* OS_TimeDlyUntil */

/*************************************************************************/
/*  OS_TimeWait                                                          */
/*                                                                       */
/*  Loop for a specified number of milliseconds,                         */
/*  will not release the CPU.                                            */
/*                                                                       */
/*  In    : TimeoutMs                                                    */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TimeWait (uint32_t dTimeoutMs)
{
   uint32_t dTicks;
   uint32_t dTimeEnd;

   dTicks = OS_MS_2_TICKS(dTimeoutMs);

   dTimeEnd = dSystemTick + dTicks;

   while (dSystemTick <= dTimeEnd)
   {
      __asm__ volatile ("nop");
   }

} /* OS_TimeWait */

/*************************************************************************/
/*  OS_UnixtimeGet                                                       */
/*                                                                       */
/*  Return the unixtime.                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Unixtime                                                     */
/*************************************************************************/
uint32_t OS_UnixtimeGet (void)
{
   return(dUnixtime);
} /* OS_UnixtimeGet */

/*************************************************************************/
/*  OS_UnixtimeGetEx                                                     */
/*                                                                       */
/*  Extended version of OS_UnixtimeGet, with possibility to return       */
/*  the milliseconds too.                                                */
/*                                                                       */
/*  In    : pMsec                                                        */
/*  Out   : pMsec                                                        */
/*  Return: Unixtime                                                     */
/*************************************************************************/
uint32_t OS_UnixtimeGetEx (uint16_t *pMsec)
{
   uint32_t dTime;
   uint16_t wMsec;

   EnterCritical();
   dTime = dUnixtime;
   wMsec = (uint16_t)dMilliSecCnt;
   ExitCritical();

   if (pMsec != NULL)
   {
      *pMsec = wMsec;
   }

   return(dTime);
} /* OS_UnixtimeGetEx */

/*************************************************************************/
/*  OS_UnixtimeSet                                                       */
/*                                                                       */
/*  Set a new unixtime.                                                  */
/*                                                                       */
/*  In    : dTime                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_UnixtimeSet (uint32_t dTime)
{
   dUnixtime = dTime;
} /* OS_UnixtimeSet */

/*************************************************************************/
/*  OS_TimezoneIDSet                                                     */
/*                                                                       */
/*  Set the timezone ID.                                                 */
/*                                                                       */
/*  In    : wID                                                          */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TimezoneIDSet (int16_t wID)
{
   wTimezoneID = wID;
} /* OS_TimezoneIDSet */

/*************************************************************************/
/*  OS_TimezoneIDGet                                                     */
/*                                                                       */
/*  Return the timezone ID.                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Timezone                                                     */
/*************************************************************************/
int16_t OS_TimezoneIDGet (void)
{
   return(wTimezoneID);
} /* OS_TimezoneIDGet */

/*************************************************************************/
/*  OS_TimezoneMinSet                                                    */
/*                                                                       */
/*  Set the timezone offset to UTC in minutes.                           */
/*                                                                       */
/*  In    : nOffset                                                      */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TimezoneMinSet (int16_t nOffset)
{
   wTimezoneOffsetMin = nOffset;
   nTimezoneOffsetSec = nOffset * 60;
} /* OS_TimezoneMinSet */

/*************************************************************************/
/*  OS_TimezoneMinGet                                                    */
/*                                                                       */
/*  Return the timezone offset in minutes to UTC.                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Timezone                                                     */
/*************************************************************************/
int16_t OS_TimezoneMinGet (void)
{
   return(wTimezoneOffsetMin);
} /* OS_TimezoneMinGet */

/*************************************************************************/
/*  OS_TimezoneSecGet                                                    */
/*                                                                       */
/*  Return the timezone offset in seconds to UTC.                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Timezone                                                     */
/*************************************************************************/
int32_t OS_TimezoneSecGet (void)
{
   return(nTimezoneOffsetSec);
} /* OS_TimezoneSecGet */

/*************************************************************************/
/*  OS_TimezoneDstSet                                                    */
/*                                                                       */
/*  Set the daylight saving time offset to UTC in hours.                 */
/*                                                                       */
/*  In    : bOffset                                                      */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_TimezoneDstSet (int8_t bOffset)
{
   dTimezoneDstOffsetSec = bOffset * 60 * 60;
} /* OS_TimezoneDstSet */

/*************************************************************************/
/*  OS_TimezoneDstSecGet                                                 */
/*                                                                       */
/*  Return the daylight saving time offset in seconds to UTC.            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Timezone                                                     */
/*************************************************************************/
int32_t OS_TimezoneDstSecGet (void)
{
   return(dTimezoneDstOffsetSec);
} /* OS_TimezoneDstSecGet */

/*************************************************************************/
/*  OS_SWDogStart                                                        */
/*                                                                       */
/*  Create the software watchdog task.                                   */
/*  IF the HW-Dog is enabled with USE_HW_WATCHDOG,                       */
/*  it will be initialised too.                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_SWDogStart (void)
{
#if (_USE_SWWD >= 1)
   static uint8_t bStart = 0;

   if (0 == bStart)
   {
      bStart = 1;

      /* Start the software watchdog task */
      OS_TaskCreate(&TCBSWDog, SWDogTask, NULL, SWDOG_TASK_PRIO,
                 SWDogStack, sizeof(SWDogStack),
                 "[swwd]");
   }
#endif

} /* OS_WatchDogStart */

/*************************************************************************/
/*  OS_SWDogTrigger                                                      */
/*                                                                       */
/*  Trigger the software watch dog.                                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_SWDogTrigger (void)
{
   RunningTask->lSWDogTimeout = RunningTask->lSWDogTimeoutMax;

} /* OS_SWDogTrigger */

/*************************************************************************/
/*  OS_SWDogExpand                                                       */
/*                                                                       */
/*  Expand the SW-Watchdog check time.                                   */
/*  BSP_MAX_SW_WATCHDOG_TIME_MS is expand to dDelay, only for the        */
/*  Delay time. After the period of dDelay, the normal time is activ.    */
/*                                                                       */
/*  In    : Delay in milliseconds                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_SWDogExpand (uint32_t dTimeoutMs)
{
   OS_TCB   *pTCB;
   uint32_t  dTimeoutTicks = OS_MS_2_TICKS(dTimeoutMs);

   /* Get start of list */
   pTCB = OS_TaskGetList();

   EnterCritical();

   while (pTCB != NULL)
   {
      if (pTCB->lSWDogTimeoutMax != 0)
      {
         pTCB->lSWDogTimeout = (int32_t)dTimeoutTicks;
      }

      pTCB = pTCB->pTaskNext;
   }

   ExitCritical();

} /* OS_SWDogExpand */

/*************************************************************************/
/*  OS_SWDogTaskSetTime                                                  */
/*                                                                       */
/*  Set the software watchdog time for the runing task.                  */
/*                                                                       */
/*  In    : Delay in milliseconds                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_SWDogTaskSetTime (uint32_t dTimeoutMs)
{
   uint32_t  dTimeoutTicks = OS_MS_2_TICKS(dTimeoutMs);

   EnterCritical();

   RunningTask->lSWDogTimeout    = (int32_t)dTimeoutTicks;
   RunningTask->lSWDogTimeoutMax = (int32_t)dTimeoutTicks;

   ExitCritical();

} /* OS_SWDogTaskSetTime */

/*************************************************************************/
/*  OS_SemaCreate                                                        */
/*                                                                       */
/*  Create a new semaphore.                                              */
/*                                                                       */
/*  In    : pSema, dCounterStart, dCounterMax                            */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_SemaCreate (OS_SEMA *pSema, int32_t nCounterStart, int32_t nCounterMax)
{
   memset(pSema, 0x00, sizeof(OS_SEMA));

   pSema->nCounter    = nCounterStart;
   pSema->nCounterMax = nCounterMax;
} /* OS_SemaCreate */

#if 1 // TODO
/*************************************************************************/
/*  OS_SemaReset                                                         */
/*                                                                       */
/*  Reset a semaphore. All waiting tasks will be released.               */
/*                                                                       */
/*  In    : pSema, nCounterStart                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_SemaReset (OS_SEMA *pSema, int32_t nCounterStart)
{
   OS_TCB *pTask;

   EnterCritical();

   /* Set new counter */
   pSema->nCounter = nCounterStart;

   /* Check if fifo is not empty  */
   if (0 == IsFifoEmpty(&pSema->Fifo))
   {
      /* Fifo not empty, remove task from the semaphore fifo */
      pTask = TCBFifoRemove(&pSema->Fifo);
      while (pTask != NULL)
      {
         /* Remove semaphore info */
         pTask->pSemaWait = NULL;

         /* Removed the task from the WaitList too. */
         WaitListRemove(pTask);
         pTask->dTimeoutTicks = 0;

         /* Return code for OSSemaWait */
         pTask->nReturnCode = OS_RC_RESETED;

         /* Make the task which is waiting on the semaphore ready to run */
         AddTaskToReadyList(pTask);

         /* Remove the next task if exist */
         if (0 == IsFifoEmpty(&pSema->Fifo))
         {
            /* Fifo not empty, remove task from the semaphore fifo */
            pTask = TCBFifoRemove(&pSema->Fifo);
         }
         else
         {
            break;
         }
      }

      /* Add the task which reset the semaphore to the ReadyList */
      AddTaskToReadyList(RunningTask);

      TaskSchedule(); /* Schedule the next task */
   }

   ExitCritical();
} /* OS_SemaReset */
#endif

/*************************************************************************/
/*  OS_SemaDelete                                                        */
/*                                                                       */
/*  Delete a semaphore. All waiting tasks will be released.              */
/*                                                                       */
/*  In    : pSema                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_SemaDelete (OS_SEMA *pSema)
{
   OS_TCB *pTask;

   EnterCritical();

   /* Check if fifo is not empty  */
   if (0 == IsFifoEmpty(&pSema->Fifo))
   {
      /* Fifo not empty, remove task from the semaphore fifo */
      pTask = TCBFifoRemove(&pSema->Fifo);
      while (pTask != NULL)
      {
         /* Remove semaphore info */
         pTask->pSemaWait = NULL;

         /* Removed task from the WaitList too. */
         WaitListRemove(pTask);
         pTask->dTimeoutTicks = 0;

         /* Return code for OSSemaWait */
         pTask->nReturnCode = OS_RC_DELETED;

         /* Make the task which is waiting on the semaphore ready to run */
         AddTaskToReadyList(pTask);

         /* Remove the next task if exist */
         if (0 == IsFifoEmpty(&pSema->Fifo))
         {
            /* Fifo not empty, remove task from the semaphore fifo */
            pTask = TCBFifoRemove(&pSema->Fifo);
         }
         else
         {
            break;
         }
      }
   }

   ExitCritical();
} /* OS_SemaDelete */

/*************************************************************************/
/*  _OSSemaSignalFromInt                                                 */
/*                                                                       */
/*  Signal a semaphore from interrupt.                                   */
/*                                                                       */
/*  Must be called with interrupts disabled.                             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pSema                                                        */
/*  Out   : none                                                         */
/*  Return: 1 = must schedule / 0 = schedule not needed / error cause    */
/*************************************************************************/
static __inline__ int _OSSemaSignalFromInt (OS_SEMA *pSema)
{
   int      rc = 0;
   OS_TCB *pTask;

   /* Check if fifo is not empty  */
   if (0 == IsFifoEmpty(&pSema->Fifo))
   {
      /* Fifo not empty, remove task from the semaphore fifo */
      pTask = TCBFifoRemove(&pSema->Fifo);

      /* Remove semaphore info */
      pTask->pSemaWait = NULL;

      /* Removed the task from the WaitList too. */
      WaitListRemove(pTask);
      pTask->dTimeoutTicks = 0;

      /* Return code for OSSemaWait */
      pTask->nReturnCode = OS_RC_OK;

      /* Make the task which is waiting on the semaphore ready to run */
      AddTaskToReadyList(pTask);

      rc = 1;
   }
   else
   {
      /* Fifo was empty, check for max value */
      if (pSema->nCounter < pSema->nCounterMax)
      {
         pSema->nCounter++;
      }
      else
      {
         rc = OS_RC_NO_SPACE;
      }
   } /* end if (0 == IsFifoEmpty(&pSema->Fifo)) */

   return(rc);
} /* _OSSemaSignalFromInt */

/*************************************************************************/
/*  OS_SemaSignalFromInt                                                 */
/*                                                                       */
/*  Signal a semaphore from interrupt.                                   */
/*                                                                       */
/*  Must be called with interrupts disabled.                             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pSema                                                        */
/*  Out   : none                                                         */
/*  Return: 1 = must schedule / 0 = schedule not needed / error cause    */
/*************************************************************************/
int OS_SemaSignalFromInt (OS_SEMA *pSema)
{
   return(_OSSemaSignalFromInt(pSema));
} /* OS_SemaSignalFromInt */

/*************************************************************************/
/*  OS_SemaSignal                                                        */
/*                                                                       */
/*  Signal a semaphore.                                                  */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : pSema                                                        */
/*  Out   : none                                                         */
/*  Return: OS_RC_OK / error cause                                       */
/*************************************************************************/
int OS_SemaSignal (OS_SEMA *pSema)
{
   int rc;

   EnterCritical();

   rc = _OSSemaSignalFromInt(pSema);

   /* Check if we must schedule */
   if (1 == rc)
   {
      /* Add the task which signal the semaphore to the ReadyList */
      AddTaskToReadyList(RunningTask);

      TaskSchedule(); /* Schedule the next task */

      rc = OS_RC_OK;
   }

   ExitCritical();

   return(rc);
} /* OS_SemaSignal */

/*************************************************************************/
/*  OS_SemaWait                                                          */
/*                                                                       */
/*  Blocks the task while waiting for the semaphore, with timeout.       */
/*                                                                       */
/*  In case the timeout value (dTimeoutMs) is 0, OSSemaWait will use     */
/*  polling to acquire the resource. If the resource counter (dCounter)  */
/*  is 0, the error cause is OS_RC_TIMEOUT.                              */
/*                                                                       */
/*  With a timeout value (dTimeoutMs) of OS_WAIT_INFINITE the waiting    */
/*  time is not ended until the semaphore is signaled.                   */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : pSema, dTimeoutMs                                            */
/*  Out   : none                                                         */
/*  Return: OS_RC_OK / error casue                                       */
/*************************************************************************/
int OS_SemaWait (OS_SEMA *pSema, uint32_t dTimeoutMs)
{
   int       rc = OS_RC_OK;
   uint32_t dTicks;

   EnterCritical();

   /* Check if semaphore is available */
   if (pSema->nCounter > 0)
   {
      pSema->nCounter--;
   }
   else
   {
      dTicks = ((dTimeoutMs != OS_WAIT_INFINITE) ? OS_MS_2_TICKS(dTimeoutMs) : OS_WAIT_INFINITE);
      if (dTicks != 0)
      {
         /* Not available, prepare the task for waiting */
         RunningTask->dTimeoutTicks = dTicks;
         RunningTask->pSemaWait     = pSema;
         RunningTask->nReturnCode   = OS_RC_ERROR;
         SET_TASK_STATE(RunningTask, OS_TASK_STATE_WAITING);

         /* Add the task to the fifo of the semaphore */
         TCBFifoAddPrio(&pSema->Fifo, RunningTask);

         /*  Add the task to the WaitList too */
         AddTaskToWaitList(RunningTask);

         TaskSchedule(); /* Schedule the next task */

         /* This is the return code of the function */
         rc = RunningTask->nReturnCode;
      }
      else
      {
         /* Polling, no resource available */
         rc = OS_RC_TIMEOUT;
      }
   }

   ExitCritical();

   return(rc);
} /* OS_SemaWait */

/*************************************************************************/
/*  OS_MutexCreate                                                       */
/*                                                                       */
/*  Create a new mutex.                                                  */
/*                                                                       */
/*  In    : pMutex                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_MutexCreate (OS_MUTEX *pMutex)
{
   memset(pMutex, 0x00, sizeof(OS_MUTEX));
} /* OS_MutexCreate */

/*************************************************************************/
/*  _OSMutexSignalFromInt                                                */
/*                                                                       */
/*  Signal a mutex from interrupt.                                       */
/*                                                                       */
/*  Must be called with interrupts disabled.                             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pMutex                                                       */
/*  Out   : none                                                         */
/*  Return: 1 = must schedule / 0 = schedule not needed / error cause    */
/*************************************************************************/
static __inline__ int _OSMutexSignalFromInt (OS_MUTEX *pMutex)
{
   int      rc = 0;
   OS_TCB *pTask;

   /* Check if fifo is not empty  */
   if (0 == IsFifoEmpty(&pMutex->Fifo))
   {
      if (pMutex->nCounter > 0)
      {
         pMutex->nCounter--;
      }

      if (0 == pMutex->nCounter)
      {
         pMutex->Owner = NULL;

         /* Fifo not empty, remove task from the mutex fifo */
         pTask = TCBFifoRemove(&pMutex->Fifo);

         /* Remove semaphore info */
         pTask->pMutexWait = NULL;

         /* Removed the task from the WaitList too. */
         WaitListRemove(pTask);
         pTask->dTimeoutTicks = 0;

         /* Return code for OSSemaWait */
         pTask->nReturnCode = OS_RC_OK;

         /* Make the task which is waiting on the semaphore ready to run */
         AddTaskToReadyList(pTask);
      }

      rc = 1;
   }
   else
   {
      /* Fifo was empty, check for max value */
      if (pMutex->nCounter > 0)
      {
         pMutex->nCounter--;
         if (0 == pMutex->nCounter)
         {
            pMutex->Owner = NULL;
         }
      }
      else
      {
         rc = OS_RC_ERROR;
      }
   } /* end if (0 == IsFifoEmpty(&pSema->Fifo)) */

   return(rc);
} /* _OSMutexSignalFromInt */

#if 0
/*************************************************************************/
/*  OS_MutexSignalFromInt                                                */
/*                                                                       */
/*  Signal a mutex from interrupt.                                       */
/*                                                                       */
/*  Must be called with interrupts disabled.                             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pMutex                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_MutexSignalFromInt (OS_MUTEX *pMutex)
{
   _OSMutexSignalFromInt(pMutex);

} /* OS_MutexSignalFromInt */
#endif

/*************************************************************************/
/*  OS_MutexSignal                                                       */
/*                                                                       */
/*  Signal a mutex.                                                      */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : pSema                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_MutexSignal (OS_MUTEX *pMutex)
{
   int rc;

   EnterCritical();

   rc = _OSMutexSignalFromInt(pMutex);

   /* Check if we must schedule */
   if (1 == rc)
   {
      /* Add the task which signal the mutex to the ReadyList */
      AddTaskToReadyList(RunningTask);

      TaskSchedule(); /* Schedule the next task */

      rc = OS_RC_OK;
   }

   ExitCritical();

} /* OS_MutexSignal */

/*************************************************************************/
/*  OS_MutexWait                                                         */
/*                                                                       */
/*  Blocks the task while waiting for the mutex, with timeout.           */
/*                                                                       */
/*  In case the timeout value (dTimeoutMs) is 0, OSSemaWait will use     */
/*  polling to acquire the resource. If the resource counter (dCounter)  */
/*  is 0, the error cause is OS_RC_TIMEOUT.                              */
/*                                                                       */
/*  With a timeout value (dTimeoutMs) of OS_WAIT_INFINITE the waiting    */
/*  time is not ended until the semaphore is signaled.                   */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : pSema, dTimeoutMs                                            */
/*  Out   : none                                                         */
/*  Return: OS_RC_OK / error casue                                       */
/*************************************************************************/
int OS_MutexWait (OS_MUTEX *pMutex, uint32_t dTimeoutMs)
{
   int       rc = OS_RC_OK;
   uint32_t dTicks;

   EnterCritical();

   /* Check if mutex is available */
   if (NULL == pMutex->Owner)
   {
      pMutex->Owner = RunningTask;
      pMutex->nCounter++;
   }
   else if (pMutex->Owner == RunningTask)
   {
      pMutex->nCounter++;
   }
   else
   {
      dTicks = ((dTimeoutMs != OS_WAIT_INFINITE) ? OS_MS_2_TICKS(dTimeoutMs) : OS_WAIT_INFINITE);
      if (dTicks != 0)
      {
         /* Not available, prepare the task for waiting */
         RunningTask->dTimeoutTicks = dTicks;
         RunningTask->pMutexWait    = pMutex;
         RunningTask->nReturnCode   = OS_RC_ERROR;
         SET_TASK_STATE(RunningTask, OS_TASK_STATE_WAITING);

         /* Add the task to the fifo of the mutex */
         TCBFifoAddPrio(&pMutex->Fifo, RunningTask);

         /*  Add the task to the WaitList too */
         AddTaskToWaitList(RunningTask);

         TaskSchedule(); /* Schedule the next task */

         /* This is the return code of the function */
         rc = RunningTask->nReturnCode;
      }
      else
      {
         /* Polling, no resource available */
         rc = OS_RC_TIMEOUT;
      }
   }

   ExitCritical();

   return(rc);
} /* OS_MutexWait */

/*************************************************************************/
/*  OS_EventCreate                                                       */
/*                                                                       */
/*  Create a new event.                                                  */
/*                                                                       */
/*  In    : pEvent                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_EventCreate (OS_EVENT *pEvent)
{
   memset(pEvent, 0x00, sizeof(OS_EVENT));

   pEvent->dPattern = 0;
} /* OS_EventCreate */

/*************************************************************************/
/*  OS_EventDelete                                                       */
/*                                                                       */
/*  Delete an event.                                                     */
/*                                                                       */
/*  In    : pEvent                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_EventDelete (OS_EVENT *pEvent)
{
   OS_TCB *pTask;

   EnterCritical();

   /* Check if fifo is not empty  */
   if (0 == IsFifoEmpty(&pEvent->Fifo))
   {
      /* Fifo not empty, remove task from the event fifo */
      pTask = TCBFifoRemove(&pEvent->Fifo);
      while (pTask != NULL)
      {
         /* Remove event info */
         pTask->pEventWait = NULL;
         pTask->dEventWaitPattern = 0;

         /* Removed the task from the WaitList too. */
         WaitListRemove(pTask);
         pTask->dTimeoutTicks = 0;

         /* Return code for OSEventWait */
         pTask->nReturnCode = OS_RC_DELETED;

         /* Make the task which is waiting on the event ready to run */
         AddTaskToReadyList(pTask);

         /* Remove the next task if exist */
         if (0 == IsFifoEmpty(&pEvent->Fifo))
         {
            /* Fifo not empty, remove task from the event fifo */
            pTask = TCBFifoRemove(&pEvent->Fifo);
         }
         else
         {
            break;
         }
      }
   }

   ExitCritical();
} /* OS_EventDelete */

/*************************************************************************/
/*  OS_EventSetFromInt                                                   */
/*                                                                       */
/*  Signal an event from interrupt.                                      */
/*                                                                       */
/*  Must be called with interrupts disabled.                             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pEvent, dPattern                                             */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_EventSetFromInt (OS_EVENT *pEvent, uint32_t dPattern)
{
   int      rc = 0;
   int      GoReady;
   OS_TCB *pTask;
   OS_TCB *pPrev;

   pEvent->dPattern |= dPattern;

   /* Check if fifo is NOT empty  */
   if (0 == IsFifoEmpty(&pEvent->Fifo))
   {
      /* Fifo is not empty */
      pTask = pEvent->Fifo.pOut;

      while (pTask != NULL)
      {
         /*
          * Save the prev pointer of the task.
          * pTask->pPrev could not be used later, because it wil be changed
          * by the TCBFifoRemoveMiddle before we can use the Prev for the loop.
          */
         pPrev = pTask->pPrev;

         /*
          * Check if the task must wait or not
          */
         if (OS_EVENT_MODE_OR == pTask->EventWaitMode)
         {
            GoReady = ((pEvent->dPattern & pTask->dEventWaitPattern) != 0) ? 1 : 0;
         }
         else
         {
            GoReady = ((pEvent->dPattern & pTask->dEventWaitPattern) == pTask->dEventWaitPattern) ? 1 : 0;
         }

         if (GoReady)
         {
            pTask->dEventWaitPattern = pEvent->dPattern;
            pTask->nReturnCode       = OS_RC_OK;

            TCBFifoRemoveMiddle(&pEvent->Fifo, pTask);

            /* Remove event info */
            pTask->pEventWait = NULL;

            /* Removed the task from the WaitList too. */
            WaitListRemove(pTask);
            pTask->dTimeoutTicks = 0;

            AddTaskToReadyList(pTask);

            rc++;
         }

         pTask = pPrev;
      }

      if (rc != 0)
      {
         /* Event occurred, the pattern can be cleared */
         pEvent->dPattern = 0;
      }
   }

} /* OS_EventSetFromInt */

/*************************************************************************/
/*  OS_EventSet                                                          */
/*                                                                       */
/*  Signal an event.                                                     */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : pEvent, dPattern                                             */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_EventSet (OS_EVENT *pEvent, uint32_t dPattern)
{
   EnterCritical();

   OS_EventSetFromInt(pEvent, dPattern);

   /* Add the task which signal the event to the ReadyList */
   AddTaskToReadyList(RunningTask);

   TaskSchedule(); /* Schedule the next task */

   ExitCritical();
} /* OS_EventSet */

/*************************************************************************/
/*  OS_EventClr                                                          */
/*                                                                       */
/*  Clear the event pattern.                                             */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : pEvent, dPattern                                             */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_EventClr (OS_EVENT *pEvent, uint32_t dPattern)
{
   pEvent->dPattern &= ~dPattern;

} /* OS_EventClr */

/*************************************************************************/
/*  OS_EventWait                                                         */
/*                                                                       */
/*  Blocks the task while waiting for the event, with timeout.           */
/*                                                                       */
/*  With a timeout value (dTimeoutMs) of OS_WAIT_INFINITE the waiting    */
/*  time is not ended until the event is signaled.                       */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : pEvent, dWaitPattern, Mode, pPattern, dTimeoutMs             */
/*  Out   : none                                                         */
/*  Return: OS_EVENT_RC_OK / error casue                                 */
/*************************************************************************/
int OS_EventWait (OS_EVENT *pEvent, uint32_t dWaitPattern,
                 os_event_mode_t Mode, uint32_t *pPattern, uint32_t dTimeoutMs)
{
   int       rc      = OS_RC_ERROR;
   int       GoReady = 0;
   uint32_t dTicks;

   dTicks = ((dTimeoutMs != OS_WAIT_INFINITE) ? OS_MS_2_TICKS(dTimeoutMs) : OS_WAIT_INFINITE);

   EnterCritical();

   *pPattern = 0;

   /*
    * Check if the task must wait or not
    */
   if (OS_EVENT_MODE_OR == Mode)
   {
      GoReady = ((pEvent->dPattern & dWaitPattern) != 0) ? 1 : 0;
   }
   else
   {
      GoReady = ((pEvent->dPattern & dWaitPattern) == dWaitPattern) ? 1 : 0;
   }

   if (GoReady)
   {
      *pPattern = pEvent->dPattern;
      pEvent->dPattern = 0;

      rc = OS_RC_OK;
   }
   else
   {
      if (dTicks != 0)
      {
         /* Prepare the task for waiting */
         RunningTask->dTimeoutTicks     = dTicks;
         RunningTask->pEventWait        = pEvent;
         RunningTask->dEventWaitPattern = dWaitPattern;
         RunningTask->EventWaitMode     = Mode;
         RunningTask->nReturnCode       = OS_RC_ERROR;
         SET_TASK_STATE(RunningTask, OS_TASK_STATE_WAITING);

         /* Add the task to the fifo of the event */
         TCBFifoAddPrio(&pEvent->Fifo, RunningTask);

         /*  Add the task to the WaitList too */
         AddTaskToWaitList(RunningTask);

         TaskSchedule(); /* Schedule the next task */

         /* This is the return code of the function */
         rc = RunningTask->nReturnCode;
         if (OS_RC_OK == rc)
         {
            *pPattern = RunningTask->dEventWaitPattern;
         }
      }
      else
      {
         /* Polling, no event available */
         rc = OS_RC_TIMEOUT;
      }
   }

   ExitCritical();

   return(rc);
} /* OS_EventWait */

/*************************************************************************/
/*  OS_MboxCreate                                                        */
/*                                                                       */
/*  Create a new mailbox.                                                */
/*                                                                       */
/*  In    : pMbox, pBuffer, nCounterMax                                  */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_MboxCreate (OS_MBOX *pMbox, void **pBuffer, uint16_t wCounterMax)
{
   memset(pMbox, 0x00, sizeof(OS_MBOX));

   OS_SemaCreate(&pMbox->UsedCntSema, 0,           wCounterMax);
   OS_SemaCreate(&pMbox->FreeCntSema, wCounterMax, wCounterMax);

   pMbox->wCountMax = wCounterMax;
   pMbox->wCount    = 0;
   pMbox->wInIndex  = 0;
   pMbox->wOutIndex = 0;
   pMbox->pBuffer   = pBuffer;
} /* OS_MboxCreate */

/*************************************************************************/
/*  OS_MboxDelete                                                        */
/*                                                                       */
/*  Delete a mailbox. All waiting tasks will be released.                */
/*                                                                       */
/*  In    : pMbox                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OS_MboxDelete (OS_MBOX *pMbox)
{
   OS_SemaDelete(&pMbox->UsedCntSema);
   OS_SemaDelete(&pMbox->FreeCntSema);
} /* OS_MboxDelete */

/*************************************************************************/
/*  _OSMboxPostFromInt                                                   */
/*                                                                       */
/*  Tries to post a message to the mailbox.                              */
/*                                                                       */
/*  Must be called with interrupts disabled.                             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pMbox, pMsg                                                  */
/*  Out   : none                                                         */
/*  Return: OS_RC_OK / OS_RC_NO_SPACE                                    */
/*************************************************************************/
static __inline__  int _OSMboxPostFromInt (OS_MBOX *pMbox, void *pMsg)
{
   int       rc = OS_RC_NO_SPACE;
   uint16_t wFreeCnt;

   wFreeCnt = pMbox->wCountMax - pMbox->wCount;
   if (wFreeCnt != 0)
   {
      /*
       * OSSemaWait cannot be used inside an interrupt.
       * Therefore decrement the semaphore counter direct.
       */
      pMbox->FreeCntSema.nCounter--;

      /* Increment counter, one more message available */
      pMbox->wCount++;

      /* Store message */
      pMbox->pBuffer[pMbox->wInIndex] = pMsg;

      /* Switch to next place */
      pMbox->wInIndex++;

      /* Check end of ring buffer */
      if (pMbox->wCountMax == pMbox->wInIndex)
      {
         pMbox->wInIndex = 0;
      }

      /* Signal message available */
      OS_SemaSignalFromInt(&pMbox->UsedCntSema);

      rc = OS_RC_OK;
   }

   return(rc);
} /* _OSMboxPostFromInt */

/*************************************************************************/
/*  OS_MboxPostFromInt                                                   */
/*                                                                       */
/*  Tries to post a message to the mailbox.                              */
/*                                                                       */
/*  Must be called with interrupts disabled.                             */
/*                                                                       */
/*  Note: Interrupts are disabled and must be disabled.                  */
/*                                                                       */
/*  In    : pMbox, pMsg                                                  */
/*  Out   : none                                                         */
/*  Return: OS_RC_OK / OS_RC_NO_SPACE                                    */
/*************************************************************************/
int OS_MboxPostFromInt (OS_MBOX *pMbox, void *pMsg)
{
   return(_OSMboxPostFromInt(pMbox, pMsg));
} /* OS_MboxPostFromInt */

/*************************************************************************/
/*  OS_MboxPost                                                          */
/*                                                                       */
/*  Tries to post a message to the mailbox.                              */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : pMbox, pMsg                                                  */
/*  Out   : none                                                         */
/*  Return: OS_RC_OK / OS_RC_NO_SPACE                                    */
/*************************************************************************/
int OS_MboxPost (OS_MBOX *pMbox, void *pMsg)
{
   int rc;

   EnterCritical();

   rc = _OSMboxPostFromInt(pMbox, pMsg);

   ExitCritical();

   return(rc);
} /* OS_MboxPost */

/*************************************************************************/
/*  OS_MboxWait                                                          */
/*                                                                       */
/*  Blocks the task while waiting for the mailbox, with timeout.         */
/*                                                                       */
/*  In case the timeout value (dTimeoutMs) is 0, OSMboxWait will use     */
/*  polling to acquire the message. If the resource counter (dCounter)   */
/*  is 0, the error cause is OS_RC_TIMEOUT.                              */
/*                                                                       */
/*  With a timeout value (dTimeoutMs) of OS_WAIT_INFINITE the waiting    */
/*  time is not ended until the mailbox is signaled.                     */
/*                                                                       */
/*  Note: Must not used from inside an interrupt.                        */
/*                                                                       */
/*  In    : pMbox, pMsg, dTimeoutMs                                      */
/*  Out   : none                                                         */
/*  Return: OS_RC_OK / OS_RC_TIMEOUT                                     */
/*************************************************************************/
int OS_MboxWait (OS_MBOX *pMbox, void **pMsg, uint32_t dTimeoutMs)
{
   int rc;

   if (0 == dTimeoutMs)
   {
      if (0 == pMbox->wCount)
      {
         return(OS_RC_TIMEOUT);
      }
   }

   rc = OS_SemaWait(&pMbox->UsedCntSema, dTimeoutMs);
   if (OS_RC_OK == rc)
   {
      EnterCritical();

      /* Decrement counter, one message will be removed */
      pMbox->wCount--;

      /* Get message */
      *pMsg = pMbox->pBuffer[pMbox->wOutIndex];

      /* Switch to next place */
      pMbox->wOutIndex++;

      /* Check end of ring buffer */
      if (pMbox->wCountMax == pMbox->wOutIndex)
      {
         pMbox->wOutIndex = 0;
      }

      OS_SemaSignalFromInt(&pMbox->FreeCntSema);

      ExitCritical();
   }

   return(rc);
} /* OS_MboxWait*/

#endif /* defined(RTOS_TCTS) */

/*** EOF ***/
