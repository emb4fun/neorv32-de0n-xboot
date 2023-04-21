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
*  04.01.2016  mifi  Added more timezone functionality.
*  10.01.2016  mifi  Added OSTaskGetPriority and OSTimeDlyUntil.
*                    Added event functionality.
*  04.02.2016  mifi  Remove OSSemaBroadcast and OSSemaBroadcastAsync.
*                    Change OSSemaCreate parameter from uint32 to int32.
*  12.02.2016  mifi  Added the some more functionality:
*                    - OSTaskTerminateRequest and OSTaskShouldTerminate
*                    - OSTimerSet
*                    - OSSemaReset
*  21.03.2016  mifi  Added OSSemaBroadcast and OSSemaBroadcastAsync again.
*  05.05.2016  mifi  Added incomplete Mailbox functionality, only:
*                    - OSMboxCreate
*                    - OSMboxDelete
*                    - OSMboxPostFromInt
*                    - OSMboxWait
*  16.06.2016  mifi  Added Cortex-M0+ support.
*  10.03.2018  mifi  Change version to v0.16.
*  12.05.2018  mifi  Added TaskScheduleExit support.
*                    Change version to v0.17.
*  16.05.2018  mifi  Check 8 byte stack alignment.
*                    Change version to v0.18.
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
#if !defined(__TCTS_H__)
#define __TCTS_H__

#if defined(RTOS_TCTS)

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>
#include "tcts_conf.h"

/**************************************************************************
*  All Structures and Common Constants
**************************************************************************/

#define OS_Init   OS_TCTS_Init

/*
 * Default ticks per second
 */
#if !defined(OS_TICKS_PER_SECOND)
#define OS_TICKS_PER_SECOND   1000
#endif

/*
 * OS version information
 */
#define OS_VER_MAJOR          0
#define OS_VER_MINOR          22
#define OS_VER_PATCH          0

#define OS_VER_NUMBER         ((OS_VER_MAJOR << 24) | (OS_VER_MINOR << 16) | (OS_VER_PATCH << 8))

#define OS_VER_STRING         (XSTR(OS_VER_MAJOR) "." \
                               XSTR(OS_VER_MINOR) "." \
                               XSTR(OS_VER_PATCH))

#define OS_NAME               "TinyCTS"


/*
 * Define for infinite timeout
 */
#define OS_WAIT_INFINITE   ((uint32_t)-1)


/*
 * Define highest priority value for a user task,
 * which is the lowest priority.
 */
#define OS_PRIO_MAX        253


typedef void (*OS_TASK)(void *arg);

typedef enum
{
   OS_TASK_STATE_NOT_IN_USE = 0,
   OS_TASK_STATE_CREATED,
   OS_TASK_STATE_READY,
   OS_TASK_STATE_RUNNING,
   OS_TASK_STATE_WAITING
} os_task_state_t;


/*
 * Forward references
 */
typedef struct _os_tcb_fifo_  os_tcb_fifo_t;
typedef struct _os_tcb_       OS_TCB;
typedef struct _os_sema_      OS_SEMA;
typedef struct _os_mutex_     OS_MUTEX;
typedef struct _os_event_     OS_EVENT;
typedef struct _os_mbox_      OS_MBOX;


/*
 * Used to build a FIFO
 */
struct _os_tcb_fifo_
{
   OS_TCB *pIn;
   OS_TCB *pOut;
};


/*
 * Return code of OS functions
 */
#define OS_RC_OK       0
#define OS_RC_ERROR    -1
#define OS_RC_TIMEOUT  -2
#define OS_RC_NO_SPACE -3
#define OS_RC_DELETED  -4
#define OS_RC_RESETED  -5

typedef int            OS_RESULT;


/*
 * Semaphore
 */
#define OS_SEMA_COUNTER_MAX   INT32_MAX
struct _os_sema_
{
   os_tcb_fifo_t  Fifo;       /* Used for the tasks which are waiting for the semaphore */
   int32_t       nCounter;
   int32_t       nCounterMax;
};


/*
 * Mutex
 */
struct _os_mutex_
{
   os_tcb_fifo_t  Fifo;       /* Used for the tasks which are waiting for the mutex */
   OS_TCB        *Owner;
   int32_t       nCounter;
};


/*
 * Event
 */
typedef enum
{
   OS_EVENT_MODE_UNKNOWN = 0,
   OS_EVENT_MODE_OR,
   OS_EVENT_MODE_AND
} os_event_mode_t;

struct _os_event_
{
   os_tcb_fifo_t  Fifo;       /* Used for the tasks which are waiting for the event */
   uint32_t      dPattern;
};


/*
 * Mailbox
 */
struct _os_mbox_
{
   OS_SEMA      UsedCntSema;
   OS_SEMA      FreeCntSema;
   uint16_t    wCountMax;
   uint16_t    wCount;
   uint16_t    wInIndex;
   uint16_t    wOutIndex;
   void      **pBuffer;
};


/*
 * Task Control Block
 */
struct _os_tcb_
{
   OS_TCB          *pNext;                /* Used for the different lists */
   OS_TCB          *pPrev;                /* Used for the different lists */

   OS_TCB          *pWaitNext;            /* Used for the WaitList */
   OS_TCB          *pWaitPrev;            /* Used for the WaitList */

   OS_TCB          *pTaskNext;            /* Used for the TaskList */
   OS_TCB          *pTaskPrev;            /* Used for the TaskList */

   char              Name[17];            /* Name */
   int              nPrio;                /* Priority */
   os_task_state_t   State;               /* State, for debug purpose */
   uintptr_t         StackPtr;            /* Actual stack pointer */
   uint8_t         *pStackStart;          /* Start of the stack */
   uint16_t         wStackSize;           /* Stack size */

   uint32_t         dTimeoutTicks;        /* Wait ticks in case of a timeout */

   OS_SEMA         *pSemaWait;            /* Pointer to the semaphore if the task is waiting for it */

   OS_MUTEX        *pMutexWait;           /* Pointer to the mutex if the task is waiting for it */

   OS_EVENT        *pEventWait;           /* Pointer to the event if the task is waiting for it */
   uint32_t         dEventWaitPattern;    /* Event pattern the task is waiting for */
   os_event_mode_t   EventWaitMode;       /* Event wait mode, OR or AND */

   int              nReturnCode;          /* Return value from OSSemaWait, OSEventWait */

   uint32_t         dStatStartTime;       /* Statistic start time */
   uint32_t         dStatEndTime;         /* Statistic end time */
   uint32_t         dStatTotalTime;       /* Statistic total task time */
   uint32_t         dStatTotalLastTime;   /* Statistic total task time of the previous statistic cycle */
   uint32_t         dStatUsage;           /* Statistic in percent * 100, 0 - 10000 */

   uint8_t          bFlagTermRequest;     /* Termination request flag */

   int32_t          lSWDogTimeout;        /* Software Watchdog timeout */
   int32_t          lSWDogTimeoutMax;     /* Software Watchdog timeout Max */
};


typedef void (*OS_TIMER_FUNC)(void);

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/*
 * This is the macro for defining the stack, use it like:
 *
 * static OS_TASK(MyStack, 512);
 *
 * The stack will be 8 byte aligned and the size 4 byte.
 */
#if !defined(__ARCH_RISCV__)
#define OS_STACK(_n,_s)       uint8_t _n[((_s+3)& ~3)] __attribute__((aligned(0x8)))
#else
/*
 * In case of RISC-V, we need an alignment of 16 bytes and size too.
 */
#define OS_STACK(_n,_s)       uint8_t _n[((_s+15)& ~15)] __attribute__((aligned(0x10)))
#endif


/*
 * Some time management macros for converting MS => Ticks and Ticks => MS
 */
#if (OS_TICKS_PER_SECOND != 1000)
#define OS_MS_2_TICKS(_a)     ((_a * OS_TICKS_PER_SECOND) / 1000)
#define OS_TICKS_2_MS(_a)     ((1000 / OS_TICKS_PER_SECOND) * _a)
#else
#define OS_MS_2_TICKS(_a)     (_a)
#define OS_TICKS_2_MS(_a)     (_a)
#endif


/*
 * Macro to check timeout
 *
 *    a = actual time
 *    s = start time
 *    t = timeout
 */
#define OS_TEST_TIMEOUT(_a,_s,_t)   (((uint32_t)(_a) - (uint32_t)(_s)) >= (uint32_t)(_t))


/*
 * Some resource macros
 */
#define OS_RES_CREATE(_a)        OS_SemaCreate(_a, 1, 1)
#define OS_RES_LOCK(_a)          OS_SemaWait(_a, OS_WAIT_INFINITE)
#define OS_RES_LOCK_TIMED(_a,_b) OS_SemaWait(_a, _b)
#define OS_RES_FREE(_a)          OS_SemaSignal(_a)


/**************************************************************************
*  Functions Definitions
**************************************************************************/

/*
 * General functionality
 */
void      OS_TCTS_Init (void);
void      OS_Start (void);
void      OS_SysTickStart (void);

void      OS_OutputRuntimeStackInfo (void);
void      OS_OutputTaskInfo (void);
void      OS_OutputSWDogInfo (void);
uint32_t  OS_GetStackInfo (OS_TCB *pTCB, uint32_t *pSize);


/*
 * Statistic functionality
 */
void      OS_StatEnable (void);
uint8_t   OS_StatGetCPULoad (void);
uint32_t  OS_StatGetTotalTaskTime (void);


/*
 * Task functionality
 */
void      OS_TaskCreate (OS_TCB *pTCB, OS_TASK Task, void *pParam, int nPrio,
                         uint8_t *pStack, uint16_t wStackSize, char *pName);
int       OS_TaskGetPriority (void);
int       OS_TaskChangePriority (int nPrio);
void      OS_TaskYield (void);
void      OS_TaskExit (void);
void      OS_TaskTerminateRequest (OS_TCB *pTCB);
void      OS_TaskWakeup (OS_TCB *pTCB);
uint8_t   OS_TaskIsSleeping (OS_TCB *pTCB);
uint8_t   OS_TaskShouldTerminate (void);
OS_TCB   *OS_TaskGetList (void);

int       OS_TaskTestStateNotInUsed (OS_TCB *pTCB);
void      OS_TaskSetStateNotInUsed (OS_TCB *pTCB);


/*
 * Time functionality
 */
void      OS_TimerCallback (void);
void      OS_TimerSet (OS_TIMER_FUNC Timer, uint32_t dTimeoutMs);

uint32_t  OS_TimeGet (void);
uint32_t  OS_TimeGetSeconds (void);
void      OS_TimeDly (uint32_t dTimeoutMs);
void      OS_TimeDlyUntil (uint32_t dTicks);
void      OS_TimeWait (uint32_t dTimeoutMs);
uint32_t  OS_UnixtimeGet (void);
uint32_t  OS_UnixtimeGetEx (uint16_t *pMsec);
void      OS_UnixtimeSet (uint32_t dTime);
void      OS_TimezoneIDSet (int16_t wID);
int16_t   OS_TimezoneIDGet (void);
void      OS_TimezoneMinSet (int16_t wOffset);
int16_t   OS_TimezoneMinGet (void);
int32_t   OS_TimezoneSecGet (void);
void      OS_TimezoneDstSet (int8_t bOffset);
int32_t   OS_TimezoneDstSecGet (void);


/*
 * Software Watchdog
 */
void      OS_SWDogStart (void);
void      OS_SWDogTrigger (void);
void      OS_SWDogExpand (uint32_t dTimeoutMs);
void      OS_SWDogTaskSetTime (uint32_t dTimeoutMs);


/*
 * Semaphore functionality
 */
void      OS_SemaCreate (OS_SEMA *pSema, int32_t nCounterStart, int32_t nCounterMax);
void      OS_SemaReset (OS_SEMA *pSema, int32_t nCounterStart);
void      OS_SemaDelete (OS_SEMA *pSema);
int       OS_SemaSignal (OS_SEMA *pSema);
int       OS_SemaSignalFromInt (OS_SEMA *pSema);
int       OS_SemaWait (OS_SEMA *pSema, uint32_t dTimeoutMs);


/*
 * Mutex functionality
 */
void      OS_MutexCreate (OS_MUTEX *pMutex);
void      OS_MutexSignal (OS_MUTEX *pMutex);
int       OS_MutexWait (OS_MUTEX *pMutex, uint32_t dTimeoutMs);


/*
 * Event functionality
 */
void      OS_EventCreate (OS_EVENT *pEvent);
void      OS_EventDelete (OS_EVENT *pEvent);
void      OS_EventSet (OS_EVENT *pEvent, uint32_t dPattern);
void      OS_EventSetFromInt (OS_EVENT *pEvent, uint32_t dPattern);
int       OS_EventWait (OS_EVENT *pEvent, uint32_t dWaitPattern,
                        os_event_mode_t Mode, uint32_t *pPattern, uint32_t dTimeoutMs);
void      OS_EventClr (OS_EVENT *pEvent, uint32_t dPattern);


/*
 * Mailbox functionality
 */
void      OS_MboxCreate (OS_MBOX *pMbox, void **pBuffer, uint16_t wCounterMax);
void      OS_MboxDelete (OS_MBOX *pMbox);
int       OS_MboxPost (OS_MBOX *pMbox, void *pMsg);
int       OS_MboxPostFromInt (OS_MBOX *pMbox, void *pMsg);
int       OS_MboxWait (OS_MBOX *pMbox, void **pMsg, uint32_t dTimeoutMs);

#endif /* defined(RTOS_TCTS) */

#endif /* !__TCTS_H__ */

/*** EOF ***/
