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
#if !defined(__PROJECT_H__)
#define __PROJECT_H__

/**************************************************************************
*  Includes
**************************************************************************/

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * Project name and version info
 */
#define PROJECT_NAME          "Tiny CTS/AL Blinky"

#define PROJECT_VER_MAJOR     0
#define PROJECT_VER_MINOR_1   0
#define PROJECT_VER_MINOR_2   1

#define PROJECT_VER_NUMBER    ((PROJECT_VER_MAJOR * 100) + (PROJECT_VER_MINOR_1 * 10) + PROJECT_VER_MINOR_2)

#define PROJECT_VER_STRING    (XSTR(PROJECT_VER_MAJOR) "." \
                               XSTR(PROJECT_VER_MINOR_1)   \
                               XSTR(PROJECT_VER_MINOR_2))


/*-----------------------------------------------------------------------*/
/*    Task priorities and delays                                         */
/*-----------------------------------------------------------------------*/

/*
 * Define priority and stack size of the tasks.
 * In case of CTS, valid priorities are in the range
 * from HIGHPRIO (1) ... (OS_PRIO_MAX) LOWPRIO.
 */

#define TASK_START_PRIORITY         1              /* This is the start priority */
#define TASK_START_PRIORITY_IDLE    OS_PRIO_MAX    /* The START thread is not needed later */
#define TASK_START_STK_SIZE         1024

#define TASK_TERM_PRIORITY          64
#define TASK_TERM_STK_SIZE          1024

#define TASK_LED_PRIORITY           64
#define TASK_LED_STK_SIZE           512

#define TASK_A_PRIORITY             65
#define TASK_A_STK_SIZE             512

#define TASK_B_PRIORITY             65
#define TASK_B_STK_SIZE             512

#define TASK_C_PRIORITY             65
#define TASK_C_STK_SIZE             512

/**************************************************************/
/*
 * Changes for TCTS
 */
#define OS_STAT_STACK_SIZE          768
/**************************************************************/



/*
 * Define delay times of the tasks
 */
#define TASK_START_DELAY_MS         1000
#define TASK_TERM_DELAY_MS          100
#define TASK_LED_DELAY_MS           500

/*-----------------------------------------------------------------------*/
/* Terminal stuff                                                        */
/*-----------------------------------------------------------------------*/

/*
 * TERM_COM_PORT is defined in talboard.h
 */
#define TERM_COM_SPEED              115200


/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

#endif /* !__PROJECT_H__ */

/*** EOF ***/
