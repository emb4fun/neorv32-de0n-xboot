/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
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
#if !defined(__TALBOARD_H__) && defined(USE_BOARD_DE0N)
#define __TALBOARD_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>
#include <time.h>
#include "project.h"
#include "taltypes.h"

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * Defines for board and cpu name
 */
#define TAL_BOARD "DE0-Nano"
#define TAL_CPU   "NEORV32"


/*
 * The DE0-Nano support 8 LEDs
 */
typedef enum _tal_led_channel_
{
   TAL_LED_CHANNEL_1 = 0,
   TAL_LED_CHANNEL_2,
   TAL_LED_CHANNEL_3,
   TAL_LED_CHANNEL_4,
   TAL_LED_CHANNEL_5,
   TAL_LED_CHANNEL_6,
   TAL_LED_CHANNEL_7,
   TAL_LED_CHANNEL_8,

   /* TAL_LED_CHANNEL_MAX must be the last one */
   TAL_LED_CHANNEL_MAX
} TAL_LED_CHANNEL;


/*
 * COM port used for the terminal
 */
#if !defined(TERM_COM_PORT)
#define TERM_COM_PORT   TAL_COM_PORT_1
#endif


/*
 * Configure external memory for the HEAP
 */
#if defined(__TCM__)
#define TAL_HEAP_MEM1_START   __SDRAM_segment_start__
#define TAL_HEAP_MEM1_END     __SDRAM_segment_end__
#endif

#if defined(__XIP__)
#define TAL_HEAP_MEM1_START   __SDRAM_segment_used_end__
#define TAL_HEAP_MEM1_END     __SDRAM_segment_end__
#endif

#if defined(__XIP_TO_TCM__)
#define TAL_HEAP_MEM1_START   __SDRAM_segment_start__
#define TAL_HEAP_MEM1_END     __SDRAM_segment_end__
#endif

#if defined(__XIP_TO_SDRAM__)
#define TAL_HEAP_MEM1_START   __SDRAM_segment_used_end__
#define TAL_HEAP_MEM1_END     __SDRAM_segment_end__
#endif

#if defined(__SDRAM__)
#define TAL_HEAP_MEM1_START   __SDRAM_segment_used_end__
#define TAL_HEAP_MEM1_END     __SDRAM_segment_end__
#endif

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

TAL_RESULT tal_BoardEnableCOM1 (void);

#endif /* !__TALBOARD_H__ */

/*** EOF ***/
