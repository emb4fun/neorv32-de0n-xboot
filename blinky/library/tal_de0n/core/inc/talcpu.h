/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2013-2023 by Michael Fischer (www.emb4fun.de).
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
#if !defined(__TALCPU_H__)
#define __TALCPU_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include "talos.h"
#include "talcpu_specific.h"

/*
 * Check for ARM7 style CPU
 */
#if defined(__ARM_ARCH_4T__) || defined(__ARM_ARCH_7A__)
#include "talcpu_arm.h"
#endif

/*
 * Check for Cortex-M and Cortex-M0 style CPU
 */
#if defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_6M__)
#include "talcpu_cm.h"
#endif

/*
 * Check for RISC-V CPU
 */
#if defined(__ARCH_RISCV__)
#include "talcpu_riscv.h"
#endif

/**************************************************************************
*  Global Definitions
**************************************************************************/

#if !defined(soc_cv_av)
typedef void (*tal_irq_fnt)(void);
#else
typedef void (*tal_irq_fnt)(uint32_t IRQ, void *Context);
#endif

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

void       tal_CPUInit (void);
void       tal_CPUSysTickStart (void);

void       tal_CPUStatDWTInit (void);
uint32_t   tal_CPUStatDWTGetCnt (void);

void       tal_CPUIrqEnable (int IRQ);
void       tal_CPUIrqDisable (int IRQ);
void       tal_CPUIrqDisableAll (void);
void       tal_CPUIrqSetPriority (int IRQ, int Priority);

#if !defined(soc_cv_av)
void       tal_CPUIrqSetFunction (int IRQ, tal_irq_fnt Function);
#else
void       tal_CPUIrqSetFunction (int IRQ, tal_irq_fnt Function, void *Context);
#endif

uint32_t   tal_CPUStatGetHiResPeriod (void);
uint32_t   tal_CPUStatGetHiResCnt (void);

uint32_t   tal_CPUGetFrequencyCPU (void);

void       tal_CPURngInit (void);
void       tal_CPURngDeInit (void);
TAL_RESULT tal_CPURngHardwarePoll (uint8_t *pData, uint32_t dSize);

void       tal_CPUReboot (void);

#define TAL_CPU_WD_TIME_MS    500 
void       tal_CPUInitHWDog (void);
void       tal_CPUTriggerHWDog (void);

#endif /* !__TALCPU_H__ */

/*** EOF ***/
