/**************************************************************************
*  Copyright (c) 2017-2023 by Michael Fischer (www.emb4fun.de).
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
*  21.08.2017  mifi  First Version.
**************************************************************************/
#if !defined(__PROJECT_H__)
#define __PROJECT_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>
#include <stdarg.h>
#include "system_neorv32.h"

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * Project name and version info
 */
#define PROJECT_NAME          "XModem Bootloader"
#define PROJECT_BOARD         "DE0-Nano"

#define PROJECT_VER_MAJOR     1
#define PROJECT_VER_MINOR_1   0
#define PROJECT_VER_MINOR_2   0

#define PROJECT_VER_NUMBER    ((PROJECT_VER_MAJOR * 100) + (PROJECT_VER_MINOR_1 * 10) + PROJECT_VER_MINOR_2)

#define PROJECT_VER_STRING    (XSTR(PROJECT_VER_MAJOR) "." \
                               XSTR(PROJECT_VER_MINOR_1)   \
                               XSTR(PROJECT_VER_MINOR_2))


/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

#endif /* !__PROJECT_H__ */

/*** EOF ***/
