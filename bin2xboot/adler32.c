/**************************************************************************
*  Copyright (c) 2013 by Michael Fischer.
*  All rights reserved.
*
*  Source based on an example from Jean-loup Gailly and Mark Adler.
*  Therefore partial copyright:
*  Copyright (C) 1995-2017 Jean-loup Gailly and Mark Adler
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*  
*  1. Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the distribution.
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
*  26.12.2013  mifi  First Version
*  18.01.2015  mifi  Rework for use with Windows too.
*  07.01.2016  mifi  Use adler32 from zlib v1.2.10.
**************************************************************************/

/* zlib.h -- interface of the 'zlib' general purpose compression library
  version 1.2.10, January 2nd, 2017

  Copyright (C) 1995-2017 Jean-loup Gailly and Mark Adler

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  Jean-loup Gailly        Mark Adler
  jloup@gzip.org          madler@alumni.caltech.edu


  The data format used by the zlib library is described by RFCs (Request for
  Comments) 1950 to 1952 in the files http://tools.ietf.org/html/rfc1950
  (zlib format), rfc1951 (deflate format) and rfc1952 (gzip format).
*/


/* adler32.c -- compute the Adler-32 checksum of a data stream
 * Copyright (C) 1995-2011, 2016 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 */
 
#define __ADLER32_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#ifdef _MSC_VER
#include <windows.h>
#else
#include <stdint.h>
#endif
#include <stdio.h>
#include "adler32.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define BASE 65521UL          /* largest prime smaller than 65536      */
#define NMAX 5552             /* NMAX is the largest n such that       */
                              /* 255n(n+1)/2 + (n+1)(BASE-1) <= 2^32-1 */

#define DO1(buf,i)   {adler += (buf)[i]; sum2 += adler;}
#define DO2(buf,i)   DO1(buf,i); DO1(buf,i+1);
#define DO4(buf,i)   DO2(buf,i); DO2(buf,i+2);
#define DO8(buf,i)   DO4(buf,i); DO4(buf,i+4);
#define DO16(buf)    DO8(buf,0); DO8(buf,8);  


/* 
 * use NO_DIVIDE if your processor does not do division in hardware --
 * try it both ways to see which is faster 
 */
#ifdef NO_DIVIDE
/* 
 * note that this assumes BASE is 65521, where 65536 % 65521 == 15
 * (thank you to John Reiser for pointing this out) 
 */
#  define CHOP(a) \
    do { \
        unsigned long tmp = a >> 16; \
        a &= 0xffffUL; \
        a += (tmp << 4) - tmp; \
    } while (0)
#  define MOD28(a) \
    do { \
        CHOP(a); \
        if (a >= BASE) a -= BASE; \
    } while (0)
#  define MOD(a) \
    do { \
        CHOP(a); \
        MOD28(a); \
    } while (0)
#  define MOD63(a) \
    do { /* this assumes a is not negative */ \
        z_off64_t tmp = a >> 32; \
        a &= 0xffffffffL; \
        a += (tmp << 8) - (tmp << 5) + tmp; \
        tmp = a >> 16; \
        a &= 0xffffL; \
        a += (tmp << 4) - tmp; \
        tmp = a >> 16; \
        a &= 0xffffL; \
        a += (tmp << 4) - tmp; \
        if (a >= BASE) a -= BASE; \
    } while (0)
#else
#  define MOD(a) a %= BASE
#  define MOD28(a) a %= BASE
#  define MOD63(a) a %= BASE
#endif

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

uint32_t adler32 (uint32_t adler, const uint8_t *buf, uint32_t len)
{
   unsigned long sum2;
   unsigned n;

   /* split Adler-32 into component sums */
   sum2 = (adler >> 16) & 0xffff;
   adler &= 0xffff;

   /* in case user likes doing a byte at a time, keep it fast */
   if (len == 1) {
       adler += buf[0];
       if (adler >= BASE)
           adler -= BASE;
       sum2 += adler;
       if (sum2 >= BASE)
           sum2 -= BASE;
       return adler | (sum2 << 16);
   }

   /* initial Adler-32 value (deferred check for len == 1 speed) */
   if (buf == NULL)
       return 1L;

   /* in case short lengths are provided, keep it somewhat fast */
   if (len < 16) {
       while (len--) {
           adler += *buf++;
           sum2 += adler;
       }
       if (adler >= BASE)
           adler -= BASE;
       MOD28(sum2);            /* only added so many BASE's */
       return adler | (sum2 << 16);
   }

   /* do length NMAX blocks -- requires just one modulo operation */
   while (len >= NMAX) {
       len -= NMAX;
       n = NMAX / 16;          /* NMAX is divisible by 16 */
       do {
           DO16(buf);          /* 16 sums unrolled */
           buf += 16;
       } while (--n);
       MOD(adler);
       MOD(sum2);
   }

   /* do remaining bytes (less than NMAX, still just one modulo) */
   if (len) {                  /* avoid modulos if none remaining */
       while (len >= 16) {
           len -= 16;
           DO16(buf);
           buf += 16;
       }
       while (len--) {
           adler += *buf++;
           sum2 += adler;
       }
       MOD(adler);
       MOD(sum2);
   }

   /* return recombined sums */
   return adler | (sum2 << 16);
}

/*** EOF ***/

