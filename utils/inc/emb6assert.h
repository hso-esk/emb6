/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __EMB6ASSERT_H__
#define __EMB6ASSERT_H__


/*==============================================================================
 MACROS
 ==============================================================================*/
#ifndef EMB6_ASSERT_CFG
#define EMB6_ASSERT_CFG                     1
#endif /* #ifndef EMB6_ASSERT_DEBUG_PRINT_V_CFG */

#ifndef EMB6_ASSERT_DEBUG_PRINT_CFG
#define EMB6_ASSERT_DEBUG_PRINT_CFG         1
#endif /* #ifndef EMB6_ASSERT_DEBUG_PRINT_V_CFG */

#ifndef EMB6_ASSERT_DEBUG_PRINT_V_CFG
#define EMB6_ASSERT_DEBUG_PRINT_V_CFG       0
#endif /* #ifndef EMB6_ASSERT_DEBUG_PRINT_V_CFG */

#ifndef EMB6_ASSERT_FILE_NAME
#define EMB6_ASSERT_FILE_NAME    __FILE__
#endif /* EMB6_ASSERT_FILE_NAME */

#ifndef EMB6_ASSERT_DBG_PRINTF
#if (defined(EMB6_ASSERT_DEBUG_PRINT_CFG) || defined(EMB6_ASSERT_DEBUG_PRINT_V_CFG) ) && defined(EMB6_ASSERT_CFG)
#include <stdio.h>
//! Use regular printf function for debugging output
#define EMB6_ASSERT_DBG_PRINTF(format, ...)  printf(format, ##__VA_ARGS__)
#else
#define EMB6_ASSERT_DBG_PRINTF(format, ...) (void)0
#endif /* #if defined(EMB6_ASSERT_DEBUG) && defined(EMB6_ASSERT) */
#endif /* #ifndef EMB6_ASSERT_DBG_PRINTF */

#ifndef EMB6_ASSERT_DBG_PRINTF_V
#if defined(EMB6_ASSERT_DEBUG_PRINT_V_CFG) && defined(EMB6_ASSERT_CFG)
#include <stdio.h>
//! Verbose debugging
#define EMB6_ASSERT_DBG_PRINTF_V(format, ...) (void)0
#else
#define EMB6_ASSERT_DBG_PRINTF_V(format, ...) (void)0
#endif /* #if defined(EMB6_ASSERT_DEBUG) && defined(EMB6_ASSERT) */
#endif /* #ifndef EMB6_ASSERT_DBG_PRINTF */

static inline int __emb6_assert_func(int r, const char* c, const char* f, int l)
{
#if defined(EMB6_ASSERT_CFG)
  if (!r)
  {
	  EMB6_ASSERT_DBG_PRINTF("\n %s, %d: assertion \"%s\" failed", f, l, c);
  }
#endif /* #if defined(EMB6_ASSERT_CFG) */
  return !r;
}

#if defined(EMB6_ASSERT_CFG)
#ifndef EMB6_ASSERT
#define EMB6_ASSERT(x) (__emb6_assert_func((int)(x), #x, EMB6_ASSERT_FILE_NAME, __LINE__))
#endif /* EMB6_ASSERT */

#ifndef EMB6_ASSERT_RET
#define EMB6_ASSERT_RET(x, r)  \
do { \
  if (!(x)) \
  { \
	  EMB6_ASSERT(x); \
    return r; \
  } \
} while(0)
#endif /* EMB6_ASSERT_RET */

#ifndef EMB6_ASSERT_FN
#define EMB6_ASSERT_FN(x, f)  \
do { \
  if (!(x)) \
  { \
	  EMB6_ASSERT(x); \
    return f(); \
  } \
} while(0)
#endif /* EMB6_ASSERT_FN */

#ifndef EMB6_ASSERT_S
#define EMB6_ASSERT_S(x, a, b) do{(a=b);EMB6_ASSERT(x)} while(0)
#endif /* EMB6_ASSERT_S */

#ifndef EMB6_ASSERT_RETS
#define EMB6_ASSERT_RETS(x, r, a, b) do{(a=b);EMB6_ASSERT_RET(x, r)} while(0)
#endif /* EMB6_ASSERT_RETS */

#ifndef EMB6_ASSERT_RETFN
#define EMB6_ASSERT_RETFN(x, f, a, b) do{(a=b);EMB6_ASSERT_FN(x, f)} while(0)
#endif /* EMB6_ASSERT_RETFN */

#endif /* #if defined(EMB6_ASSERT_CFG) */

/** this macro is required by DTLS */
#if defined(EMB6_ASSERT_CFG)
#define assert(x)                           EMB6_ASSERT(x)
#else
#define assert(x)
#endif /* #if defined(EMB6_ASSERT_CFG) */

#endif /* __EMB6ASSERT_H__ */
