/**
********************************************************************************
\file   targetdefs/arm_altera.h

\brief  Target specific definitions for Altera ARM core systems

This file contains target specific definitions for Altera ARM cortex A9 systems.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

#ifndef _INC_targetdefs_arm_altera_H_
#define _INC_targetdefs_arm_altera_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <lock.h>

#include <oplk/basictypes.h>
#include <socal/socal.h>
#include <alt_cache.h>

#define ROM_INIT            // variables will be initialized directly in ROM (means no copy from RAM in startup)
#define ROM                 // code or variables mapped to ROM (i.e. flash)
                            // usage: CONST BYTE ROM foo = 0x00;

#define MEM                 // Memory attribute to optimize speed and code of pointer access.

#ifndef CONST
#define CONST       const   // variables mapped to ROM (i.e. flash)
#endif

#define OPLKDLLEXPORT

#define UNUSED_PARAMETER(par)                   (void)par

#if !defined(__OPTIMIZE__)
//restore default: disable inlining if optimization is disabled
#define INLINE_FUNCTION
#undef  INLINE_ENABLED
#undef  INLINE_FUNCTION_DEF
#endif

#include <oplk/section-default.h>

#ifndef NDEBUG
#define PRINTF(...)                             printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

// FIXME
#define IORD_32DIRECT(base, offset)             alt_read_word((unsigned int)base + (unsigned int)offset)
#define IORD_16DIRECT(base, offset)             alt_read_hword((unsigned int)base + (unsigned int)offset)
#define IORD_8DIRECT(base, offset)              alt_read_byte((unsigned int)base + (unsigned int)offset)

#define IOWR_32DIRECT(base, offset, dword)      alt_write_word((unsigned int)base + (unsigned int)offset, dword)
#define IOWR_16DIRECT(base, offset, word)       alt_write_hword((unsigned int)base + (unsigned int)offset, word)
#define IOWR_8DIRECT(base, offset, byte)        alt_write_byte((unsigned int)base + (unsigned int)offset, byte)

/* NOTE:
 * ARM does not support atomic instructions, hence, pseudo atomic
 * macro is applied with spin lock.
 */

#define OPLK_ATOMIC_T               uint8_t
#define OPLK_LOCK_T                 LOCK_T
#define OPLK_ATOMIC_INIT(base)             \
    if (target_initLock(&base->lock) != 0) \
        return kErrorNoResource
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
    target_lock();                                    \
    oldval = IORD_8DIRECT(address, 0);                \
    IOWR_8DIRECT(address, 0, newval);                 \
    target_unlock()

#define CACHE_ALIGNED_BYTE_CHECK    (ALT_CACHE_LINE_SIZE - 1)

#define DUMMY(...)
// FIXME screwed if the base address flooring truncates more address than the range ceiling appends //EDIT: now its fixed
#if 0 // TODO find a way to detect the cache configuration
#define OPLK_DCACHE_FLUSH(base, range)                                                                                                                \
    ({                                                                                                                                                \
         uint32_t tempBase = (uint32_t) (((uint32_t) base) & ~((uint32_t) CACHE_ALIGNED_BYTE_CHECK));                                                 \
         uint32_t tempCeil = (uint32_t) ((((uint32_t) base + (uint32_t) range) + CACHE_ALIGNED_BYTE_CHECK) & ~((uint32_t) CACHE_ALIGNED_BYTE_CHECK)); \
         alt_cache_system_clean((void*) tempBase, (size_t) (tempCeil - tempBase));                                                                    \
         DUMMY("base: %lu, range: %lu, tempBase: %lu, tempCeil: %lu\n", base, range, tempBase, tempCeil);                                             \
         DUMMY("D.");                                                                                                                                 \
     })
//alt_cache_system_clean((void*) ((uint32_t)base & ~((uint32_t) CACHE_ALIGNED_BYTE_CHECK)), (size_t) (((size_t) range + CACHE_ALIGNED_BYTE_CHECK) & ~((size_t) CACHE_ALIGNED_BYTE_CHECK)));

#define OPLK_DCACHE_INVALIDATE(base, range)                                                                                                           \
    ({                                                                                                                                                \
         uint32_t tempBase = (uint32_t) (((uint32_t) base) & ~((uint32_t) CACHE_ALIGNED_BYTE_CHECK));                                                 \
         uint32_t tempCeil = (uint32_t) ((((uint32_t) base + (uint32_t) range) + CACHE_ALIGNED_BYTE_CHECK) & ~((uint32_t) CACHE_ALIGNED_BYTE_CHECK)); \
         alt_cache_system_invalidate((void*) tempBase, (size_t) (tempCeil - tempBase));                                                               \
         DUMMY("base: %lu, range: %lu, tempBase: %lu, tempCeil: %lu\n", base, range, tempBase, tempCeil);                                             \
         DUMMY("I.");                                                                                                                                 \
     })
//alt_cache_system_invalidate((void*) ((uint32_t) base & ~((uint32_t) CACHE_ALIGNED_BYTE_CHECK)), (size_t) (((size_t) range + CACHE_ALIGNED_BYTE_CHECK) & ~((size_t) CACHE_ALIGNED_BYTE_CHECK)))
#else

#define OPLK_DCACHE_FLUSH(base, range)
#define OPLK_DCACHE_INVALIDATE(base, range)
#endif
#endif /* _INC_targetdefs_arm_altera_H_ */
