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


#ifndef NDEBUG
#define PRINTF(...)                             printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

//TODO Is the following ideal

#define FPGA_BUS_WIDTH                              32
#define __IO_CALC_ADDRESS_NATIVE(base, offset) \
    (base + offset * (FPGA_BUS_WIDTH / 8))
#define IORD16(base, offset)                    alt_read_word(base + offset * (FPGA_BUS_WIDTH / 8))
#define IORD32(base, offset)                    alt_read_word(base + offset * (FPGA_BUS_WIDTH / 8))
#define IOWR16(base, offset, val)               alt_write_word(base + offset * (FPGA_BUS_WIDTH / 8), val)
#define IOWR32(base, offset, val)               alt_write_word(base + offset * (FPGA_BUS_WIDTH / 8), val)

#define IORD_32DIRECT(base, offset)             alt_read_word((unsigned int)base + (unsigned int)offset)
#define IORD_16DIRECT(base, offset)             alt_read_hword((unsigned int)base + (unsigned int)offset)
#define IORD_8DIRECT(base, offset)              alt_read_byte((unsigned int)base + (unsigned int)offset)

#define IOWR_32DIRECT(base, offset, dword)      alt_write_word((unsigned int)base + (unsigned int)offset, dword)
#define IOWR_16DIRECT(base, offset, word)       alt_write_hword((unsigned int)base + (unsigned int)offset, word)
#define IOWR_8DIRECT(base, offset, byte)        alt_write_byte((unsigned int)base + (unsigned int)offset, byte)

/* STATUS register */
#define ALTERA_AVALON_TIMER_STATUS_REG              0
#define IOADDR_ALTERA_AVALON_TIMER_STATUS(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_STATUS_REG)
#define IORD_ALTERA_AVALON_TIMER_STATUS(base) \
    IORD16(base, ALTERA_AVALON_TIMER_STATUS_REG)
#define IOWR_ALTERA_AVALON_TIMER_STATUS(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_STATUS_REG, data)
#define ALTERA_AVALON_TIMER_STATUS_TO_MSK           (0x1)
#define ALTERA_AVALON_TIMER_STATUS_TO_OFST          (0)
#define ALTERA_AVALON_TIMER_STATUS_RUN_MSK          (0x2)
#define ALTERA_AVALON_TIMER_STATUS_RUN_OFST         (1)

/* CONTROL register */
#define ALTERA_AVALON_TIMER_CONTROL_REG             1
#define IOADDR_ALTERA_AVALON_TIMER_CONTROL(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_CONTROL_REG)
#define IORD_ALTERA_AVALON_TIMER_CONTROL(base) \
    IORD16(base, ALTERA_AVALON_TIMER_CONTROL_REG)
#define IOWR_ALTERA_AVALON_TIMER_CONTROL(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_CONTROL_REG, data)
#define ALTERA_AVALON_TIMER_CONTROL_ITO_MSK         (0x1)
#define ALTERA_AVALON_TIMER_CONTROL_ITO_OFST        (0)
#define ALTERA_AVALON_TIMER_CONTROL_CONT_MSK        (0x2)
#define ALTERA_AVALON_TIMER_CONTROL_CONT_OFST       (1)
#define ALTERA_AVALON_TIMER_CONTROL_START_MSK       (0x4)
#define ALTERA_AVALON_TIMER_CONTROL_START_OFST      (2)
#define ALTERA_AVALON_TIMER_CONTROL_STOP_MSK        (0x8)
#define ALTERA_AVALON_TIMER_CONTROL_STOP_OFST       (3)

/* Period and SnapShot Register for COUNTER_SIZE = 32 */
/*----------------------------------------------------*/
/* PERIODL register */
#define ALTERA_AVALON_TIMER_PERIODL_REG             2
#define IOADDR_ALTERA_AVALON_TIMER_PERIODL(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_PERIODL_REG)
#define IORD_ALTERA_AVALON_TIMER_PERIODL(base) \
    IORD16(base, ALTERA_AVALON_TIMER_PERIODL_REG)
#define IOWR_ALTERA_AVALON_TIMER_PERIODL(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_PERIODL_REG, data)
#define ALTERA_AVALON_TIMER_PERIODL_MSK             (0xFFFF)
#define ALTERA_AVALON_TIMER_PERIODL_OFST            (0)

/* PERIODH register */
#define ALTERA_AVALON_TIMER_PERIODH_REG             3
#define IOADDR_ALTERA_AVALON_TIMER_PERIODH(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_PERIODH_REG)
#define IORD_ALTERA_AVALON_TIMER_PERIODH(base) \
    IORD16(base, ALTERA_AVALON_TIMER_PERIODH_REG)
#define IOWR_ALTERA_AVALON_TIMER_PERIODH(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_PERIODH_REG, data)
#define ALTERA_AVALON_TIMER_PERIODH_MSK             (0xFFFF)
#define ALTERA_AVALON_TIMER_PERIODH_OFST            (0)

/* SNAPL register */
#define ALTERA_AVALON_TIMER_SNAPL_REG               4
#define IOADDR_ALTERA_AVALON_TIMER_SNAPL(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_SNAPL_REG)
#define IORD_ALTERA_AVALON_TIMER_SNAPL(base) \
    IORD16(base, ALTERA_AVALON_TIMER_SNAPL_REG)
#define IOWR_ALTERA_AVALON_TIMER_SNAPL(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_SNAPL_REG, data)
#define ALTERA_AVALON_TIMER_SNAPL_MSK               (0xFFFF)
#define ALTERA_AVALON_TIMER_SNAPL_OFST              (0)

/* SNAPH register */
#define ALTERA_AVALON_TIMER_SNAPH_REG               5
#define IOADDR_ALTERA_AVALON_TIMER_SNAPH(base) \
    __IO_CALC_ADDRESS_NATIVE(base, ALTERA_AVALON_TIMER_SNAPH_REG)
#define IORD_ALTERA_AVALON_TIMER_SNAPH(base) \
    IORD16(base, ALTERA_AVALON_TIMER_SNAPH_REG)
#define IOWR_ALTERA_AVALON_TIMER_SNAPH(base, data) \
    IOWR16(base, ALTERA_AVALON_TIMER_SNAPH_REG, data)
#define ALTERA_AVALON_TIMER_SNAPH_MSK               (0xFFFF)
#define ALTERA_AVALON_TIMER_SNAPH_OFST              (0)

#define IOADDR_ALTERA_AVALON_PIO_DATA(base)                 __IO_CALC_ADDRESS_NATIVE(base, 0)
#define IORD_ALTERA_AVALON_PIO_DATA(base)                   IORD32(base, 0)
#define IOWR_ALTERA_AVALON_PIO_DATA(base, data)             IOWR32(base, 0, data)

#define IOADDR_ALTERA_AVALON_PIO_DIRECTION(base)            __IO_CALC_ADDRESS_NATIVE(base, 1)
#define IORD_ALTERA_AVALON_PIO_DIRECTION(base)              IORD32(base, 1)
#define IOWR_ALTERA_AVALON_PIO_DIRECTION(base, data)        IOWR32(base, 1, data)

#define IOADDR_ALTERA_AVALON_PIO_IRQ_MASK(base)             __IO_CALC_ADDRESS_NATIVE(base, 2)
#define IORD_ALTERA_AVALON_PIO_IRQ_MASK(base)               IORD32(base, 2)
#define IOWR_ALTERA_AVALON_PIO_IRQ_MASK(base, data)         IOWR32(base, 2, data)

#define IOADDR_ALTERA_AVALON_PIO_EDGE_CAP(base)             __IO_CALC_ADDRESS_NATIVE(base, 3)
#define IORD_ALTERA_AVALON_PIO_EDGE_CAP(base)               IORD32(base, 3)
#define IOWR_ALTERA_AVALON_PIO_EDGE_CAP(base, data)         IOWR32(base, 3, data)

#define IOADDR_ALTERA_AVALON_PIO_SET_BIT(base)              __IO_CALC_ADDRESS_NATIVE(base, 4)
#define IORD_ALTERA_AVALON_PIO_SET_BITS(base)               IORD32(base, 4)
#define IOWR_ALTERA_AVALON_PIO_SET_BITS(base, data)         IOWR32(base, 4, data)

#define IOADDR_ALTERA_AVALON_PIO_CLEAR_BITS(base)           __IO_CALC_ADDRESS_NATIVE(base, 5)
#define IORD_ALTERA_AVALON_PIO_CLEAR_BITS(base)             IORD32(base, 5)
#define IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(base, data)       IOWR32(base, 5, data)

/* NOTE:
 * ARM does not support atomic instructions, hence, pseudo atomic
 * macro is applied with spin lock.
 */

#define OPLK_MUTEX_T                uint8_t
#define OPLK_ATOMIC_T               uint8_t
#define OPLK_LOCK_T                 LOCK_T
#define OPLK_ATOMIC_INIT(base)             \
    if (target_initLock(&base->lock) != 0) \
        return kErrorNoResource
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
    target_lock();                                    \
    oldval = alt_read_byte(address);                  \
    alt_write_byte(address, newval);                  \
    target_unlock()

#define CACHE_ALIGNED_BYTE_CHECK    (ALT_CACHE_LINE_SIZE - 1)

#define DUMMY(...)
// FIXME screwed if the base address flooring truncates more address than the range ceiling appends //EDIT: now its fixed
#if 1 // TODO find a way to detect the cache configuration
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
