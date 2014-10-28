/**
********************************************************************************
\file   altera_arm/target-arm.c

\brief  Target specific functions for ARM on Altera SoC without OS

This target depending module provides several functions that are necessary for
systems without OS and not using the shared buffer library.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>
#include <oplk/debug.h>
#include <sys/unistd.h>
#include <alt_timers.h>
#include <alt_globaltmr.h>
#include <alt_interrupt.h>
#include <alt_cache.h>
#include <alt_fpga_manager.h>
#include <alt_bridge_manager.h>
#include <alt_address_space.h>
#include <alt_mpu_registers.h>

#include <common/target.h>
#include <alt_clock_manager.h>

#include <system.h>
//#include <xparameters.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define SECS_TO_MILLISECS    1000

#define FPGA_BUS_WIDTH          32
#define __IO_CALC_ADDRESS_NATIVE(base, offset) \
                                            (base + offset * (FPGA_BUS_WIDTH/8))
#define IORD16(base, offset)                alt_read_word(base + offset * (FPGA_BUS_WIDTH/8))
#define IORD32(base, offset)               alt_read_word(base + offset * (FPGA_BUS_WIDTH/8))
#define IOWR16(base, offset, val)           alt_write_word(base + offset * (FPGA_BUS_WIDTH/8), val)
#define IOWR32(base, offset, val)           alt_write_word(base + offset * (FPGA_BUS_WIDTH/8), val)

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



#define IOADDR_ALTERA_AVALON_PIO_DATA(base)           __IO_CALC_ADDRESS_NATIVE(base, 0)
#define IORD_ALTERA_AVALON_PIO_DATA(base)             IORD32(base, 0)
#define IOWR_ALTERA_AVALON_PIO_DATA(base, data)       IOWR32(base, 0, data)

#define IOADDR_ALTERA_AVALON_PIO_DIRECTION(base)      __IO_CALC_ADDRESS_NATIVE(base, 1)
#define IORD_ALTERA_AVALON_PIO_DIRECTION(base)        IORD32(base, 1)
#define IOWR_ALTERA_AVALON_PIO_DIRECTION(base, data)  IOWR32(base, 1, data)

#define IOADDR_ALTERA_AVALON_PIO_IRQ_MASK(base)       __IO_CALC_ADDRESS_NATIVE(base, 2)
#define IORD_ALTERA_AVALON_PIO_IRQ_MASK(base)         IORD32(base, 2)
#define IOWR_ALTERA_AVALON_PIO_IRQ_MASK(base, data)   IOWR32(base, 2, data)

#define IOADDR_ALTERA_AVALON_PIO_EDGE_CAP(base)       __IO_CALC_ADDRESS_NATIVE(base, 3)
#define IORD_ALTERA_AVALON_PIO_EDGE_CAP(base)         IORD32(base, 3)
#define IOWR_ALTERA_AVALON_PIO_EDGE_CAP(base, data)   IOWR32(base, 3, data)


#define IOADDR_ALTERA_AVALON_PIO_SET_BIT(base)       __IO_CALC_ADDRESS_NATIVE(base, 4)
#define IORD_ALTERA_AVALON_PIO_SET_BITS(base)         IORD32(base, 4)
#define IOWR_ALTERA_AVALON_PIO_SET_BITS(base, data)   IOWR32(base, 4, data)

#define IOADDR_ALTERA_AVALON_PIO_CLEAR_BITS(base)       __IO_CALC_ADDRESS_NATIVE(base, 5)
#define IORD_ALTERA_AVALON_PIO_CLEAR_BITS(base)         IORD32(base, 5)
#define IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(base, data)   IOWR32(base, 5, data)



/* Defintions for direction-register operation with bi-directional PIOs */
#define ALTERA_AVALON_PIO_DIRECTION_INPUT  0
#define ALTERA_AVALON_PIO_DIRECTION_OUTPUT 1

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// external vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

    volatile int a = 0;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static int              enableInterruptMaster(void);
static int              disableInterruptMaster(void);
static int              initializeFpga(void);
static int              initializeTimer(void);

static inline uint64_t  getTimerMaxScaledCount(ALT_GPT_TIMER_t tmr_id, uint32_t scalingFactor);
static inline uint64_t  getTimerCurrentScaledCount(ALT_GPT_TIMER_t tmr_id, uint32_t scalingFactor);
static inline uint64_t  getTimerTicksFromScaled(ALT_GPT_TIMER_t tmr_id, uint32_t scalingFactor, uint32_t scaledTimeDuration_p);
//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Get current system tick

This function returns the current system tick determined by the system timer.

\return The function returns the system tick in milliseconds

\ingroup module_target
*/
//------------------------------------------------------------------------------
UINT32 target_getTickCount(void)
{
    UINT32      ticks_l = 0;
    UINT32      ticks_h = 0;

    /* Uses global timer functions */
    /* Select the lower 32 bit of the timer value */
    //ticks = alt_gpt_counter_get(ALT_GPT_CPU_GLOBAL_TMR);
    //ticks = alt_gpt_time_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR);
//    if (alt_globaltmr_get(&ticks_h, &ticks_l) != ALT_E_SUCCESS)
//    {
//        DEBUG_LVL_ERROR_TRACE("Timer is returning junk!\n");
//    }
//
//    return ticks_l;
    return (uint32_t) getTimerCurrentScaledCount(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS);
}

//------------------------------------------------------------------------------
/**
\brief    Enables global interrupt

This function enables/disables global interrupts.

\param  fEnable_p               TRUE = enable interrupts
                                FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableGlobalInterrupt(UINT8 fEnable_p)
{
    static INT    lockCount = 0;

    if (fEnable_p != FALSE) // restore interrupts
    {
        if (--lockCount == 0)
        {
           // printf("e");
            enableInterruptMaster();
        }
    }
    else
    {                       // disable interrupts
        if (lockCount == 0)
        {
           //printf("%d\n", a);
            disableInterruptMaster();
        }

        lockCount++;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize target specific stuff

The function initialize target specific stuff which is needed to run the
openPOWERLINK stack.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------


void cs3_isr_1(uint32_t icciar_p, void* context_p)
{
    alt_int_dist_pending_clear(ALT_INT_INTERRUPT_F2S_FPGA_IRQ1);
    alt_int_dist_pending_clear(ALT_INT_INTERRUPT_F2S_FPGA_IRQ3);
    alt_globaltmr_int_clear_pending();
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE, 0x3);
    printf("ISR\n");
    *(volatile int *)context_p = 1;

    a = 2;
    return;
}

tOplkError target_init(void)
{
    tOplkError          oplkRet = kErrorOk;
    ALT_STATUS_CODE     halRet = ALT_E_SUCCESS;

    // Enable Cache
    //halRet = alt_cache_system_enable();
    halRet = alt_cache_system_disable();
    //alt_cache_l1_data_disable();
    if (halRet != ALT_E_SUCCESS)
    {
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

    // Initialize Interrupts
    // initialize the global interrupt controller
    halRet = alt_int_global_init();
    if (halRet != ALT_E_SUCCESS)
    {
        printf("global IRQ controller initialization Failed!!\n");
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

    // initialize the cpu interrupt interface
    halRet = alt_int_cpu_init();
    if (halRet != ALT_E_SUCCESS)
    {
        printf("CPU IRQ interface initialization Failed!!\n");
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

    if (initializeTimer() != 0)
    {
        printf("general purpose time module intialization Failed!!\n");
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

    target_msleep(5000);

    if (0) //(initializeFpga() != 0)
    {
        oplkRet = kErrorGeneralError;
        printf("FPGA initialization Failed!!\n");
    }

    if (alt_bridge_init(ALT_BRIDGE_LWH2F, NULL, NULL) != ALT_E_SUCCESS)
    {
        oplkRet = kErrorGeneralError;
        printf("LWH2F initialization Failed!!: 0x%X\n", alt_bridge_init(ALT_BRIDGE_LWH2F, NULL, NULL));
    }

    if (alt_bridge_init(ALT_BRIDGE_H2F, NULL, NULL) != ALT_E_SUCCESS)
    {
        oplkRet = kErrorGeneralError;
        printf("H2F initialization Failed!!: 0x%X\n", alt_bridge_init(ALT_BRIDGE_H2F, NULL, NULL));
    }

//    if (alt_addr_space_remap(ALT_ADDR_SPACE_MPU_ZERO_AT_BOOTROM,
//                             ALT_ADDR_SPACE_NONMPU_ZERO_AT_SDRAM,
//                             ALT_ADDR_SPACE_H2F_ACCESSIBLE,
//                             ALT_ADDR_SPACE_LWH2F_ACCESSIBLE)
//        == ALT_E_SUCCESS)
//    {
//        oplkRet = kErrorGeneralError;
//        printf("Address remapping Failed!!\n");
//    }
    // test code

    target_enableSyncIrq(FALSE);
    target_regSyncIrqHdl(cs3_isr_1, &a);
    target_enableSyncIrq(TRUE);
    //enableInterruptMaster();


Exit:
    return oplkRet;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup target specific stuff

The function cleans-up target specific stuff.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_cleanup(void)
{
    //FIXME
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param  milliSeconds_p            Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_msleep(UINT32 milliSeconds_p)
{
    uint64_t                startTickStamp = alt_globaltmr_get64();
    uint64_t                waitTickCount = getTimerTicksFromScaled(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS, milliSeconds_p);
    volatile uint32_t*      glbTimerRegCntBase_l = (volatile uint32_t*) (GLOBALTMR_BASE + GLOBALTMR_CNTR_LO_REG_OFFSET);
    volatile uint32_t*      glbTimerRegCntBase_h = (volatile uint32_t*) (GLOBALTMR_BASE + GLOBALTMR_CNTR_HI_REG_OFFSET);
    uint64_t                curTickStamp = 0;             //((((*glbTimerRegCntBase_h) << 32) & ~UINT32_MAX) | (*glbTimerRegCntBase_l));
    uint32_t                temp = 0;
    uint32_t                hi = 0;
    uint32_t                lo = 0;
    volatile uint32_t       waitCount = 0;
    uint32_t                waitCountLimit = (uint32_t) (5 + (waitTickCount >> 12));
    uint32_t                entryCount = 0;
    uint8_t                 fExit = FALSE;
    uint8_t                 readCntLimit = 3;

    //printf("CurTime: %llu, startTime: %llu, waitCountLimit: %lu\n", curTickStamp, startTickStamp, waitCountLimit);
    curTickStamp = alt_globaltmr_get64();
    //printf("CurTime: %llu, startTime: %llu, waitTickCount: %llu\n", curTickStamp, startTickStamp, waitTickCount);
    //printf("s");
    while (fExit != TRUE)
    {
        fExit = FALSE;
        //if ((waitCount % waitCountLimit) == 0)
        if (waitCount == waitCountLimit)
        {
            //printf("2\n");
            readCntLimit = 3;
            do
            {
                //printf("3\n");
                temp = (*glbTimerRegCntBase_h);
                lo =  (*glbTimerRegCntBase_l);
                hi = (*glbTimerRegCntBase_h);
                //printf("3.5\n");
            } while ((temp != hi) && (--readCntLimit));

            entryCount++;

            if (readCntLimit != 0)
            {
                //printf("4\n");
                curTickStamp =  (uint64_t) hi;
                curTickStamp =  (((curTickStamp << 32) & ~((uint64_t) UINT32_MAX)) | lo);
                if ((((curTickStamp >= startTickStamp) && ((curTickStamp - startTickStamp) < waitTickCount)) ||
                     ((curTickStamp < startTickStamp) && ((UINT64_MAX - startTickStamp) +
                                                          curTickStamp) < waitTickCount)))
                {
                    fExit = FALSE;
                }
                else
                {
                    //printf("\n");
                    //printf("5:: hi: %lu, lo: %lu\n", hi, lo);
                    fExit = TRUE;
                }

                waitCount = 1;
            }
            else
            {
                //printf("6\n");
                waitCount--;
            }
        }

        if (waitCount % 50000 == 0)
        {
            //if (waitCount == 100000)
            //printf(".......................................");
            //printf("waitCount: %lu\n", waitCount);
            //printf(".");
        }

        waitCount++;
    }

    //printf("\n");
    //printf("CurTime: %llu, startTime: %llu, entryCount: %lu\n", curTickStamp, startTickStamp, entryCount);
}

//void target_msleep(UINT32 milliSeconds_p)
//{
//	uint64_t			msStartTime = getTimerCurrentScaledCount(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS); //alt_gpt_curtime_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR);
//	//uint8_t				fExit = FALSE;
//	volatile uint64_t			msCurTime = (volatile uint64_t) msStartTime;
//	uint64_t		    msMaxTime = getTimerMaxScaledCount(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS);
//	uint32_t			waitCount = 0;
//	uint32_t			waitCount1 = 0;
//
//	printf("CurTime: %llu, startTime: %llu, maxTime: %llu\n", msCurTime, msStartTime, msMaxTime);
//
//	while ((((msCurTime >= msStartTime) && ((msCurTime - msStartTime) <= milliSeconds_p))
//			|| ((msCurTime < msStartTime) && ((msMaxTime - msStartTime)
//				+ msCurTime) <= milliSeconds_p)))
//	{
//		//FILE* fp = tmpfile();
///*
//		if (waitCount1++ > 500)
//		{
//			waitCount1 = 0;
//			msCurTime = (volatile uint64_t) getTimerCurrentScaledCount(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS);
//		}
//
//		if (waitCount++ > 500000)
//		{
//			waitCount = 0;
//			printf("50000\n");
//		}
///*/
//		waitCount++;
//		msCurTime = (volatile uint64_t) getTimerCurrentScaledCount(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS);
//		//*/
//
//		//fclose(fp);
//	}
//
//	printf("waitCount: %lu\n", waitCount);
//	printf("CurTime: %llu, startTime: %llu, maxTime: %llu\n", msCurTime, msStartTime, msMaxTime);
//
////	while (!fExit)
////	{
//////		if ((((alt_gpt_curtime_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR) > msStartTime) &&
//////			 (alt_gpt_curtime_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR)	- msStartTime) <= milliSeconds_p))
//////			|| ((alt_gpt_curtime_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR) < msStartTime) &&
//////					(alt_gpt_curtime_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR) +
//////							alt_gpt_maxtime_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR)
//////							- msStartTime) <= milliSeconds_p))
//////		msCurTime = alt_gpt_curtime_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR);
//////
//////		if (((msCurTime <= msStartTime) && ((msStartTime - msCurTime) <= milliSeconds_p))
//////			|| ((msCurTime > msStartTime) && ((msMaxTime - msCurTime)
//////				+ msStartTime) <= milliSeconds_p))
//////		{
//////			printf("CurTime: %u, startTime: %u, maxTime: %u\n", msCurTime, msStartTime, msMaxTime);
//////			//printf(".\n");
////////			for (unsigned long i = 0; i < 50000000; i++)
////////				__asm("NOP");
//////			// wait
//////			waitCount++;
//////		}
//////		else
//////		{
//////			printf("%u\n", waitCount);
//////			while (waitCount--)
//////			fExit = TRUE;
//////		}
////
////
////		msCurTime = getTimerCurrentScaledCount(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS);
////
////		//printf("CurTime: %llu, startTime: %llu, maxTime: %llu\n", msCurTime, msStartTime, msMaxTime);
////		if (((msCurTime >= msStartTime) && ((msCurTime - msStartTime) <= milliSeconds_p))
////			|| ((msCurTime < msStartTime) && ((msMaxTime - msStartTime)
////				+ msCurTime) <= milliSeconds_p))
////		{
////			//for (uint64_t i =0; i < 0xFFFFFFFF; i++);
////			fExit = FALSE;
////			waitCount++;
////		}
////		else
////		{
////			//printf("CurTime: %llu, startTime: %llu, maxTime: %llu\n", msCurTime, msStartTime, msMaxTime);
////			//printf("waitCount: %u\n", waitCount);
////			//while (waitCount--);
////			waitCount = 0;
////			fExit = TRUE;
////		}
////
////	}
//}

//------------------------------------------------------------------------------
/**
\brief Register synchronization interrupt handler

The function registers the ISR for target specific synchronization interrupt
used by the application for PDO and event synchronization.

\param  callback_p              Interrupt handler
\param  pArg_p                  Argument to be passed while calling the handler

\return The function returns the error code as a integer value
\retval 0 if able to register
\retval other if not

\ingroup module_target
*/
//------------------------------------------------------------------------------


void target_regSyncIrqHdl(void* callback_p, void* pArg_p)
{
    ALT_INT_INTERRUPT_t    irqId = ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + 1;

    if (callback_p == NULL)
    {
        printf("UnReg Hostif ISR\n");
        alt_int_isr_unregister(irqId);
    }
    else
    {
        printf("Reg Hostif ISR\n");
        if ( alt_int_isr_register(irqId, callback_p, pArg_p) != ALT_E_SUCCESS)
        {
            printf("isr registration failed\n");
        }
        else
        {
            printf("isr registration successful\n");
        }
    }

    alt_int_isr_register(ALT_INT_INTERRUPT_PPI_TIMER_GLOBAL, callback_p, NULL);
}

//------------------------------------------------------------------------------
/**
\brief Sync interrupt control routine

The function is used to enable or disable the sync interrupt

\param  fEnable_p              enable if TRUE, disable if FALSE

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableSyncIrq(BOOL fEnable_p)
{
    ALT_STATUS_CODE         ret;
    ALT_INT_INTERRUPT_t     irqId = ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + 1;
    int                     cpu_target = 0x1;                                             // cortexA9_0

    if (fEnable_p)
    {
        alt_int_dist_pending_clear(ALT_INT_INTERRUPT_F2S_FPGA_IRQ0);
        alt_int_dist_pending_clear(ALT_INT_INTERRUPT_F2S_FPGA_IRQ1);
        alt_int_dist_pending_clear(ALT_INT_INTERRUPT_F2S_FPGA_IRQ3);
        for (irqId = ALT_INT_INTERRUPT_F2S_FPGA_IRQ1; irqId <= ALT_INT_INTERRUPT_F2S_FPGA_IRQ1; irqId++)
        {
            printf("Enable IRQ: %X(%X)\n", fEnable_p,irqId);
            if (alt_int_dist_target_set(irqId, cpu_target) != ALT_E_SUCCESS)
            {
                printf("trigger set failed\n");
            }
            //else if (alt_int_dist_trigger_set(irqId, ALT_INT_TRIGGER_EDGE) != ALT_E_SUCCESS)
            else if (alt_int_dist_trigger_set(irqId, ALT_INT_TRIGGER_EDGE) != ALT_E_SUCCESS)
            {
                printf("trigger set failed\n");
            }
            else if (alt_int_dist_enable(irqId) != ALT_E_SUCCESS)
            {
                // Set interrupt distributor target
                printf("distributor set failed\n");
            }
            else
                printf("Irq enabled\n");
        }

        printf("PIO data status: 0x%X\n", IORD_ALTERA_AVALON_PIO_DATA(BUTTON_PIO_BASE));
        printf("PIO edge capture status: 0x%X\n", IORD_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE));
        printf("PIO irq status: 0x%X\n", IORD_ALTERA_AVALON_PIO_IRQ_MASK(BUTTON_PIO_BASE));
        IOWR_ALTERA_AVALON_PIO_IRQ_MASK(BUTTON_PIO_BASE, 0x3);
        IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE, 0x3);
        printf("PIO edge capture status: 0x%X\n", IORD_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE));
        printf("PIO irq status: 0x%X\n", IORD_ALTERA_AVALON_PIO_IRQ_MASK(BUTTON_PIO_BASE));

        // Set interrupt trigger type
        if (alt_int_dist_trigger_set(ALT_INT_INTERRUPT_PPI_TIMER_GLOBAL, ALT_INT_TRIGGER_AUTODETECT) != ALT_E_SUCCESS)
        {
            printf("trigger set failed\n");
        }

        // Enable interrupt at the distributor level
        //if (alt_int_dist_enable(ALT_INT_INTERRUPT_PPI_TIMER_GLOBAL) != ALT_E_SUCCESS)
        {
            //printf("distributor set failed\n");
        }

    }
    else
    {
        if (alt_int_dist_disable(irqId) == ALT_E_SUCCESS)
            printf("disable irq\n");
        else
            printf("nothing to disable irq\n");

        printf("PIO data status: 0x%X\n", IORD_ALTERA_AVALON_PIO_DATA(BUTTON_PIO_BASE));
        printf("PIO edge capture status: 0x%X\n", IORD_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE));
        printf("PIO irq status: 0x%X\n", IORD_ALTERA_AVALON_PIO_IRQ_MASK(BUTTON_PIO_BASE));
        IOWR_ALTERA_AVALON_PIO_IRQ_MASK(BUTTON_PIO_BASE, 0x0);
        IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE, 0x3);
        printf("PIO edge capture status: 0x%X\n", IORD_ALTERA_AVALON_PIO_EDGE_CAP(BUTTON_PIO_BASE));
        printf("PIO irq status: 0x%X\n", IORD_ALTERA_AVALON_PIO_IRQ_MASK(BUTTON_PIO_BASE));
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Enable the global interrupt master

*/
//------------------------------------------------------------------------------
static int enableInterruptMaster(void)
{
    ALT_STATUS_CODE     retStatus = ALT_E_SUCCESS;
    int                 ret = 0;

    // enable global interrupt master
    // Global interrupt enable
    retStatus = alt_int_global_enable();
    if (retStatus != ALT_E_SUCCESS)
    {
        printf("enabling interrupts failed\n");
        ret = -1;
        goto Exit;
    }

    // CPU interface global enable
    retStatus = alt_int_cpu_enable();
    if (retStatus != ALT_E_SUCCESS)
    {
        printf("enabling interrupts failed\n");
        ret = -1;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Disable the global interrupt master

*/
//------------------------------------------------------------------------------
static int disableInterruptMaster(void)
{
    ALT_STATUS_CODE     retStatus = ALT_E_SUCCESS;
    int                 ret = 0;

    // Disable all interrupts from the distributor
    retStatus = alt_int_global_disable();
    if (retStatus != ALT_E_SUCCESS)
    {
        ret = -1;
        goto Exit;
    }

    // Reset the CPU interface
    retStatus = alt_int_cpu_disable();
    if (retStatus != ALT_E_SUCCESS)
    {
        ret = -1;
        goto Exit;
    }

Exit:
    return ret;
}

static int initializeFpga(void)
{
    int                 ret = 0;
    ALT_STATUS_CODE     halRet = ALT_E_SUCCESS;

    /* intialize the FPGA control */
    if (alt_fpga_init() != ALT_E_SUCCESS) // initialize the FPGA manager
    {
        printf("FPGA interface initialization Failed!!\n");
        ret = -1;
        goto Exit;
    }
    else if (alt_fpga_state_get() == ALT_FPGA_STATE_POWER_OFF) // check the FPGA state
    {
        printf("FPGA is powered Off!!\n");
        ret = -1;
        goto Exit;
    }
    else if (!alt_fpga_control_is_enabled()) // check if CPU has the control of the FPGA control block
    {           // if not acquire control
        halRet = alt_fpga_control_enable();
    }

    if (halRet != ALT_E_SUCCESS)
    {
        printf("FPGA interface control could not be acquired\n");
        ret = -1;
        goto Exit;
    }

    /* Program FPGA here if required */

    /* Enable the HPS-FPGA bridge */
    if (alt_bridge_init(ALT_BRIDGE_F2H, NULL, NULL) != ALT_E_SUCCESS)
    {
        printf("F2H initialization Failed!!\n");
        ret = -1;
        goto Exit;
    }
    else if (alt_bridge_init(ALT_BRIDGE_H2F, NULL, NULL) != ALT_E_SUCCESS)
    {
        printf("H2F initialization Failed!!\n");
        ret = -1;
        goto Exit;
    }
    else if (alt_bridge_init(ALT_BRIDGE_LWH2F, NULL, NULL) != ALT_E_SUCCESS)
    {
        printf("LWH2F initialization Failed!!\n");
        ret = -1;
        goto Exit;
    }

    if (alt_addr_space_remap(ALT_ADDR_SPACE_MPU_ZERO_AT_BOOTROM,
                             ALT_ADDR_SPACE_NONMPU_ZERO_AT_OCRAM,
                             ALT_ADDR_SPACE_H2F_ACCESSIBLE,
                             ALT_ADDR_SPACE_LWH2F_ACCESSIBLE) !=
        ALT_E_SUCCESS)
    {
        printf("FPGA address space remapping Failed!!\n");
        ret = -1;
        goto Exit;
    }

Exit:
    return ret;
}

static int initializeTimer(void)
{
    int                 ret = 0;
    ALT_STATUS_CODE     halRet = ALT_E_SUCCESS;

    // initialize timer, only the 64 bit global timer is used
    halRet =  alt_globaltmr_init();
    if (halRet != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("general purpose time module intialization Failed!!\n");
        ret = -1;
        goto Exit;
    }

    // set the comparator value to the maximum global timer value even though we do not use it
    // the 'alt_gpt_curtime_millisecs_get()' api uses this to determine current time
    if ((alt_globaltmr_autoinc_set(1) != ALT_E_SUCCESS) ||
        (alt_globaltmr_comp_set64(GLOBALTMR_MAX) != ALT_E_SUCCESS))
    {
        DEBUG_LVL_ERROR_TRACE("Auto increment mode could not be enabled for this timer!\n");
    }

    // Check if the timer  is already running
    halRet = alt_gpt_tmr_is_running(ALT_GPT_CPU_GLOBAL_TMR);
    if (halRet == ALT_E_FALSE)
    {
        DEBUG_LVL_TIMERU_TRACE("Timer has to be started!\n");
        // timer is not running, so try to start it
        halRet =  alt_globaltmr_start();
        //halRet =  alt_globaltmr_stop();
    }
    else if (halRet == ALT_E_BAD_ARG)
    {
        // this timer instance does not exist
        ret = -1;
        goto Exit;
    }
    else // if (retStatus == ALT_E_TRUE)
    {
        // timer is already running. try to reset it
        // retStatus = alt_gpt_tmr_reset(ALT_GPT_CPU_GLOBAL_TMR);
        // Do not do it as its not needed and we would require to set its mode
        // configuration again if we did
    }

    // check if any of the previous 2 timer operation failed, it can not be
    // a bad instance as that is covered
    if (halRet != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("Timer initialization Failed!!\n");
        ret = -1;
        goto Exit;
    }

    DEBUG_LVL_TIMERU_TRACE("Timer Comparator Mode: %u, value: %lu",
                           alt_globaltmr_is_comp_mode(), alt_globaltmr_comp_get64() - 1);
    DEBUG_LVL_TIMERU_TRACE("Timer Auto increment mode: %u, value: %u\n",
                           alt_globaltmr_is_autoinc_mode(), alt_globaltmr_autoinc_get());

    // stop the comparison function for this timer
    if ((alt_globaltmr_autoinc_mode_stop() != ALT_E_SUCCESS) ||
        (alt_globaltmr_comp_mode_start() != ALT_E_SUCCESS))
    {
        DEBUG_LVL_ERROR_TRACE("Timer mode could not be set\n");
        ret = -1;
        goto Exit;
    }

    DEBUG_LVL_TIMERU_TRACE("Timer Comparator Mode: %u, value: %lu",
                           alt_globaltmr_is_comp_mode(), alt_globaltmr_comp_get64() - 1);
    DEBUG_LVL_TIMERU_TRACE("Timer Auto increment mode: %u, value: %u\n",
                           alt_globaltmr_is_autoinc_mode(), alt_globaltmr_autoinc_get());

    // disable comparator interrupts from this timer
    if ((alt_globaltmr_int_disable() != ALT_E_SUCCESS) ||
        (alt_globaltmr_int_clear_pending() != ALT_E_SUCCESS))
    {
        DEBUG_LVL_ERROR_TRACE("Timer IRQ could not be disabled\n");
        ret = -1;
        goto Exit;
    }

Exit:
    return ret;
}

static inline uint64_t getTimerTicksFromScaled(ALT_GPT_TIMER_t tmr_id, uint32_t scalingFactor, uint32_t scaledTimeDuration_p)
{
    uint64_t        ticks = 0;                      // value to return
    ALT_CLK_t       clkSrc = ALT_CLK_UNKNOWN;
    uint32_t        preScaler = 0;
    uint32_t        freq = 1;

    preScaler = alt_gpt_prescaler_get(tmr_id);
    if (preScaler <= UINT8_MAX)
    {
        if (tmr_id == ALT_GPT_CPU_GLOBAL_TMR)       // Global Timer
        {
            ticks = 1;
            clkSrc = ALT_CLK_MPU_PERIPH;
        }
        else
        {
            ticks = 0;
            goto Exit;
        }

        if (alt_clk_freq_get(clkSrc, &freq) == ALT_E_SUCCESS)
        {
            // convert into clock ticks
            ticks *= freq;

            // convert into seconds
            ticks *= (uint64_t) (scaledTimeDuration_p / scalingFactor);

            // convert into timer ticks

            ticks /= (preScaler + 1);
        }
    }

Exit:
    return ticks;
}

static inline uint64_t getTimerCurrentScaledCount(ALT_GPT_TIMER_t tmr_id, uint32_t scalingFactor)
{
    uint64_t        timeStamp_l = 0;                // r2 & r3
    uint64_t        timeStamp_h = 0;
    uint64_t        timeStamp = 0;
    uint64_t        scaledTime = 0;                 // value to return
    ALT_CLK_t       clkSrc = ALT_CLK_UNKNOWN;
    uint32_t        preScaler = 0;
    uint32_t        freq = 1;

    preScaler = alt_gpt_prescaler_get(tmr_id);
    if (preScaler <= UINT8_MAX)
    {
        if (tmr_id == ALT_GPT_CPU_GLOBAL_TMR)       // Global Timer
        {
            //timeStamp = alt_globaltmr_get64();
            alt_globaltmr_get((uint32_t*)&timeStamp_h, (uint32_t*)&timeStamp_l);
            clkSrc = ALT_CLK_MPU_PERIPH;
        }
        else
        {
            scaledTime = 0;
            goto Exit;
        }

        if (alt_clk_freq_get(clkSrc, &freq) == ALT_E_SUCCESS)
        {
            //             // remaining count divided by cycles-per-second becomes seconds,
            //             // milliseconds, microseconds, or nanoseconds remaining
            //           timeStamp /= freq;
            //             timeStamp *= (preScaler + 1);
            //                 // ARM can usually do a 32x64 bit multiply faster than a 64x64 bit multiply
            //             timeStamp *= scalingFactor;
            //             scaledTime = (timeStamp > UINT64_MAX) ? UINT64_MAX : (uint64_t) timeStamp;

            // printf("timeStamp_l: %llu, timeStamp_h: %llu\n", timeStamp_l, timeStamp_h);
            timeStamp_l *= (preScaler + 1);
            timeStamp_h *= (preScaler + 1);

            //printf("timeStamp_l: %llu, timeStamp_h: %llu\n", timeStamp_l, timeStamp_h);
            timeStamp_l *= scalingFactor;
            timeStamp_h *= scalingFactor;

            //printf("timeStamp_l: %llu, timeStamp_h: %llu\n", timeStamp_l, timeStamp_h);
            //timeStamp_l /= freq;
            //timeStamp_h /= freq;

            //printf("timeStamp_l: %llu, timeStamp_h: %llu\n", timeStamp_l, timeStamp_h);
            timeStamp = (uint64_t) ((((timeStamp_h << 32) & ~UINT32_MAX) | timeStamp_l) / freq);
            //timeStamp /= freq;

            //printf("timeStamp: %llu\n", timeStamp);
            scaledTime = (timeStamp > UINT64_MAX) ? UINT64_MAX : (uint64_t) timeStamp;
            //printf("scaledTime: %llu\n", scaledTime);
        }
    }

Exit:
    return scaledTime;
}

static inline uint64_t getTimerMaxScaledCount(ALT_GPT_TIMER_t tmr_id, uint32_t scalingFactor)
{
    uint64_t        maxScaledTime = 0;
    uint32_t        freq = 1;
    uint64_t        maxTimeStamp_l = 0;
    uint64_t        maxTimeStamp_h = 0;
    uint64_t        maxTimeStamp = 0;
    uint32_t        preScaler = 0;
    ALT_CLK_t       clkSrc;

    preScaler = alt_gpt_prescaler_get(tmr_id);

    if (tmr_id == ALT_GPT_CPU_GLOBAL_TMR)
    {
        clkSrc = ALT_CLK_MPU_PERIPH;
        //maxTimeStamp = UINT64_MAX;
        maxTimeStamp_l = (uint64_t) UINT32_MAX;
        maxTimeStamp_h = (uint64_t) UINT32_MAX;
    }
    else
    {
        goto Exit;
    }

    if (alt_clk_freq_get(clkSrc, &freq) == ALT_E_SUCCESS)
    {
        //        maxTimeStamp /= (uint64_t) freq;
        //        maxTimeStamp *= ((uint64_t) (alt_gpt_prescaler_get(tmr_id) + 1));
        //        maxTimeStamp *= (uint64_t) scalingFactor;                 //scale the output
        //        maxScaledTime = (maxTimeStamp > UINT64_MAX) ? UINT64_MAX : (uint64_t) maxTimeStamp;

        //printf("maxTimeStamp_l: %llu, maxTimeStamp_h: %llu\n", maxTimeStamp_l, maxTimeStamp_h);
        maxTimeStamp_l *= (preScaler + 1);
        maxTimeStamp_h *= (preScaler + 1);

        //printf("maxTimeStamp_l: %llu, maxTimeStamp_h: %llu\n", maxTimeStamp_l, maxTimeStamp_h);
        maxTimeStamp_l *= scalingFactor;                    //scale the output
        maxTimeStamp_h *= scalingFactor;                    //scale the output

        //printf("maxTimeStamp_l: %llu, maxTimeStamp_h: %llu\n", maxTimeStamp_l, maxTimeStamp_h);
        //maxTimeStamp_l /= freq;
        //maxTimeStamp_h /= freq;

        //printf("maxTimeStamp_l: %llu, maxTimeStamp_h: %llu\n", maxTimeStamp_l, maxTimeStamp_h);
        maxTimeStamp = (uint64_t) ((((maxTimeStamp_h << 32) & ~UINT32_MAX) | maxTimeStamp_l) / freq);
        //maxTimeStamp_h /= freq;

        //printf("maxTimeStamp: %llu\n", maxTimeStamp);
        maxScaledTime = (maxTimeStamp > UINT64_MAX) ? UINT64_MAX : (uint64_t) maxTimeStamp;
        //printf("maxScaledTime: %llu\n", maxScaledTime);
    }

Exit:
    return maxScaledTime;
}


//------------------------------------------------------------------------------
/**
\brief  Set IP address of specified Ethernet interface

The function sets the IP address, subnetMask and MTU of an Ethernet
interface.

\param  ifName_p                Name of Ethernet interface.
\param  ipAddress_p             IP address to set for interface.
\param  subnetMask_p            Subnet mask to set for interface.
\param  mtu_p                   MTU to set for interface.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setIpAdrs(char* ifName_p, UINT32 ipAddress_p, UINT32 subnetMask_p,
                            UINT16 mtu_p)
{
    UNUSED_PARAMETER(ifName_p);
    UNUSED_PARAMETER(ipAddress_p);
    UNUSED_PARAMETER(subnetMask_p);
    UNUSED_PARAMETER(mtu_p);

    //Note: The given parameters are ignored because the application must set
    //      these settings to the used IP stack by itself!

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set default gateway for Ethernet interface

The function sets the default gateway of an Ethernet interface.

\param  defaultGateway_p            Default gateway to set.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setDefaultGateway(UINT32 defaultGateway_p)
{
    UNUSED_PARAMETER(defaultGateway_p);

    //Note: The given parameters are ignored because the application must set
    //      these settings to the used IP stack by itself!

    return kErrorOk;
}

///\}
