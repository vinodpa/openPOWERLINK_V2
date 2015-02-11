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

#include <sys/unistd.h>
#include <alt_timers.h>
#include <alt_globaltmr.h>
#include <alt_interrupt.h>
#include <alt_cache.h>
#include <alt_fpga_manager.h>
#include <alt_bridge_manager.h>
#include <alt_address_space.h>
#include <alt_mpu_registers.h>
#include <alt_clock_manager.h>

#include <system.h>

#include <oplk/oplk.h>
#include <oplk/debug.h>
#include <common/target.h>
#include "sleep.h"
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef SYNC_IRQ
#define SYNC_IRQ    1
#endif
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
#define SECS_TO_MILLISECS                           1000

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// external vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static inline int       enableInterruptMaster(void);
static inline int       disableInterruptMaster(void);

static inline uint64_t  getTimerMaxScaledCount(ALT_GPT_TIMER_t timerId_p,
                                               uint32_t scalingFactor_p);
static inline uint64_t  getTimerCurrentScaledCount(ALT_GPT_TIMER_t timerId_p,
                                                   uint32_t scalingFactor_p);

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
            enableInterruptMaster();
        }
    }
    else
    {                       // disable interrupts
        if (lockCount == 0)
        {
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
tOplkError target_init(void)
{
    tOplkError          oplkRet = kErrorOk;
    ALT_STATUS_CODE     halRet = ALT_E_SUCCESS;

#ifdef ENABLE_CACHE
    // Enable Cache
    halRet = alt_cache_system_enable();
#else
    halRet = alt_cache_system_disable();
#endif

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
        DEBUG_LVL_ERROR_TRACE("global IRQ controller initialization Failed!!\n");
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

    // Initialize the cpu interrupt interface
    halRet = alt_int_cpu_init();
    if (halRet != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("CPU IRQ interface initialization Failed!!\n");
        oplkRet = kErrorGeneralError;
        goto Exit;
    }


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
    disableInterruptMaster();
    alt_int_cpu_uninit();
    alt_int_global_uninit();

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
    msleep(milliSeconds_p);
}

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
    ALT_INT_INTERRUPT_t    irqId = ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + SYNC_IRQ;

    if (callback_p == NULL)
    {
        alt_int_isr_unregister(irqId);
    }
    else
    {
        if ( alt_int_isr_register(irqId, callback_p, pArg_p) != ALT_E_SUCCESS)
        {
            DEBUG_LVL_ERROR_TRACE("Sync ISR registration failed\n");
        }
    }
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
    ALT_INT_INTERRUPT_t     irqId = ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + SYNC_IRQ;
    int                     cpu_target = 0x1;                                             // cortexA9_0

    if (fEnable_p)
    {
        alt_int_dist_pending_clear(irqId);

        if (alt_int_dist_target_set(irqId, cpu_target) != ALT_E_SUCCESS)
        {
            DEBUG_LVL_ERROR_TRACE("Sync IRQ target cpu set failed\n");
        }
        //else if (alt_int_dist_trigger_set(irqId, ALT_INT_TRIGGER_EDGE) != ALT_E_SUCCESS)
        else if (alt_int_dist_trigger_set(irqId, ALT_INT_TRIGGER_EDGE) != ALT_E_SUCCESS)
        {
            DEBUG_LVL_ERROR_TRACE("Sync IRQ trigger set failed\n");
        }
        else if (alt_int_dist_enable(irqId) != ALT_E_SUCCESS)
        {
            // Set interrupt distributor target
            DEBUG_LVL_ERROR_TRACE("Sync IRQ could not be enabled in the distributor\n");
        }
    }
    else
    {
        if (alt_int_dist_disable(irqId) == ALT_E_SUCCESS)
        {
            // access to any FPGA registers if required
            return;
        }
        else
        {
            DEBUG_LVL_ERROR_TRACE("Sync IRQ could not be disabled in the distributor\n");
        }
    }
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


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Enable the global interrupt master

The function enables interrupt reception in the global and target processor
interrupt interfaces

\return The function returns an integer
\retval 0                   Success
\retval -1                  Failure
*/
//------------------------------------------------------------------------------
static inline int enableInterruptMaster(void)
{
    ALT_STATUS_CODE     retStatus = ALT_E_SUCCESS;
    int                 ret = 0;

    // enable global interrupt master
    // Global interrupt enable
    retStatus = alt_int_global_enable();
    if (retStatus != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("enabling global interrupt receiver failed\n");
        ret = -1;
        goto Exit;
    }

    // CPU interface global enable
    retStatus = alt_int_cpu_enable();
    if (retStatus != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("enabling cpu interrupt receiver failed\n");
        ret = -1;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Disable the global interrupt master

The function disables interrupt reception in the global and target processor
interrupt interfaces

\return The function returns an integer
\retval 0                   Success
\retval -1                  Failure
*/
//------------------------------------------------------------------------------
static inline int disableInterruptMaster(void)
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

//------------------------------------------------------------------------------
/**
\brief Convert timer ticks into time units

The function returns the timestamp of the provided timer in standard time units

\param  timerId_p               The ALT_GPT_TIMER_t enum Id of the timer used
\param  scalingFactor_p         Ratio of provided time duration scale to seconds

\return The function returns a unsigned 64 bit value.
\retval Timestamp from the given timer in standard time unit provided
*/
//------------------------------------------------------------------------------
static inline uint64_t getTimerCurrentScaledCount(ALT_GPT_TIMER_t timerId_p,
                                                  uint32_t scalingFactor_p)
{
    uint64_t        timeStamp_l = 0;                // r2 & r3
    uint64_t        timeStamp_h = 0;
    uint64_t        timeStamp = 0;
    uint64_t        scaledTime = 0;                 // value to return
    ALT_CLK_t       clkSrc = ALT_CLK_UNKNOWN;
    uint32_t        preScaler = 0;
    uint32_t        freq = 1;

    preScaler = alt_gpt_prescaler_get(timerId_p);
    if (preScaler <= UINT8_MAX)
    {
        if (timerId_p == ALT_GPT_CPU_GLOBAL_TMR)       // Global Timer
        {
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
            timeStamp_l *= (preScaler + 1);
            timeStamp_h *= (preScaler + 1);
            timeStamp_l *= scalingFactor_p;
            timeStamp_h *= scalingFactor_p;
            timeStamp = (uint64_t) ((((timeStamp_h << 32) & ~UINT32_MAX)
                                     | timeStamp_l) / freq);
            scaledTime = (timeStamp > UINT64_MAX) ? UINT64_MAX : (uint64_t) timeStamp;
        }
    }

Exit:
    return scaledTime;
}

//------------------------------------------------------------------------------
/**
\brief Get maximum timestamp of the timer

The function returns the maximum timestamp of the provided timer
in standard time units

\param  timerId_p                The ALT_GPT_TIMER_t enum Id of the timer used
\param  scalingFactor_p          Ratio of provided time duration scale to seconds

\return The function returns a unsigned 64 bit value.
\retval Maximum timestamp from the given timer in standard time unit provided
*/
//------------------------------------------------------------------------------
static inline uint64_t getTimerMaxScaledCount(ALT_GPT_TIMER_t timerId_p,
                                              uint32_t scalingFactor_p)
{
    uint64_t        maxScaledTime = 0;
    uint32_t        freq = 1;
    uint64_t        maxTimeStamp_l = 0;
    uint64_t        maxTimeStamp_h = 0;
    uint64_t        maxTimeStamp = 0;
    uint32_t        preScaler = 0;
    ALT_CLK_t       clkSrc;

    preScaler = alt_gpt_prescaler_get(timerId_p);

    if (timerId_p == ALT_GPT_CPU_GLOBAL_TMR)
    {
        clkSrc = ALT_CLK_MPU_PERIPH;
        maxTimeStamp_l = (uint64_t) UINT32_MAX;
        maxTimeStamp_h = (uint64_t) UINT32_MAX;
    }
    else
    {
        goto Exit;
    }

    if (alt_clk_freq_get(clkSrc, &freq) == ALT_E_SUCCESS)
    {
        maxTimeStamp_l *= (preScaler + 1);
        maxTimeStamp_h *= (preScaler + 1);
        maxTimeStamp_l *= scalingFactor_p;                    //scale the output
        maxTimeStamp_h *= scalingFactor_p;                    //scale the output
        maxTimeStamp = (uint64_t) ((((maxTimeStamp_h << 32) & ~UINT32_MAX)
                                   | maxTimeStamp_l) / freq);

        maxScaledTime = (maxTimeStamp > UINT64_MAX) ? UINT64_MAX : (uint64_t) maxTimeStamp;
    }

Exit:
    return maxScaledTime;
}

///\}
