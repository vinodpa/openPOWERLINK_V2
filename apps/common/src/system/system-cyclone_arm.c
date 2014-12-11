/**
********************************************************************************
\file   system-cyclone_arm.c

\brief  System specific functions for Altera Cyclone-V ARM

The file implements the system specific functions for ARM on Altera Cyclone-V
used by the openPOWERLINK demo applications.

\ingroup module_app_common
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2014, Kalycito Infotech Private Ltd.
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
#include <system/system.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define SECS_TO_MILLISECS                           1000
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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static int              initializeFpga(void);
static int              initializeTimer(void);
static int              cleanupTimer(void);

static inline uint64_t  getTimerTicksFromScaled(ALT_GPT_TIMER_t timerId_p,
                                                uint32_t scalingFactor_p,
                                                uint32_t scaledTimeDuration_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize system

The function initializes important stuff on the system for openPOWERLINK to
work correctly.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
int system_init(void)
{
    tOplkError          oplkRet = kErrorOk;

    if (initializeTimer() != 0)
    {
        DEBUG_LVL_ERROR_TRACE("general purpose timer module initialization Failed!!\n");
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

    // Initialize the HPS to FPGA bridges alone and not initializeFpga()
    if (alt_bridge_init(ALT_BRIDGE_LWH2F, NULL, NULL) != ALT_E_SUCCESS)
    {
        oplkRet = kErrorGeneralError;
        DEBUG_LVL_ERROR_TRACE("LWH2F initialization Failed!!\n");
        goto Exit;
    }

    if (alt_bridge_init(ALT_BRIDGE_H2F, NULL, NULL) != ALT_E_SUCCESS)
    {
        oplkRet = kErrorGeneralError;
        DEBUG_LVL_ERROR_TRACE("H2F initialization Failed!!\n");
        goto Exit;
    }

Exit:
    return oplkRet;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown system

The function shuts-down the system.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_exit(void)
{
    alt_bridge_uninit(ALT_BRIDGE_H2F, NULL, NULL);
    alt_bridge_uninit(ALT_BRIDGE_LWH2F, NULL, NULL);
    cleanupTimer();
}

#if defined(CONFIG_USE_SYNCTHREAD)
//------------------------------------------------------------------------------
/**
\brief  Start synchronous data thread

The function starts the thread used for synchronous data handling.

\param  pfnSync_p           Pointer to sync callback function

\note   Currently not implemented for Windows!

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_startSyncThread(tSyncCb pfnSync_p)
{
    UNUSED_PARAMETER(pfnSync_p);
}

//------------------------------------------------------------------------------
/**
\brief  Stop synchronous data thread

The function stops the thread used for synchronous data handling.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_stopSyncThread(void)
{
    syncThreadInstance_l.fTerminate = TRUE;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Return true if a termination signal has been received

The function can be used by the application to react on termination request.
On Windows, this function only implemented as a stub.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
BOOL system_getTermSignalState(void)
{
    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param  milliSeconds_p      Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void system_msleep(unsigned int milliSeconds_p)
{
    uint64_t                startTickStamp = alt_globaltmr_get64();
    uint64_t                waitTickCount = getTimerTicksFromScaled(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS, milliSeconds_p);
    volatile uint32_t*      glbTimerRegCntBase_l = (volatile uint32_t*) (GLOBALTMR_BASE + GLOBALTMR_CNTR_LO_REG_OFFSET);
    volatile uint32_t*      glbTimerRegCntBase_h = (volatile uint32_t*) (GLOBALTMR_BASE + GLOBALTMR_CNTR_HI_REG_OFFSET);
    uint64_t                curTickStamp = 0;
    uint32_t                temp = 0;
    uint32_t                hi = 0;
    uint32_t                lo = 0;
    volatile uint32_t       waitCount = 0;
    uint32_t                waitCountLimit = (uint32_t) (5 + (waitTickCount >> 12));
    uint8_t                 fExit = FALSE;
    uint8_t                 readCntLimit = 3;

    curTickStamp = alt_globaltmr_get64();

    while (fExit != TRUE)
    {
        fExit = FALSE;

        if (waitCount == waitCountLimit)
        {
            readCntLimit = 3;

            do
            {
                temp = (*glbTimerRegCntBase_h);
                lo =  (*glbTimerRegCntBase_l);
                hi = (*glbTimerRegCntBase_h);
            } while ((temp != hi) && (--readCntLimit));

            if (readCntLimit != 0)
            {
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
                    fExit = TRUE;
                }

                waitCount = 1;
            }
            else
            {
                waitCount--;
            }
        }
//
//        if (waitCount % 5000 == 0)
//        printf(".");
        waitCount++;
    }

//    printf("\n");
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{
//------------------------------------------------------------------------------
/**
\brief Initialize the FPGA from the hard processor system

The function initializes the FPGA, manager interface, data bridges and
configures the FPGA for operation

\return The function returns an integer
\retval 0                   Success
\retval -1                  Failure
*/
//------------------------------------------------------------------------------
static int initializeFpga(void)
{
    int                 ret = 0;
    ALT_STATUS_CODE     halRet = ALT_E_SUCCESS;

    /* initialize the FPGA control */
    if (alt_fpga_init() != ALT_E_SUCCESS)                       // initialize the FPGA manager
    {
        DEBUG_LVL_ERROR_TRACE("FPGA interface initialization Failed!!\n");
        ret = -1;
        goto Exit;
    }
    else if (alt_fpga_state_get() == ALT_FPGA_STATE_POWER_OFF)  // check the FPGA state
    {
        DEBUG_LVL_ERROR_TRACE("FPGA is powered Off!!\n");
        ret = -1;
        goto Exit;
    }
    else if (!alt_fpga_control_is_enabled())                    // check if CPU has the control of the FPGA control block
    {           // if not acquire control
        halRet = alt_fpga_control_enable();
    }

    if (halRet != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("FPGA interface control could not be acquired\n");
        ret = -1;
        goto Exit;
    }

    /* Program FPGA here if required */

    /* Enable the HPS-FPGA bridge */
    if (alt_bridge_init(ALT_BRIDGE_F2H, NULL, NULL) != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("F2H initialization Failed!!\n");
        ret = -1;
        goto Exit;
    }
    else if (alt_bridge_init(ALT_BRIDGE_H2F, NULL, NULL) != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("H2F initialization Failed!!\n");
        ret = -1;
        goto Exit;
    }
    else if (alt_bridge_init(ALT_BRIDGE_LWH2F, NULL, NULL) != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("LWH2F initialization Failed!!\n");
        ret = -1;
        goto Exit;
    }

    if (alt_addr_space_remap(ALT_ADDR_SPACE_MPU_ZERO_AT_BOOTROM,
                             ALT_ADDR_SPACE_NONMPU_ZERO_AT_OCRAM,
                             ALT_ADDR_SPACE_H2F_ACCESSIBLE,
                             ALT_ADDR_SPACE_LWH2F_ACCESSIBLE) !=
        ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("FPGA address space remapping Failed!!\n");
        ret = -1;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Initialize the timer module

The function initializes the global timer and configures it to be used as
the user stack generic timer

\return The function returns an integer
\retval 0                   Success
\retval -1                  Failure
*/
//------------------------------------------------------------------------------
static int initializeTimer(void)
{
    int                 ret = 0;
    ALT_STATUS_CODE     halRet = ALT_E_SUCCESS;

    // initialize timer, only the 64 bit global timer is used
    halRet =  alt_globaltmr_init();
    if (halRet != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("general purpose timer module intialization Failed!!\n");
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

//------------------------------------------------------------------------------
/**
\brief Uninitialize the timer module

The function uninitializes the global timer and configures it to be used as
the user stack generic timer.

\return The function returns an integer
\retval 0                   Success
\retval -1                  Failure
*/
//------------------------------------------------------------------------------
static int cleanupTimer(void)
{
    int                 ret = 0;
    ALT_STATUS_CODE     halRet = ALT_E_SUCCESS;

    halRet = alt_globaltmr_stop();

    if (halRet == ALT_E_SUCCESS)
    {
        halRet = alt_globaltmr_uninit();
    }

    if (halRet != ALT_E_SUCCESS)
        ret = -1;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Convert time units into timer ticks

The function converts the time in standard unit into ticks for the
given timer.

\param  timerId_p               The ALT_GPT_TIMER_t enum Id of the timer used
\param  scalingFactor_p         Ratio of provided time duration scale to seconds
\param  scaledTimeDuration_p    Time duration in standard unit to be converted

\return The function returns a unsigned 64 bit value.
\retval The converted tick count for the given timer.
*/
//------------------------------------------------------------------------------
static inline uint64_t getTimerTicksFromScaled(ALT_GPT_TIMER_t timerId_p,
                                               uint32_t scalingFactor_p,
                                               uint32_t scaledTimeDuration_p)
{
    uint64_t        ticks = 0;                      // value to return
    ALT_CLK_t       clkSrc = ALT_CLK_UNKNOWN;
    uint32_t        preScaler = 0;
    uint32_t        freq = 1;

    preScaler = alt_gpt_prescaler_get(timerId_p);
    if (preScaler <= UINT8_MAX)
    {
        if (timerId_p == ALT_GPT_CPU_GLOBAL_TMR)       // Global Timer
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
            // clock ticks per second
            ticks *= freq;

            // total clock ticks
            ticks *= (uint64_t) (scaledTimeDuration_p / scalingFactor_p);

            // convert into timer ticks
            ticks /= (preScaler + 1);
        }
    }

Exit:
    return ticks;
}

///\}
