/**
********************************************************************************
\file   hostiflibint_arm.c

\brief  Host Interface Library - Driver Implementation for ARM

The file contains the high level driver for the host interface library for
ARM targets.

\ingroup module_hostiflib
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

/**
********************************************************************************
\defgroup   module_hostiflib    Host Interface Library
\ingroup    libraries

The host interface library provides a software interface for using the host
interface IP-Core. It provides several features like queues and linear memory
modules.
*******************************************************************************/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "hostiflib.h"
#include "hostiflib_target.h"
#include "hostiflibint.h"

#include <alt_interrupt.h>
#include <alt_timers.h>

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
static tHostifIrqCb    pfnIrqCb_l = NULL;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Register interrupt service routine

This function registers the interrupt service routine in the host processor
interrupt services.

\param  pfnIrqCb_p              The interrupt service routine callback
\param  pArg_p                  Argument pointer provided to the callback

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       The interrupt service routine was registered
                                successfully.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------

void __cs3_isr(uint32_t icciar_p, void* context_p)
{
    alt_int_dist_pending_clear(ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + HOSTIF_IRQ);
    if (pfnIrqCb_l != NULL)
        pfnIrqCb_l(context_p);
    return;
}

tHostifReturn hostif_sysIrqRegHandler(tHostifIrqCb pfnIrqCb_p, void* pArg_p)
{
    ALT_INT_INTERRUPT_t    irqId = ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + HOSTIF_IRQ;

    if (pfnIrqCb_p == NULL)
    {
        printf("UnReg Hostif ISR\n");
        alt_int_isr_unregister(irqId);
    }
    else
    {
        printf("Reg Hostif ISR\n");
        if ( alt_int_isr_register(irqId, __cs3_isr, pArg_p) != ALT_E_SUCCESS)
        {
            return kHostifUnspecError;
        }
        else
        {
            pfnIrqCb_l = pfnIrqCb_p;
            return kHostifSuccessful;
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Enable interrupt for host interface driver

This function enables the interrupt for the host interface driver instance.

\param  fEnable_p   Determines if the interrupt must be enabled (TRUE) or
                    disabled (FALSE)

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       Interrupt enable/disable failed

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_sysIrqEnable(BOOL fEnable_p)
{
    ALT_STATUS_CODE         ret;
    ALT_INT_INTERRUPT_t     irqId = ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + HOSTIF_IRQ;
    int                     cpu_target = 0x1;                                             // cortexA9_0

    printf("Enable Hostif IRQ: %X(%X)\n", fEnable_p,irqId);
    if (fEnable_p)
    {
        if (alt_int_dist_trigger_set(irqId, ALT_INT_TRIGGER_LEVEL) != ALT_E_SUCCESS)
        {
            printf("trigger set failed\n");
            return kHostifNoResource;
        }
        else if (alt_int_dist_target_set(irqId, cpu_target) != ALT_E_SUCCESS)
        {
            printf("trigger set failed\n");
            return kHostifNoResource;
        }
        else if (alt_int_dist_enable(irqId) != ALT_E_SUCCESS)
        {
            // Set interrupt distributor target
            printf("distributor set failed\n");
            return kHostifNoResource;
        }
        else
            return kHostifSuccessful;
    }
    else
    {
        if (alt_int_dist_disable(irqId) == ALT_E_SUCCESS)
            return kHostifSuccessful;
        else
            return kHostifNoResource;
    }
}

//void hostif_msleep(UINT32 milliSeconds_p)
//{
//	uint32_t			msStartTime = alt_gpt_time_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR);
//	uint8_t				fExit = FALSE;
//
//	while (!fExit)
//	{
//		if ((((alt_gpt_time_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR) > msStartTime) &&
//			(alt_gpt_time_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR)	- msStartTime) <= milliSeconds_p))
//			|| ((alt_gpt_time_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR) < msStartTime) &&
//					(alt_gpt_time_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR) +
//							alt_gpt_maxtime_millisecs_get(ALT_GPT_CPU_GLOBAL_TMR)
//							- msStartTime) <= milliSeconds_p))
//		{
//			// wait
//		}
//		else
//		{
//			fExit = TRUE;
//		}
//	}
//}

void hostif_msleep(UINT32 milliSeconds_p)
{
    target_msleep(milliSeconds_p);
}
