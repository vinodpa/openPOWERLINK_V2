/**
********************************************************************************
\file   ctrlk.c

\brief  Implementation of kernel control module

This file contains the implementation of the kernel control module. The kernel
control module is responsible to execute commands from the user part of the
stack. Additionally, it provides status information to the user part.

\ingroup module_ctrlk
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>
#include <kernel/dllk.h>
#include <kernel/dllkcal.h>
#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <kernel/errhndk.h>
#include <kernel/nmtk.h>
#include <kernel/pdok.h>

#if defined(CONFIG_INCLUDE_VETH)
#include <kernel/veth.h>
#endif

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
/**
\brief  Instance of kernel control module

The structure specifies the instance variable of the kernel control module.
*/
typedef struct
{
    tCtrlInitParam      initParam;          ///< Initialization parameters
    UINT16              heartbeat;          ///< Heartbeat counter
    UINT32              features;           ///< Features provided by the kernel stack
} tCtrlkInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrlkInstance   instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initStack(void);
static tOplkError shutdownStack(void);
static void setupKernelFeatures(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the kernel control module

The function initializes the kernel control module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
tOplkError ctrlk_init(void)
{
    tOplkError      ret = kErrorOk;
    if ((ret = ctrlkcal_init()) != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("ctrlkcal_init failed!\n");
        goto ExitCleanup;
    }

    // initialize heartbeat counter
    instance_l.heartbeat = 1;

    setupKernelFeatures();

    return kErrorOk;

ExitCleanup:
    ctrlkcal_exit();
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup kernel control module

The function cleans up the kernel control module.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
void ctrlk_exit(void)
{
    ctrlkcal_exit();
}

//------------------------------------------------------------------------------
/**
\brief  Process function of kernel control module

This function implements the main function of the control module. It processes
commands from the user part of the stack and executes them.

\return Returns the exit flag.
\retval TRUE    Kernel stack should exit.
\retval FALSE   Kernel stack should continue running.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
BOOL ctrlk_process(void)
{
    tOplkError          ret = kErrorOk;
    UINT16              fRet;
    UINT16              status;
    tCtrlCmdType        cmd = kCtrlNone;
    BOOL                fExit = FALSE;

    if (ctrlkcal_getCmd(&cmd) != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s: error getting command!\n", __func__);
        return FALSE;
    }

    if (cmd != kCtrlNone)
    {
        ret = ctrlk_executeCmd(cmd, &fRet, &status, &fExit);
        if (ret == kErrorOk)
        {
            ctrlkcal_sendReturn(fRet);
            if (status != kCtrlStatusUnchanged)
                ctrlkcal_setStatus(status);
        }
    }

    eventkcal_process();

    ret = ctrlkcal_process();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s: CAL process returned with 0x%X\n", __func__, ret);
        fExit = TRUE;
    }

    return fExit;
}

//------------------------------------------------------------------------------
/**
\brief  Execute a control command

The function executes a control command from the user part of the stack.
The return value of the executed function will be stored a \p pRet_p. If the
pointer to the status and exit flag is not NULL the appropriate data is stored.

\param  cmd_p               The command to be executed.
\param  pRet_p              Pointer to store the return value.
\param  pStatus_p           Pointer to store the kernel stack status. (if not NULL)
\param  pfExit_p            Pointer to store the exit flag. (if not NULL)

\return The function returns a tOplkError error code.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
tOplkError ctrlk_executeCmd(tCtrlCmdType cmd_p, UINT16* pRet_p, UINT16* pStatus_p,
                            BOOL* pfExit_p)
{
    tOplkError          ret = kErrorOk;
    UINT16              status;
    BOOL                fExit;
    tOplkError          retVal;

    switch (cmd_p)
    {
        case kCtrlInitStack:
            DEBUG_LVL_CTRL_TRACE("Initialize kernel modules...\n");
<<<<<<< HEAD
            retVal = initStack();
            *pRet_p = (UINT16)retVal;
=======
            *pRet_p = initStack();
            printf("Initialize kernel modules...\n");
>>>>>>> 765f179... MN and CN Operational
            status = kCtrlStatusRunning;
            fExit = FALSE;
            break;

        case kCtrlCleanupStack:
            DEBUG_LVL_CTRL_TRACE("Shutdown kernel modules...\n");
<<<<<<< HEAD
            retVal = shutdownStack();
            *pRet_p = (UINT16)retVal;
=======
            *pRet_p = shutdownStack();
            printf("Shutdown kernel modules...\n");
>>>>>>> 765f179... MN and CN Operational
            status = kCtrlStatusReady;
            fExit = FALSE;
            break;

        case kCtrlShutdown:
            DEBUG_LVL_CTRL_TRACE("Shutdown kernel stack...\n");
            retVal = shutdownStack();
            *pRet_p = (UINT16)retVal;
            status = kCtrlStatusUnavailable;
            fExit = TRUE;
            break;

        case kCtrlGetVersionHigh:
            *pRet_p = (UINT16)(PLK_DEFINED_STACK_VERSION >> 16);
            status = kCtrlStatusUnchanged;
            fExit = FALSE;
            break;

        case kCtrlGetVersionLow:
            *pRet_p = (UINT16)(PLK_DEFINED_STACK_VERSION & 0xFFFF);
            status = kCtrlStatusUnchanged;
            fExit = FALSE;
            break;

        case kCtrlGetFeaturesHigh:
            *pRet_p = (UINT16)(instance_l.features >> 16);
            status = kCtrlStatusUnchanged;
            fExit = FALSE;
            break;

        case kCtrlGetFeaturesLow:
            *pRet_p = (UINT16)(instance_l.features & 0xFFFF);
            status = kCtrlStatusUnchanged;
            fExit = FALSE;
            break;

        default:
            DEBUG_LVL_ERROR_TRACE("%s() Unknown command %d\n", __func__, cmd_p);
            ret = kErrorGeneralError;
            status = kCtrlStatusUnavailable;
            fExit = TRUE;
            break;
    }

    if (pStatus_p != NULL)
    {
        *pStatus_p = status;
    }

    if (pfExit_p != NULL)
    {
        *pfExit_p = fExit;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update heartbeat counter

The function updates the heartbeat counter of the kernel stack which can be
used by the user stack to detect if the kernel stack is still running. It has
to be called periodically, e.g. from a timer routine.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
void ctrlk_updateHeartbeat(void)
{
    UINT16      heartbeat;

    heartbeat = instance_l.heartbeat++;
    ctrlkcal_updateHeartbeat(heartbeat);
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat counter

The function returns the heartbeat counter.

\return     Heartbeat counter.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
UINT16 ctrlk_getHeartbeat(void)
{
    return instance_l.heartbeat;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Initialize the kernel stack modules

The function initializes the kernel stack modules.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initStack(void)
{
    tOplkError          ret;
    tDllkInitParam      dllkInitParam;

    //printf("1.1\n");
    ctrlkcal_readInitParam(&instance_l.initParam);
    //printf("1.2\n");
    if ((ret = eventk_init()) != kErrorOk)
        return ret;
    //printf("1.3\n");
    if ((ret = nmtk_init()) != kErrorOk)
        return ret;
    //printf("1.4\n");
    OPLK_MEMCPY(dllkInitParam.aLocalMac, instance_l.initParam.aMacAddress, 6);
    dllkInitParam.hwParam.pDevName = instance_l.initParam.szEthDevName;
    dllkInitParam.hwParam.devNum = instance_l.initParam.ethDevNumber;
    //printf("1.5\n");
    ret = dllk_addInstance(&dllkInitParam);
    if (ret != kErrorOk)
        return ret;
    //printf("1.6\n");
    // copy MAC address back to instance structure
    OPLK_MEMCPY(instance_l.initParam.aMacAddress, dllkInitParam.aLocalMac, 6);
    ctrlkcal_storeInitParam(&instance_l.initParam);
    //printf("1.7\n");
    dllk_regSyncHandler(pdok_sendSyncEvent);
    //printf("1.8\n");
    // initialize dllkcal module
    if ((ret = dllkcal_init()) != kErrorOk)
        return ret;
    //printf("1.9\n");
#if defined(CONFIG_INCLUDE_PDO)
    if ((ret = pdok_init()) != kErrorOk)
        return ret;
#endif
    //printf("1.10\n");
    // initialize Virtual Ethernet Driver
#if defined(CONFIG_INCLUDE_VETH)
    if ((ret = veth_addInstance(instance_l.initParam.aMacAddress)) != kErrorOk)
        return ret;
#endif
    //printf("1.11\n");
    ret = errhndk_init();
    //printf("1.12 %x\n",ret);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Clean up the kernel stack modules

The function cleans up the kernel stack modules.

\return Returns always kErrorOk.
*/
//------------------------------------------------------------------------------
static tOplkError shutdownStack(void)
{

#if defined(CONFIG_INCLUDE_VETH)
    veth_delInstance();
#endif

#if defined(CONFIG_INCLUDE_PDO)
    pdok_exit();
#endif

    nmtk_delInstance();

    dllk_delInstance();

    dllkcal_exit();

    eventk_exit();
    errhndk_exit();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Setup kernel features

The function sets up the features which are supported by the kernel stack.
*/
//------------------------------------------------------------------------------
void setupKernelFeatures(void)
{
    instance_l.features = 0;

#if defined(CONFIG_INCLUDE_NMT_MN)
    // We do have NMT functionality compiled in and therefore need an MN
    // kernel stack
    instance_l.features |= OPLK_KERNEL_MN;
#endif

#if defined(CONFIG_INCLUDE_PDO)
    // We contain the PDO module for isochronous transfers and therefore need
    // a kernel module which can handle isochronous transfers.
    instance_l.features |= OPLK_KERNEL_ISOCHR;
#endif

#if defined(CONFIG_INCLUDE_PRES_FORWARD)
    // We contain the PRES forwarding module (used for diagnosis) and therefore
    // need a kernel with this feature.
    instance_l.features |= OPLK_KERNEL_PRES_FORWARD;
#endif

#if defined(CONFIG_INCLUDE_VETH)
    // We contain the virtual Ethernet module and therefore need a kernel
    // which supports virtual Ethernet.
    instance_l.features |= OPLK_KERNEL_VETH;
#endif

#if (CONFIG_DLL_PRES_CHAINING_CN == TRUE)
    instance_l.features |= OPLK_KERNEL_PRES_CHAINING_CN;
#endif
}

/// \}
