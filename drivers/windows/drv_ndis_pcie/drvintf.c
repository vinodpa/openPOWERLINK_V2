/**
********************************************************************************
\file   drvintf.c

\brief  Interface module for application interface to kernel daemon in Windows

// TODO: Add description

\ingroup module_driver_ndispcie
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

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <errhndkcal.h>
#include <dualprocshm.h>
#include <common/circbuffer.h>
#include <drvintf.h>
#include <ndis.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PROC_ID                         0xFA
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
#define DUALPROCSHM_BUFF_ID_ERRHDLR     12
#define DUALPROCSHM_BUFF_ID_PDO         13

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CMD_TIMEOUT_CNT     500     // loop counter for command timeout
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief PDO memory information

This structure stores the information for the PDO memory shared between user and
kernel space.
*/
typedef struct
{
    PMDL      pMdl;                 ///< Memory descriptor list describing the PDO memory.
    size_t    memSize;              ///< SIze of PDO memory
    void*     pKernelVa;            ///< Pointer to PDO memory in kernel space.
    void*     pUserVa;              ///< Pointer to PDO memory mapped in user space.
}tPdoMemInfo;

/**
\brief Control module instance - User Layer

The control module instance stores the local parameters used by the
control CAL module during runtime
*/
typedef struct
{
    tDualprocDrvInstance dualProcDrvInst;               ///< Dual processor driver instance
    BOOL                 fIrqMasterEnable;              ///< Master interrupts status
    tCircBufInstance*    eventQueueInst[NR_OF_CIRC_BUFFERS];
    tCircBufInstance*    dllQueueInst[NR_OF_CIRC_BUFFERS];
    tErrHndObjects*      pErrorObjects;
    tPdoMemInfo          pdoMem;
    BOOL                 fDriverActive;
}tDriverInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDriverInstance     drvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError   initEvent(void);
static tOplkError   initDllQueues(void);
static tOplkError   initErrHndl(void);
static void         exitEvent(void);
static void         exitDllQueues(void);
static void         exitErrHndl(void);
static tOplkError   insertDataBlock(tCircBufInstance* pDllCircBuffInst_p,
                                    BYTE* pData_p, UINT* pDataSize_p);


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Execute a control command from user application

This function parsed the control command from user and passes it to the control
module for processing. The return value is again passed to user by copying it
into the common control structure.

\param  ctrlCmd_p       Pointer to control command structure.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_executeCmd(tCtrlCmd* pCtrlCmd_p)
{
    tOplkError          ret;
    UINT16              status;
    UINT16              cmd = pCtrlCmd_p->cmd;
    int                 timeout;

    if (dualprocshm_writeDataCommon(drvInstance_l.dualProcDrvInst, FIELD_OFFSET(tCtrlBuf, ctrlCmd),
        sizeof(tCtrlCmd), (UINT8 *) pCtrlCmd_p) != kDualprocSuccessful)
        return;

    // wait for response
    for (timeout = 0; timeout < CMD_TIMEOUT_CNT; timeout++)
    {
        target_msleep(10);

        if (dualprocshm_readDataCommon(drvInstance_l.dualProcDrvInst, FIELD_OFFSET(tCtrlBuf, ctrlCmd),
            sizeof(tCtrlCmd), (UINT8 *) pCtrlCmd_p) != kDualprocSuccessful)
            return;

        if (pCtrlCmd_p->cmd == 0)
            break;
    }

    if (cmd == kCtrlInitStack && pCtrlCmd_p->retVal == kErrorOk)
    {
        ret = initEvent();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Event Initialization Failed %x\n", ret);
            pCtrlCmd_p->retVal = ret;
            return;
        }

        ret = initErrHndl();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Error Module Initialization Failed %x\n", ret);
            pCtrlCmd_p->retVal = ret;
            return;
        }

        ret = initDllQueues();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Dll Queues Initialization Failed %x\n", ret);
            pCtrlCmd_p->retVal = ret;
            return;
        }
    }

    if (cmd == kCtrlShutdown)
    {
        exitDllQueues();
        exitErrHndl();
        exitEvent();
    }

}

//------------------------------------------------------------------------------
/**
\brief  Read initialization parameters

Read the initialization parameters from the kernel stack.

\param  pInitParam_p       Pointer to initialization parameters structure.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_readInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn    dualRet;

    if (!drvInstance_l.fDriverActive)
        return;
    dualRet = dualprocshm_readDataCommon(drvInstance_l.dualProcDrvInst, FIELD_OFFSET(tCtrlBuf, initParam),
                                         sizeof(tCtrlInitParam), (UINT8*) pInitParam_p);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot read initparam (0x%X)\n", dualRet);
        return;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Write initialization parameters

Write the initialization parameters from the user into kernel memory.

\param  pInitParam_p       Pointer to initialization parameters structure.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    if (!drvInstance_l.fDriverActive)
        return;

    dualprocshm_writeDataCommon(drvInstance_l.dualProcDrvInst, FIELD_OFFSET(tCtrlBuf, initParam),
                                sizeof(tCtrlInitParam), (UINT8*) pInitParam_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get kernel status

Return the current status of kernel stack.

\param  pStatus_p       Pointer to status variable to return.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_getStatus(UINT16* pStatus_p)
{
    if (!drvInstance_l.fDriverActive)
        return;

    if (dualprocshm_readDataCommon(drvInstance_l.dualProcDrvInst, FIELD_OFFSET(tCtrlBuf, status),
        sizeof(UINT16), (UINT8*) pStatus_p) != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Error Reading Status\n");
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat

Return the current heartbeat value in kernel.

\param  pHeartbeat       Pointer to heartbeat variable to return.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_getHeartbeat(UINT16* pHeartbeat)
{
    if (!drvInstance_l.fDriverActive)
        return;

    if (dualprocshm_readDataCommon(drvInstance_l.dualProcDrvInst, FIELD_OFFSET(tCtrlBuf, heartbeat),
        sizeof(UINT16), (UINT8*) pHeartbeat) != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s()Error Reading HeartBeat\n");
    }
}

//------------------------------------------------------------------------------
/**
\brief  Write asynchronous frame

This routines extracts the azynchronous frame from the IOCTL buffer and passes it
to DLL for processing.

\param  pArg_p       Pointer to IOCTL buffer.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_sendAsyncFrame(unsigned char* pArg_p)
{
    tIoctlDllCalAsync*    asyncFrameInfo;
    tFrameInfo            frameInfo;
    tOplkError            ret;

    if (!drvInstance_l.fDriverActive)
        return;

    asyncFrameInfo = (tIoctlDllCalAsync*) pArg_p;
    frameInfo.frameSize = asyncFrameInfo->size;
    frameInfo.pFrame = (tPlkFrame*) (pArg_p + sizeof(tIoctlDllCalAsync));

    ret = insertDataBlock(drvInstance_l.dllQueueInst[asyncFrameInfo->queue],
                          (UINT8*) frameInfo.pFrame,
                          &(frameInfo.frameSize));

    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error Sending ASync Frame Queue %d\n", asyncFrameInfo->queue);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Write error object

This routines updates the error objects in kernel with the value passed by user.

\param  pWriteObject_p       Pointer to writeobject to update.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_writeErrorObject(tErrHndIoctl* pWriteObject_p)
{
    tErrHndObjects*   errorObjects = drvInstance_l.pErrorObjects;
    if (!drvInstance_l.fDriverActive)
        return;

    *((UINT32*) ((char*) errorObjects + pWriteObject_p->offset)) = pWriteObject_p->errVal;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object

This routines fetches the error objects in kernel to be passed to user.

\param  pWriteObject_p       Pointer to pReadObject_p to fetch.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_readErrorObject(tErrHndIoctl* pReadObject_p)
{
    tErrHndObjects*   errorObjects = drvInstance_l.pErrorObjects;

    if (!drvInstance_l.fDriverActive)
        return;

    pReadObject_p->errVal = *((UINT32*)((char*) errorObjects + pReadObject_p->offset));
}

//------------------------------------------------------------------------------
/**
\brief  Initialize dual processor shared memory driver instance

This routine initializes the driver instance of dualprocshm for HOST processor.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_initDualProcDrv(void)
{
    tDualprocReturn dualRet;
    tDualprocConfig dualProcConfig;

    TRACE(" Initialize Driver interface...");
    OPLK_MEMSET(&drvInstance_l, 0, sizeof(tDriverInstance));

    OPLK_MEMSET(&dualProcConfig, 0, sizeof(tDualprocConfig));

    dualProcConfig.procInstance = kDualProcSecond;
    dualProcConfig.procId = PROC_ID;

    dualRet = dualprocshm_create(&dualProcConfig, &drvInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE(" {%s} Could not create dual processor driver instance (0x%X)\n",
                              __func__, dualRet);
        dualprocshm_delete(drvInstance_l.dualProcDrvInst);
        return kErrorNoResource;
    }

    // Disable the Interrupts from PCP
    drvInstance_l.fIrqMasterEnable = FALSE;

    dualRet = dualprocshm_initInterrupts(drvInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("{%s} Error Initializing interrupts %x\n ", __func__, dualRet);
        return kErrorNoResource;
    }

    drvInstance_l.fDriverActive = TRUE;
    TRACE(" OK\n");
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Delete dual processor shared memory driver instance

This routine Deletes the driver instance of dualprocshm created during
initialization.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_exitDualProcDrv(void)
{
    tDualprocReturn dualRet;

    drvInstance_l.fIrqMasterEnable = FALSE;
    drvInstance_l.fDriverActive = FALSE;
    // disable system irq
    dualprocshm_freeInterrupts(drvInstance_l.dualProcDrvInst);

    dualRet = dualprocshm_delete(drvInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not delete dual proc driver inst (0x%X)\n", dualRet);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Post an user event

Posts the user to kernel event into the user to kernel event queue.

\param  pEvent_p    Pointer to event memory.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_postEvent(void* pEvent_p)
{
    tOplkError              ret = kErrorOk;
    tCircBufError           circError;
    tEvent                  event;
    char*                   pArg = NULL;

    tCircBufInstance*   pCircBufInstance = drvInstance_l.eventQueueInst[kEventQueueU2K];

    if (!drvInstance_l.fDriverActive)
        return;

    OPLK_MEMCPY(&event, pEvent_p, sizeof(tEvent));

    if (event.eventArgSize != 0)
    {
        pArg = (char*) ((UINT8*) pEvent_p + sizeof(tEvent));
        event.pEventArg = (ULONGLONG)pArg;
    }

    if (event.eventArgSize == 0)
    {
        circError = circbuf_writeData(pCircBufInstance, &event, sizeof(tEvent));
    }
    else
    {
        circError = circbuf_writeMultipleData(pCircBufInstance, pEvent_p, sizeof(tEvent),
                                              (void*)event.pEventArg, event.eventArgSize);
    }

    if (circError != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error in Post event %x\n", circError);
        ret = kErrorEventPostError;
    }

}

//------------------------------------------------------------------------------
/**
\brief  Get an user event

Retreives an event from kernel to user event queue.

\param  pEvent_p    Pointer to event memory.
\param  pSize_p     Size of the event buffer.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_getEvent(void* pEvent_p, size_t* pSize_p)
{
    tCircBufInstance*   pCircBufInstance = drvInstance_l.eventQueueInst[kEventQueueK2U];
    if (!drvInstance_l.fDriverActive)
        return;

    if (circbuf_getDataCount(pCircBufInstance) > 0)
    {
        circbuf_readData(pCircBufInstance, pEvent_p,
                         sizeof(tEvent) + MAX_EVENT_ARG_SIZE, pSize_p);
    }
    else
    {
        *pSize_p = 0;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get PDO memory

Retrieves the PDO memory from the dualprocshm library and maps it into user space
before sharing it to user application.

\param  ppPdoMem_p    Pointer to PDO memory.
\param  memSize_p     Size of the PDO memory.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getPdoMem(UINT8** ppPdoMem_p, size_t memSize_p)
{
    tDualprocReturn         dualRet;
    UINT8*                  pMem;
    tPdoMemInfo*            pPdoMemInfo = &drvInstance_l.pdoMem;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_getMemory(drvInstance_l.dualProcDrvInst,
                                    DUALPROCSHM_BUFF_ID_PDO, &pMem, &memSize_p, FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't allocate Pdo buffer (%d)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    pPdoMemInfo->pKernelVa = pMem;
    pPdoMemInfo->memSize = memSize_p;

    // Allocate new MDL pointing to PDO memory
    pPdoMemInfo->pMdl = IoAllocateMdl(pPdoMemInfo->pKernelVa, pPdoMemInfo->memSize,
                                      FALSE, FALSE, NULL);

    if (pPdoMemInfo->pMdl == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error allocating MDL !\n", __func__);
        return kErrorNoResource;
    }

    // Update the MDL with physical addresses
    MmBuildMdlForNonPagedPool(pPdoMemInfo->pMdl);
    // Map the memory in user space and get the address
    pPdoMemInfo->pUserVa = MmMapLockedPagesSpecifyCache(pPdoMemInfo->pMdl,  // MDL
                                                      UserMode,             // Mode
                                                      MmCached,             // Caching
                                                      NULL,                 // Address
                                                      FALSE,                // Bugcheck?
                                                      NormalPagePriority);  // Priority

    if (pPdoMemInfo->pUserVa == NULL)
    {
        MmUnmapLockedPages(pPdoMemInfo->pUserVa, pPdoMemInfo->pMdl);
        IoFreeMdl(pPdoMemInfo->pMdl);
        DEBUG_LVL_ERROR_TRACE("%s() Error mapping MDL !\n", __func__);
        return kErrorNoResource;
    }

    *ppPdoMem_p = pPdoMemInfo->pUserVa;

    TARCE("%s() PDO mem address in user space %p\n", pPdoMemInfo->pUserVa);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free PDO memory

Frees the PDO memory previously allocated.

\param  ppPdoMem_p    Pointer to PDO memory.
\param  memSize_p     Size of the PDO memory.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_freePdoMem(UINT8* pPdoMem_p, size_t memSize_p)
{
    tPdoMemInfo*           pPdoMemInfo = &drvInstance_l.pdoMem;
    tDualprocReturn        dualRet;

    if (pPdoMemInfo->pMdl == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() MDL already deleted !\n", __func__);
        return;
    }

    PRINTF("%s() try to free address %p\n", __func__, pPdoMem_p);

    if (pPdoMemInfo->pUserVa != NULL)
    {
        MmUnmapLockedPages(pPdoMemInfo->pUserVa, pPdoMemInfo->pMdl);
        IoFreeMdl(pPdoMemInfo->pMdl);
    }

    dualRet = dualprocshm_freeMemory(drvInstance_l.dualProcDrvInst, DUALPROCSHM_BUFF_ID_PDO, FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't free Pdo buffer (%d)\n",
                              __func__, dualRet);
        return;
    }

    pPdoMem_p = NULL;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize event queues

Initializes shared event queues between user and kernel. The memory for the
queues are allocated in PCIe memory and is retrieved using dualprocshm library.
The circular buffer library is used to manage the queues.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initEvent(void)
{
    tCircBufError           circError = kCircBufOk;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_USER_TO_KERNEL_QUEUE, &drvInstance_l.eventQueueInst[kEventQueueU2K]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_USER_TO_KERNEL_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_KERNEL_TO_USER_QUEUE, &drvInstance_l.eventQueueInst[kEventQueueK2U]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_KERNEL_TO_USER_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close event queues

Close event queues initialized earlier.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static void exitEvent(void)
{
    circbuf_disconnect(drvInstance_l.eventQueueInst[kEventQueueK2U]);
    circbuf_disconnect(drvInstance_l.eventQueueInst[kEventQueueU2K]);
}

//------------------------------------------------------------------------------
/**
\brief  Initialize error handler memory

Retrieves the shared memory for the error handler module. This memory is only
accesible to user space through IOCTL calls.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initErrHndl(void)
{
    tDualprocReturn      dualRet;
    UINT8*               pBase;
    size_t               span;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (drvInstance_l.pErrorObjects != NULL)
        return kErrorInvalidOperation;

    dualRet = dualprocshm_getMemory(drvInstance_l.dualProcDrvInst, DUALPROCSHM_BUFF_ID_ERRHDLR,
                                    &pBase, &span, FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get Error counter buffer(%d)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    if (span < sizeof(tErrHndObjects))
    {
        DEBUG_LVL_ERROR_TRACE("%s: Error Handler Object Buffer too small\n",
                              __func__);
        return kErrorNoResource;
    }

    drvInstance_l.pErrorObjects = (tErrHndObjects*) pBase;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free error handler memory

*/
//------------------------------------------------------------------------------
static void exitErrHndl(void)
{
    if (drvInstance_l.pErrorObjects != NULL)
    {
        dualprocshm_freeMemory(drvInstance_l.dualProcDrvInst, DUALPROCSHM_BUFF_ID_ERRHDLR, FALSE);
        drvInstance_l.pErrorObjects = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize DLL queues for user layer

This routine initializes the DLL queues shared between user and kernel stack.
The memories for the queue are located in PCIe memory and is accesses using
circular buffer and dualprocshm library

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initDllQueues(void)
{
    tCircBufError           circError = kCircBufOk;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXGEN, &drvInstance_l.dllQueueInst[kDllCalQueueTxGen]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXGEN circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXNMT, &drvInstance_l.dllQueueInst[kDllCalQueueTxNmt]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXNMT circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXSYNC, &drvInstance_l.dllQueueInst[kDllCalQueueTxSync]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXSYNC circbuffer\n");
        return kErrorNoResource;
    }

#if 0
    circError = circbuf_connect(CIRCBUF_DLLCAL_TXVETH, &drvInstance_l.dllQueueInst[kDllCalQueueTxVeth]);
    
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXVETH circbuffer\n");
        return kErrorNoResource;
    }
#endif
    return kErrorOk;

}

//------------------------------------------------------------------------------
/**
\brief  Free DLL queues for user layer

*/
//------------------------------------------------------------------------------
static void exitDllQueues(void)
{
    circbuf_disconnect(drvInstance_l.dllQueueInst[kDllCalQueueTxGen]);
    circbuf_disconnect(drvInstance_l.dllQueueInst[kDllCalQueueTxNmt]);
    circbuf_disconnect(drvInstance_l.dllQueueInst[kDllCalQueueTxSync]);
#if 0
    circbuf_disconnect(drvInstance_l.dllQueueInst[kDllCalQueueTxVeth]);
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Write data into DLL queue

Writes the data into specified DLL queue shared between user and kernel.

\param  pDllCircBuffInst_p  Pointer to the DLL queue instance.
\param  pData_p             Pointer to the data to be insterted.
\param  pDataSize_p         Pointer to size of data.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError insertDataBlock(tCircBufInstance* pDllCircBuffInst_p,
                                  BYTE* pData_p, UINT* pDataSize_p)
{
    tOplkError                  ret = kErrorOk;
    tCircBufError               error;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (pDllCircBuffInst_p == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    error = circbuf_writeData(pDllCircBuffInst_p, pData_p, *pDataSize_p);
    switch (error)
    {
        case kCircBufOk:
            break;

        case kCircBufExceedDataSizeLimit:
        case kCircBufBufferFull:
            ret = kErrorDllAsyncTxBufferFull;
            break;

        case kCircBufInvalidArg:
        default:
            ret = kErrorNoResource;
            break;
    }

Exit:
    return ret;
}

///\}
