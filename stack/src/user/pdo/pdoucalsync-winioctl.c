/**
********************************************************************************
\file   pdoucalsync-winioctl.c

\brief  Sync implementation for the PDO user CAL module using Windows ioctl

This files implements the user PDO CAL module for synchronization of PDO
exchange for the system using Windows ioctl for communication.

\ingroup module_pdoucal
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
#include <oplk/oplkinc.h>
#include <common/pdo.h>
#include <user/pdoucal.h>
#include <user/ctrlucal.h>

#include <common/driver.h>

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
typedef struct
{
    HANDLE    pGlobalFileHandle;
    HANDLE    pSyncFileHandle;
    BOOL      fIntialized;
}tPdoInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

tPdoInstance    pdoInstance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO user CAL sync module

The function initializes the PDO user CAL sync module

\param  pfnSyncCb_p             Function that is called in case of sync event

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_initSync(tSyncCb pfnSyncCb_p)
{
    UNUSED_PARAMETER(pfnSyncCb_p);
    UINT    errNum = 0;
    pdoInstance_l.pGlobalFileHandle = ctrlucal_getFd();
    pdoInstance_l.pSyncFileHandle = CreateFile(PLK_DEV_FILE,                                        // Name of the NT "device" to open
                                           GENERIC_READ | GENERIC_WRITE,                        // Access rights requested
                                           FILE_SHARE_READ | FILE_SHARE_WRITE,                  // Share access - NONE
                                           NULL,                                                // Security attributes - not used!
                                           OPEN_EXISTING,                                       // Device must exist to open it.
                                           FILE_ATTRIBUTE_NORMAL,                               // Open for overlapped I/O
                                           NULL);

    if (pdoInstance_l.pSyncFileHandle == INVALID_HANDLE_VALUE)
    {
        errNum = GetLastError();

        if (!(errNum == ERROR_FILE_NOT_FOUND ||
              errNum == ERROR_PATH_NOT_FOUND))
        {
            DEBUG_LVL_ERROR_TRACE("%s() createFile failed!  ERROR_FILE_NOT_FOUND = %d\n",
                                  errNum);
            return kErrorNoResource;
        }
    }

    pdoInstance_l.fIntialized = TRUE;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO user CAL sync module

The function cleans up the PDO user CAL sync module
*/
//------------------------------------------------------------------------------
void pdoucal_exitSync(void)
{
    ULONG    bytesReturned;
    //printf("Exit Sync\n");
    if (!DeviceIoControl(pdoInstance_l.pGlobalFileHandle, PLK_CMD_CLEAN,
        0, 0,
        0, 0, &bytesReturned, NULL))
    CloseHandle(pdoInstance_l.pSyncFileHandle);
    pdoInstance_l.fIntialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event.

\param  timeout_p       Specifies a timeout in microseconds. If 0 it waits
                        forever.

\return The function returns a tOplkError error code.
\retval kErrorOk              Successfully received sync event
\retval kErrorGeneralError    Error while waiting on sync event
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_waitSyncEvent(ULONG timeout_p)
{
    ULONG    bytesReturned;

    if (!pdoInstance_l.fIntialized)
        return kErrorNoResource;

    if (!DeviceIoControl(pdoInstance_l.pSyncFileHandle, PLK_CMD_PDO_SYNC,
        &timeout_p, sizeof(ULONG),
        0, 0, &bytesReturned, NULL))
    {
        printf("Error in IOCTL %d\n", GetLastError());
        return kErrorGeneralError;
    }

    //printf("Sync Return\n");
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}

