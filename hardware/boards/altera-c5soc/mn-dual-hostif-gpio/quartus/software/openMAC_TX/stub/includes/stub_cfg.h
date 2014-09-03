/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  interface for Ethernet driver

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                Dev C++ and GNU-Compiler for m68k

  -------------------------------------------------------------------------

  Revision History:

  2005/08/01 m.b.:   start of implementation

****************************************************************************/

#ifndef _STUB_CFG_H_
#define _STUB_CFG_H_


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
#ifndef EDRV_8168
    #define EDRV_8168
#endif
#ifndef CONFIG_EDRV
    #define CONFIG_EDRV 8111
#endif

//#define USE_TX_CB
//#define USE_TX_LOOP
#define USE_RX_FILTERING

#define WAIT_LOOP_COUNT         5000
#define SEND_LOOP_COUNT         20

#define PLK_CLASS_NAME    "plk"
#define PLK_DEV_NAME      "plk" // used for "/dev" and "/proc" entry
#define PLK_DRV_NAME      "plk"
#define PLK_DEV_FILE      "/dev/plk"
#define PLK_IOC_MAGIC     '='

//------------------------------------------------------------------------------
//  Commands for <ioctl>
//------------------------------------------------------------------------------
#define PLK_CMD_CTRL_EXECUTE_CMD                _IOWR(PLK_IOC_MAGIC, 0, tCtrlCmd)
#define PLK_CMD_CTRL_STORE_INITPARAM            _IOW (PLK_IOC_MAGIC, 1, tCtrlInitParam)
#define PLK_CMD_CTRL_READ_INITPARAM             _IOR (PLK_IOC_MAGIC, 2, tCtrlInitParam)
#define PLK_CMD_CTRL_GET_STATUS                 _IOR (PLK_IOC_MAGIC, 3, UINT16)
#define PLK_CMD_CTRL_GET_HEARTBEAT              _IOR (PLK_IOC_MAGIC, 4, UINT16)
#define PLK_CMD_POST_EVENT                      _IOW (PLK_IOC_MAGIC, 5, tEplEvent)
#define PLK_CMD_GET_EVENT                       _IOR (PLK_IOC_MAGIC, 6, tEplEvent)
#define PLK_CMD_DLLCAL_ASYNCSEND                _IO  (PLK_IOC_MAGIC, 7)
#define PLK_CMD_ERRHND_WRITE                    _IOW (PLK_IOC_MAGIC, 8, tErrHndIoctl)
#define PLK_CMD_ERRHND_READ                     _IOR (PLK_IOC_MAGIC, 9, tErrHndIoctl)
#define PLK_CMD_PDO_SYNC                        _IO  (PLK_IOC_MAGIC, 10)

//---------------------------------------------------------------------------
// types
//---------------------------------------------------------------------------
/*
typedef struct
{
    unsigned long    length;        // frame length excluding checksum

    struct ometh_packet_data_typ
    {
        unsigned char dstMac[6];
        unsigned char srcMac[6];
        unsigned char ethertype[2];
        unsigned char minData[46];    // minimum number of data bytes for a standard ethernet frame
        unsigned char checkSum[4];
    }data;
}ometh_packet_typ;
*/
#endif  // #ifndef _STUB_CFG_H_


