/**
********************************************************************************
\file   edrv.c

\brief  Implementation of ethernet driver module

This file contains the implementation of the ethernet driver module for
Realtek 8111B / 8168B.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include "stub_cfg.h"
//#include "stub_scg_new.h"
//#include "libs/global.h"
//#include "libs/EplInc.h"
//#include "driver/edrv.h"

#include <oplk/oplk.h>
#include <common/ami.h>
#include <kernel/dllkfilter.h>

#include <kernel/edrv.h>
#include <target/openmac.h>
#include <omethlib.h>
#include <common/target.h>

#include <oplk/benchmark.h>
#include <oplk/debug.h>



//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS 256
#endif

#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS 14
#endif
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
UINT16 count;
BOOL sendFlag;

int                     plkMajor_g = 0;
int                     plkMinor_g = 0;
int                     plkNrDevs_g = 1;
//dev_t                   plkDev_g;
//struct class            *plkClass_g;
//struct cdev             plkCdev_g;
tEdrvTxBuffer*          m_apTxBuffer[EDRV_MAX_TX_BUFFERS];
tEdrvTxBuffer*          apRxBuffer[EDRV_MAX_RX_BUFFERS];
static  UINT16          bufferCount = 0;
static  UINT16          rxBufferCount = 0;
//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------
static  int powerlinkInit (void);
static void powerlinkExit (void);

static int      powerlinkOpen    (void);
static int      powerlinkRelease (void);
//static ssize_t  powerlinkRead    (struct file* pInstance_p, char* pDstBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
//static ssize_t  powerlinkWrite   (struct file* pInstance_p, const char* pSrcBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
//#ifdef HAVE_UNLOCKED_IOCTL
//static  long     powerlinkIoctl   (struct file* filp, unsigned int cmd, unsigned long arg);
//#else
//static int      powerlinkIoctl   (struct inode* dev, struct file* filp, unsigned int cmd, unsigned long arg);
//#endif

//static int      powerlinkMmap(struct file *filp, struct vm_area_struct *vma);
//static void     powerlinkVmaOpen(struct vm_area_struct *vma);
//static void     powerlinkVmaClose(struct vm_area_struct *vma);


tEdrvReleaseRxBuffer stub_processFrameReceived(tEdrvRxBuffer * pRxBuffer_p);
void stub_processFrameTransmitted(tEdrvTxBuffer * pTxBuffer_p);
tOplkError stub_transmitFrame(void);
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
tEdrvInitParam  EdrvInitParam;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
tEdrvReleaseRxBuffer dllk_processFrameReceived(tEdrvRxBuffer * pRxBuffer_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
//---------------------------------------------------------------------------
//  Initailize Driver
//---------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  module initialization

The function implements openPOWERLINK kernel module initialization function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static  int powerlinkInit (void)
{
    int  err;

    printf("PLK: powerlinkInit()  Driver build: %s / %s\n", __DATE__, __TIME__);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  module cleanup and exit

The function implements openPOWERLINK kernel module exit function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void powerlinkExit (void)
{
    printf("PLK: powerlinkExit...\n");
}


void main(void)
{
	int ret = 0;

	ret = powerlinkOpen();
	ret = powerlinkRelease();

}
//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver open function

The function implements openPOWERLINK kernel module open function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int powerlinkOpen(void)
{
    unsigned int        ret;
    UINT16              idx;
    static char         devName[128];
    unsigned int        devNumber = 0;
    const char* macAddr = "00:00:00:00:00:00";

    UINT16  lcount = WAIT_LOOP_COUNT;
    printf("PLK: + powerlinkOpen...\n");
    printf("PLK: + powerlinkOpen...\n");

    EdrvInitParam.hwParam.pDevName = devName;
    EdrvInitParam.hwParam.devNum = devNumber;
    OPLK_MEMCPY(EdrvInitParam.aMacAddr, macAddr, sizeof (EdrvInitParam.aMacAddr));
    EdrvInitParam.pfnRxHandler = stub_processFrameReceived;
    //    EdrvInitParam.pfnTxHandler = EplDllkCbFrameTransmitted; //jba why commented out?
    printf("Passed MacAddr: %s\n",EdrvInitParam.aMacAddr);

    if ((ret = edrv_init(&EdrvInitParam)) != kErrorOk)
        return ret;

    printf("PLK: + powerlinkOpen - OK\n");

    //Vinod


    for (idx = 0; idx < EDRV_MAX_TX_BUFFERS; idx++)
    {
        m_apTxBuffer[idx] = (tEdrvTxBuffer*) OPLK_MALLOC(sizeof(tEdrvTxBuffer));
        if (m_apTxBuffer[idx] == NULL)
        {
            printf("Could not allocate memory for buffer structure\n");
            return -1;
        }
        OPLK_MEMSET(m_apTxBuffer[idx], 0, sizeof(tEdrvTxBuffer));
        //printf("idx %d..\n", idx);
    }
    //printf("34..\n");
#ifdef USE_RX_FILTERING
    {
        UINT8 abmacAddr[6];
        ami_setUint48Be(&abmacAddr[0],  0x01005E000002LL);//0xFFFFFFFFFFFF);//
        ret = edrv_setRxMulticastMacAddr(abmacAddr);
        ami_setUint48Be(&abmacAddr[0],  0x01111E000002LL);//0xFFFFFFFFFFFF);//
        ret = edrv_setRxMulticastMacAddr(abmacAddr);
    }
#endif

#ifdef USE_TX_LOOP
    //for (idx = 0; idx < SEND_LOOP_COUNT; idx++)
    while(1)
    {
    	printf("Send..\n");
        ret = stub_transmitFrame();
        //usleep(100000);
        usleep(100000);
    }
#endif
//*
    //while((count < (SEND_LOOP_COUNT)) && (lcount-- > 0))
    printf("Entering to While loop..\n");
    while(1)
    {
#ifdef USE_TX_CB
        if (sendFlag == FALSE)
        {
            lcount = WAIT_LOOP_COUNT;
            sendFlag = TRUE;
            ret = stub_transmitFrame();
            if (ret != kErrorOk)
            {
                printf("Pkt buffer could not be acquired\n");
                return 0;
            }

        }
#endif
        //msleep_interruptible(10);
        usleep(100000);
    }//*/
    printf("PLK: + powerlinkOpen - DONE\n");
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver close function

The function implements openPOWERLINK kernel module close function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int  powerlinkRelease (void)
{
    UINT16  idx;
    printf("PLK: + powerlinkRelease...\n");

    edrv_shutdown();
    printf("PLK: + powerlinkRelease - OK\n");
    for (idx = 0; idx < EDRV_MAX_TX_BUFFERS; idx++)
    {
        if (m_apTxBuffer[idx] != NULL)
        {
            OPLK_FREE(m_apTxBuffer[idx]);
        }
    }

    for (idx = 0; idx < EDRV_MAX_RX_BUFFERS; idx++)
    {
        if (apRxBuffer[idx] != NULL)
        {
            OPLK_FREE(apRxBuffer[idx]);
        }
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver read function

The function implements openPOWERLINK kernel module read function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
//static ssize_t  powerlinkRead(struct file* pInstance_p, char* pDstBuff_p,
//                              size_t BuffSize_p, loff_t* pFileOffs_p)
//{
//    int  ret;
//
//    OPLK_DBGLVL_ALWAYS_TRACE("PLK: + powerlinkRead...\n");
//    OPLK_DBGLVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
//    ret = -EINVAL;
//    OPLK_DBGLVL_ALWAYS_TRACE("PLK: - powerlinkRead (iRet=%d)\n", ret);
//    return ret;
//
//}


//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver write function

The function implements openPOWERLINK kernel module write function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
//static ssize_t  powerlinkWrite(struct file* pInstance_p, const char* pSrcBuff_p,
//                               size_t BuffSize_p, loff_t* pFileOffs_p)
//{
//    int  ret;
//
//    OPLK_DBGLVL_ALWAYS_TRACE("PLK: + powerlinkWrite...\n");
//    OPLK_DBGLVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
//    ret = -EINVAL;
//    OPLK_DBGLVL_ALWAYS_TRACE("PLK: - powerlinkWrite (iRet=%d)\n", ret);
//    return ret;
//}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

/**
\brief  Callback function for received frame

The function implements the callback function to process a received frame.

\param  pRxBuffer_p         Pointer to received frame.

\return The function returns a tEdrvReleaseRxBuffer flag to determine if the
        buffer could be released immediately
\retval kEdrvReleaseRxBufferImmediately     Buffer could be released immediately.
\retval kEdrvReleaseRxBufferLater           Buffer must be released later.
*/
//------------------------------------------------------------------------------
tOplkError stub_transmitFrame()
{
    tEdrvTxBuffer* pTxBuffer;
    tOplkError      ret;
    UINT16          idx;
    ometh_packet_typ    packet;
    static BYTE         macaddr = 0x00;

    if(macaddr >= 0xFB) macaddr = 0x00;
    printf("Fill Tx dum\n");
    //Fill the dummy pkt
    OPLK_MEMCPY(packet.data.srcMac, EdrvInitParam.aMacAddr, 6*sizeof(unsigned char));

    packet.data.dstMac[0] = 0x00;
    packet.data.dstMac[1] = 0x60;
    packet.data.dstMac[2] = 0x65;
    packet.data.dstMac[3] = 0x07;
    packet.data.dstMac[4] = 0x08;
    packet.data.dstMac[5] = 0xfe;
/*
    packet.data.dstMac[0] = 0x00;
    packet.data.dstMac[1] = 0x00;
    packet.data.dstMac[2] = 0x00;
    packet.data.dstMac[3] = 0x00;
    packet.data.dstMac[4] = 0x00;
    packet.data.dstMac[5] = 0x00;/*/
/*
    packet.data.dstMac[0] = macaddr++;
    packet.data.dstMac[1] = macaddr++;
    packet.data.dstMac[2] = macaddr++;
    packet.data.dstMac[3] = macaddr++;
    packet.data.dstMac[4] = macaddr++;
    packet.data.dstMac[5] = macaddr++;//*/

    packet.data.ethertype[0] = 0x88;
    packet.data.ethertype[1] = 0xAB;
    packet.length = 60;


    packet.data.minData[0] = 0x04;
    packet.data.minData[1] = 0xff;
    packet.data.minData[2] = 0xf0;
    packet.data.minData[3] = 0x1d;

    for (idx=4; idx < 46; idx++)
    {
        packet.data.minData[idx] = 0x0;
    }

    /*
    01 11 1e 00 00 03
    00 00  08 00 00 00
    88 ab
    05 ff f0 1d 00 00 00 00
    20 00 00 00 00 00 00 00
    00 00 00 00 00 00 00 00
    00 00 00 00 00 00 00 00
    00 00 00 00 00 00 00 00
    00 00 00 00 00 00
   //*/

    pTxBuffer = m_apTxBuffer[bufferCount++];
    if (bufferCount == EDRV_MAX_TX_BUFFERS)
    {
        bufferCount = 0;
    }
    if (pTxBuffer == NULL)
    {
        printf("what the... buffCount: %u\n",bufferCount);
        ret = kErrorNoResource;
        return ret;
    }
    pTxBuffer->maxBufferSize = 0x5DC;
    if (pTxBuffer->pBuffer == NULL)
    {

        ret = edrv_allocTxBuffer(pTxBuffer);
        if (ret != kErrorOk)
        {
            printf("Alloc buffer sucks.. 0x%08X\n", ret);
            return ret;
        }
        printf("Allocate tx buffer: 0x%08X\n", pTxBuffer);
    }

    pTxBuffer->timeOffsetAbs = 0;
    pTxBuffer->timeOffsetNs = 0;
    pTxBuffer->pfnTxHandler = stub_processFrameTransmitted;
/*
    for (idx=0; idx < 60; idx++)
    {
        pTxBuffer->pBuffer[idx] =     packet->data.minData[] = 0x00;idx;
    }//*/
    printf("Fill Tx @ 0x%08X, from: 0x%08X\n", pTxBuffer->pBuffer, &packet);
    OPLK_MEMCPY(pTxBuffer->pBuffer, &packet.data, sizeof(struct ometh_packet_data_typ));
    printf("Fill Tx done\n");
    printf("0x%08X\n", (struct ometh_packet_data_typ *)(pTxBuffer->pBuffer));
    pTxBuffer->txFrameSize = 60;
    ret = edrv_sendTxBuffer (pTxBuffer);
    return ret;
}
/**
\brief  Callback function for received frame

The function implements the callback function to process a received frame.

\param  pRxBuffer_p         Pointer to received frame.

\return The function returns a tEdrvReleaseRxBuffer flag to determine if the
        buffer could be released immediately
\retval kEdrvReleaseRxBufferImmediately     Buffer could be released immediately.
\retval kEdrvReleaseRxBufferLater           Buffer must be released later.
*/
//------------------------------------------------------------------------------
void stub_processFrameTransmitted(tEdrvTxBuffer * pTxBuffer_p)
{
    tOplkError ret;

    printf("pkt transmitted\n");
    printf("Releasing buffer 0x%08X\n", (struct ometh_packet_data_typ *)(pTxBuffer_p->pBuffer));
    ret = edrv_freeTxBuffer(pTxBuffer_p);
    sendFlag = FALSE;
    count++;
}
/**
\brief  Callback function for received frame

The function implements the callback function to process a received frame.

\param  pRxBuffer_p         Pointer to received frame.

\return The function returns a tEdrvReleaseRxBuffer flag to determine if the
        buffer could be released immediately
\retval kEdrvReleaseRxBufferImmediately     Buffer could be released immediately.
\retval kEdrvReleaseRxBufferLater           Buffer must be released later.
*/
//------------------------------------------------------------------------------
tEdrvReleaseRxBuffer stub_processFrameReceived(tEdrvRxBuffer * pRxBuffer_p)
{
    tOplkError ret = kErrorOk;
    static UINT16   count = 0;

    printf("pkt received: %u, buf_struct_VA: 0x%08X, buf_VA: 0x%08X \n", count++, pRxBuffer_p, pRxBuffer_p->pBuffer);
    printf("Dst Mac: %X:%X:%X:%X:%X:%X\n", pRxBuffer_p->pBuffer[0], pRxBuffer_p->pBuffer[1],pRxBuffer_p->pBuffer[2],
              pRxBuffer_p->pBuffer[3], pRxBuffer_p->pBuffer[4], pRxBuffer_p->pBuffer[5]);

//#define RELEASE_RX_BUF
#ifdef RELEASE_RX_BUF
    if (rxBufferCount >= EDRV_MAX_RX_BUFFERS)
    {
        rxBufferCount = 0;
    }
    if (apRxBuffer[rxBufferCount] == NULL)
    {
        apRxBuffer[rxBufferCount] = (tEdrvRxBuffer*) OPLK_MALLOC(sizeof(tEdrvRxBuffer));
    }
    OPLK_MEMSET(apRxBuffer[rxBufferCount], 0, sizeof(tEdrvRxBuffer));
    OPLK_MEMCPY(apRxBuffer[rxBufferCount], pRxBuffer_p, sizeof(tEdrvRxBuffer));
    if (rxBufferCount >= 5)
    {
        tEdrvRxBuffer * pRxBuffer = apRxBuffer[rxBufferCount - 5];
        ret = edrv_releaseRxBuffer(pRxBuffer);
        if ( ret == kErrorOk)
        {
            OPLK_FREE(pRxBuffer);
            apRxBuffer[rxBufferCount - 5] = NULL;
        }
    }
    rxBufferCount++;
#endif
    return kEdrvReleaseRxBufferImmediately;
}

//#ifdef HAVE_UNLOCKED_IOCTL
//static long powerlinkIoctl (struct file* filp, unsigned int cmd,
//                         unsigned long arg)
//#else
//static int  powerlinkIoctl (struct inode* dev, struct file* filp,
//                         unsigned int cmd, unsigned long arg)
//#endif
//{
//    printf("No Implementation for VMA\n");
//    return 0;
//
//}
//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver mmap function

The function implements openPOWERLINK kernel module mmap function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
//static int powerlinkMmap(struct file *filp, struct vm_area_struct *vma)
//{
//    printf("No Implementation for MMAP\n");
//    return 0;
//
//}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver VMA open functionnet

The function implements openPOWERLINK kernel module VMA open function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
//static void powerlinkVmaOpen(struct vm_area_struct *vma)
//{
//    printf("No Implementation for VMA\n");
//}

//------------------------------------------------------------------------------
/**printf
\brief  openPOWERLINK driver VMA close function

The function implements openPOWERLINK kernel module VMA close function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
//static void powerlinkVmaClose(struct vm_area_struct *vma)
//{
//    printf("No Implementation for VMA\n");
//}

