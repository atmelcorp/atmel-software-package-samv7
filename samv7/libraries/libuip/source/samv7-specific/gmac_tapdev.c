/* ----------------------------------------------------------------------------
 *         SAM Software Package License 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "uip.h"
#include "uip_arp.h"
#include "string.h"
#include "gmac_tapdev.h"

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/
#define TX_BUFFERS         32     /** Must be a power of 2 */
#define RX_BUFFERS         32     /** Must be a power of 2 */
#define BUFFER_SIZE        1536
#define DUMMY_SIZE              2
#define DUMMY_BUFF_SIZE         512


#define GMAC_CAF_DISABLE  0
#define GMAC_CAF_ENABLE   1
#define GMAC_NBC_DISABLE  0
#define GMAC_NBC_ENABLE   1
/*----------------------------------------------------------------------------
 *        Variables
 *----------------------------------------------------------------------------*/

/* The PINs for GMAC */
static const Pin gmacPins[]   = {BOARD_GMAC_RUN_PINS};
static const Pin gmacResetPin = BOARD_GMAC_RESET_PIN;

/* The GMAC driver instance */
COMPILER_ALIGNED(32) static sGmacd gGmacd;

/* The GMACB driver instance */
COMPILER_ALIGNED(32) static GMacb gGmacb;

/** TX descriptors list */
COMPILER_SECTION(".ram_nocache")
COMPILER_ALIGNED(32) static sGmacTxDescriptor gTxDs[TX_BUFFERS],  gDummyTxDs[DUMMY_SIZE];

/** TX callbacks list */
COMPILER_ALIGNED(32) static fGmacdTransferCallback gTxCbs[TX_BUFFERS], gDummyTxCbs[DUMMY_SIZE];

/** RX descriptors list */
COMPILER_SECTION(".ram_nocache")
COMPILER_ALIGNED(32) static sGmacRxDescriptor gRxDs[RX_BUFFERS], gDummyRxDs[DUMMY_SIZE];

/** Send Buffer */
/* Section 3.6 of AMBA 2.0 spec states that burst should not cross 1K Boundaries.
   Receive buffer manager writes are burst of 2 words => 3 lsb bits of the address
   shall be set to 0 */
COMPILER_ALIGNED(32) static uint8_t pTxBuffer[TX_BUFFERS * BUFFER_SIZE], pTxDummyBuffer[DUMMY_SIZE * DUMMY_BUFF_SIZE];

/** Receive Buffer */
COMPILER_ALIGNED(32) static uint8_t pRxBuffer[RX_BUFFERS * BUFFER_SIZE], pRxDummyBuffer[DUMMY_SIZE * DUMMY_BUFF_SIZE];

/* MAC address used for demo */
static uint8_t gGMacAddress[6] = {0x00, 0x45, 0x56, 0x78, 0x9a, 0xbc};

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * Gmac interrupt handler
 */
void GMAC_Handler(void)
{
    GMACD_Handler(&gGmacd, GMAC_QUE_0);
}


/**
 * Set the MAC address of the system.
 * Should only be called before tapdev_init is called.
 */
void gmac_tapdev_setmac(u8_t *addr)
{
    gGMacAddress[0] = addr[0];
    gGMacAddress[1] = addr[1];
    gGMacAddress[2] = addr[2];
    gGMacAddress[3] = addr[3];
    gGMacAddress[4] = addr[4];
    gGMacAddress[5] = addr[5];
}

/**
 * Initialization for GMAC device.
 * Should be called at the beginning of the program to set up the
 * network interface.
 */
void gmac_tapdev_init(void)
{
    sGmacd    *pGmacd = &gGmacd;
    GMacb      *pGmacb = &gGmacb;
    sGmacInit Que0, Que;

	
    /* Init GMAC driver structure */
    memset(&Que0, 0, sizeof(Que0));
    Que0.bIsGem = 1;
    Que0.bDmaBurstLength = 4;
    Que0.pRxBuffer =pRxBuffer;
    Que0.pRxD = gRxDs;
    Que0.wRxBufferSize = BUFFER_SIZE;
    Que0.wRxSize = RX_BUFFERS;
    Que0.pTxBuffer = pTxBuffer;
    Que0.pTxD = gTxDs;
    Que0.wTxBufferSize = BUFFER_SIZE;
    Que0.wTxSize = TX_BUFFERS;
    Que0.pTxCb = gTxCbs;
    

    
    memset(&Que, 0, sizeof(Que));
    Que.bIsGem = 1;
    Que.bDmaBurstLength = 4;
    Que.pRxBuffer =pRxDummyBuffer;
    Que.pRxD = gDummyRxDs;
    Que.wRxBufferSize = DUMMY_BUFF_SIZE;
    Que.wRxSize = DUMMY_SIZE;
    Que.pTxBuffer = pTxDummyBuffer;
    Que.pTxD = gDummyTxDs;
    Que.wTxBufferSize = DUMMY_BUFF_SIZE;
    Que.wTxSize = DUMMY_SIZE;
    Que.pTxCb = gDummyTxCbs;
    
    /* Init GMAC driver structure */
    GMACD_Init(pGmacd, GMAC, ID_GMAC, GMAC_CAF_ENABLE, GMAC_NBC_DISABLE);
    for(gmacQueList_t q = GMAC_QUE_1; q <= GMAC_QUE_MAX; q++)
    {
        /* All queues must be initialized regardless if they're used or not */
        GMACD_InitTransfer(pGmacd, &Que,  q);
    }
    GMACD_InitTransfer(pGmacd, &Que0, GMAC_QUE_0);
    GMAC_SetAddress(gGmacd.pHw, 0, gGMacAddress);

    /* Setup GMAC buffers and interrupts */
    /* Configure and enable interrupt on RC compare */    
    NVIC_ClearPendingIRQ(GMAC_IRQn);
    NVIC_EnableIRQ(GMAC_IRQn);
    
    /* Init GMACB driver */
    GMACB_Init(pGmacb, pGmacd, BOARD_GMAC_PHY_ADDR);

    /* PHY initialize */
    if (!GMACB_InitPhy(pGmacb, BOARD_MCK,  &gmacResetPin, 1,  gmacPins, PIO_LISTSIZE( gmacPins )))
    {
        printf( "P: PHY Initialize ERROR!\n\r" ) ;
        return;
    }

    /* Auto Negotiate, work in RMII mode */
    if (!GMACB_AutoNegotiate(pGmacb))
    {
        printf( "P: Auto Negotiate ERROR!\n\r" ) ;
        return;
    }
    
    
    printf( "P: Link detected \n\r" ) ;
}

/**
 * Read for GMAC device.
 */
uint32_t gmac_tapdev_read( void )
{
    uint32_t pkt_len = 0 ;
    if ( GMACD_OK != GMACD_Poll( &gGmacd, (uint8_t*)uip_buf, UIP_CONF_BUFFER_SIZE, &pkt_len, GMAC_QUE_0) )
    {
        pkt_len = 0 ;
    }
    return pkt_len ;
}

/**
 * Send to GMAC device
 */
void gmac_tapdev_send( void )
{
    uint8_t gmac_rc ;

    gmac_rc = GMACD_Send( &gGmacd, (void*)uip_buf, uip_len, NULL, GMAC_QUE_0) ;
    if ( gmac_rc != GMACD_OK )
    {
        TRACE_ERROR( "E: Send, rc 0x%x\n\r", gmac_rc ) ;
    }
}

