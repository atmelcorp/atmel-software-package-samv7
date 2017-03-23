/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

/**
 *  \page gmac GMAC Example
 *
 *  \section Purpose
 *
 *  This example uses the Gigabit Ethernet MAC (GMAC) and the on-board Ethernet
 *  transceiver available on Atmel evaluation kits. It enables the device to
 *  respond to a ping command sent by a host computer.
 *
 *  \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  Upon startup, the program will configure the GMAC with a default IP and
 *  MAC address and then ask the transceiver to auto-negotiate the best mode
 *  of operation. Once this is done, it will start to monitor incoming packets
 *  and processing them whenever appropriate.
 *
 *  The basic will only answer to two kinds of packets:
 *
 *  - It will reply to ARP requests with its MAC address,
 *  - and to ICMP ECHO request so the device can be PING'ed.
 *
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Connect an Ethernet cable between the evaluation board and the network.
 *      The board may be connected directly to a computer; in this case,
 *      make sure to use a cross/twisted wired cable such as the one provided
 *      with the evaluation kit.
 *  -# Start the application. It will display the following message on the DBGU:
 *     \code
 *      -- GMAC Example xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      MAC 3a 1f 34 08 54 54
 *      IP  192, 168, 1, 3
 *     \endcode
 *  -# The program will then auto-negotiate the mode of operation and start
 *     receiving packets, displaying feedback on the DBGU. To display additional
 *     information, press any key in the terminal application.
 *  \note
 *  Make sure the IP address of the device(EK board) and the computer are in the
 *  same network.
 *
 *  \section References
 *  - gmac/main.c
 *  - gmacb.h
 *  - gmacd.h
 *  - gmac.h
 */

/** \file
 *
 *  This file contains all the specific code for the gmac example.
 *
 */

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include <board.h>
#include <string.h>

#include "MiniIp.h"

/*---------------------------------------------------------------------------
 *         Local Define
 *---------------------------------------------------------------------------*/

#define TX_BUFFERS           32     /** Must be a power of 2 */
#define RX_BUFFERS           32     /** Must be a power of 2 */

#define BUFFER_SIZE          1536
#define DUMMY_SIZE           2
#define DUMMY_BUFF_SIZE      128

/** GMAC packet processing offset */
#define GMAC_RCV_OFFSET     0

/** Enable/Disable CopyAllFrame */
#define GMAC_CAF_DISABLE    0
#define GMAC_CAF_ENABLE     1

/** Enable/Disable NoBroadCast */
#define GMAC_NBC_DISABLE    0
#define GMAC_NBC_ENABLE     1

/** EK board sends out MAX_ARP_REQUEST ARP request and wants to get at least
    MIN_ARP_REPLY reply */
#define GMAX_ARP_REQUEST    100
#define GMIN_ARP_REPLY      90

/** Try to get link */
#define GMAX_TRY_LINK       500

/** TWI clock frequency in Hz. */
#define TWCK            400000
/** Slave address of twi_eeprom AT24MAC.*/
#define AT24MAC_SERIAL_NUM_ADD  0x5F
/** Page size of an AT24MAC402 chip (in bytes)*/
#define PAGE_SIZE       16
/** Page numbers of an AT24MAC402 chip */
#define EEPROM_PAGES    16
/** EEPROM Pins definition */
#define BOARD_PINS_TWI_EEPROM PINS_TWI0
/** TWI0 peripheral ID for EEPROM device*/
#define BOARD_ID_TWI_EEPROM   ID_TWIHS0
/** TWI0 base address for EEPROM device */
#define BOARD_BASE_TWI_EEPROM TWIHS0
/*---------------------------------------------------------------------------
 *         Local variables
 *---------------------------------------------------------------------------*/
/** TWI driver instance.*/
static Twid twid;

/** GMAC power control pin */
#if !defined(BOARD_GMAC_POWER_ALWAYS_ON)
static const Pin gmacPwrDn[] = {BOARD_GMAC_PIN_PWRDN};
#endif

/** The PINs for GMAC */
static const Pin gmacPins[]      = {BOARD_GMAC_RUN_PINS};
static const Pin gmacResetPin   = BOARD_GMAC_RESET_PIN;
/** The PINs for TWI*/
static const Pin twiPins[]      = BOARD_PINS_TWI_EEPROM;

/** The MAC address used for demo */
static uint8_t GMacAddress[6] = {0x3a, 0x1f, 0x34, 0x08, 0x54, 0x54};

/** The IP address used for demo */
static uint8_t GIpAddress[4]    = {192, 168, 1,  3 };
static uint8_t GDesIpAddress[4] = {192, 168, 1, 2 };

/** The GMAC driver instance */
static sGmacd gGmacd;

/** The MACB driver instance */
static GMacb gGmacb;

/** TX descriptors list */
COMPILER_SECTION(".ram_nocache")
COMPILER_ALIGNED(8) static sGmacTxDescriptor gTxDs[TX_BUFFERS],
				gDummyTxDs[DUMMY_SIZE];


/** TX callbacks list */
COMPILER_ALIGNED(8) static fGmacdTransferCallback gTxCbs[TX_BUFFERS],
				gDummyTxCbs[DUMMY_SIZE];

/** RX descriptors list */
COMPILER_SECTION(".ram_nocache")
COMPILER_ALIGNED(8) static sGmacRxDescriptor gRxDs[RX_BUFFERS],
				gDummyRxDs[DUMMY_SIZE];

/** Send Buffer */
/* Section 3.6 of AMBA 2.0 spec states that burst should not cross 1K Boundaries.
   Receive buffer manager writes are burst of 2 words => 3 lsb bits of the address
   shall be set to 0 */
COMPILER_ALIGNED(32) static uint8_t pTxBuffer[TX_BUFFERS * BUFFER_SIZE],
				pTxDummyBuffer[DUMMY_SIZE * DUMMY_BUFF_SIZE];

/** Receive Buffer */
COMPILER_ALIGNED(32) static uint8_t pRxBuffer[RX_BUFFERS * BUFFER_SIZE],
				pRxDummyBuffer[DUMMY_SIZE * DUMMY_BUFF_SIZE];

/** Buffer for Ethernet packets */
static uint8_t GEthBuffer[BUFFER_SIZE * TX_BUFFERS * 2];

static uint8_t gbIsIpAddrInit = 1;
static uint8_t gtotal_request;
static uint8_t gtotal_reply;

/*---------------------------------------------------------------------------
 *         Local functions
 *---------------------------------------------------------------------------*/
/**
 * Gmac interrupt handler
 */
void GMAC_Handler(void)
{
	GMACD_Handler(&gGmacd, GMAC_QUE_0);
}

/**
 * Display the protocol header
 */
static void GDisplayEthernetHeader(PEthHeader pEth, uint32_t size)
{
	printf("======= Ethernet %4u bytes, HEADER ==========\n\r",
			(unsigned int)size);
	printf(" @Mac dst = %02x.%02x.%02x.%02x.%02x.%02x\n\r",
			pEth->et_dest[0], pEth->et_dest[1], pEth->et_dest[2],
			pEth->et_dest[3], pEth->et_dest[4], pEth->et_dest[5]);
	printf(" @Mac src = %02x.%02x.%02x.%02x.%02x.%02x\n\r",
			pEth->et_src[0], pEth->et_src[1], pEth->et_src[2],
			pEth->et_src[3], pEth->et_src[4], pEth->et_src[5]);
	printf(" Protocol = %d\n\r", pEth->et_protlen);
}

static void GDisplayArpHeader(PArpHeader pArp, uint32_t size)
{
	printf("======= ARP %4u bytes, HEADER ==========\n\r", (unsigned int)size);
	printf(" Hardware type        = %d\n\r", SWAP16(pArp->ar_hrd) );
	printf(" protocol type        = 0x%04x\n\r", SWAP16(pArp->ar_pro) );
	printf(" Hardware addr lg     = %d\n\r", pArp->ar_hln);
	printf(" Protocol addr lg     = %d\n\r", pArp->ar_pln);
	printf(" Operation            = %d\n\r", SWAP16(pArp->ar_op) );
	printf(" Sender hardware addr = %02x.%02x.%02x.%02x.%02x.%02x\n\r",
			pArp->ar_sha[0], pArp->ar_sha[1], pArp->ar_sha[2],
			pArp->ar_sha[3], pArp->ar_sha[4], pArp->ar_sha[5]);
	printf(" Sender protocol addr = %d.%d.%d.%d\n\r",
			pArp->ar_spa[0], pArp->ar_spa[1], pArp->ar_spa[2], pArp->ar_spa[3]);
	printf(" Target hardware addr = %02x.%02x.%02x.%02x.%02x.%02x\n\r",
			pArp->ar_tha[0], pArp->ar_tha[1], pArp->ar_tha[2],
			pArp->ar_tha[3], pArp->ar_tha[4], pArp->ar_tha[5]);
	printf(" Target protocol addr = %d.%d.%d.%d\n\r",
			pArp->ar_tpa[0], pArp->ar_tpa[1], pArp->ar_tpa[2], pArp->ar_tpa[3]);
}

static void GDisplayIpHeader(PIpHeader pIpHeader, uint32_t size)
{
	printf("======= IP %4u bytes, HEADER ==========\n\r", (unsigned int) size);
	printf(" IP Version        = v.%d\n\r", (pIpHeader->ip_hl_v & 0xF0) >> 4);
	printf(" Header Length     = %d\n\r", pIpHeader->ip_hl_v & 0x0F);
	printf(" Type of service   = 0x%x\n\r", pIpHeader->ip_tos);
	printf(" Total IP Length   = 0x%X\n\r",
			(((pIpHeader->ip_len) >> 8) & 0xff) + (((pIpHeader->ip_len) << 8) & 0xff00) );
	printf(" ID                = 0x%X\n\r",
			(((pIpHeader->ip_id) >> 8) & 0xff) + (((pIpHeader->ip_id) << 8) & 0xff00) );
	printf(" Header Checksum   = 0x%X\n\r",
			(((pIpHeader->ip_sum) >> 8) & 0xff) + (((pIpHeader->ip_sum) << 8) & 0xff00) );
	printf(" Protocol          = ");

	switch (pIpHeader->ip_p) {
	case IP_PROT_ICMP:
		printf("ICMP\n\r");
		break;

	case IP_PROT_IP:
		printf("IP\n\r");
		break;

	case IP_PROT_TCP:
		printf("TCP\n\r");
		break;

	case IP_PROT_UDP:
		printf("UDP\n\r");
		break;

	default:
		printf("%d (0x%X)\n\r", pIpHeader->ip_p, pIpHeader->ip_p);
		break;
	}

	printf(" IP Src Address    = %d:%d:%d:%d\n\r",
			pIpHeader->ip_src[0],
			pIpHeader->ip_src[1],
			pIpHeader->ip_src[2],
			pIpHeader->ip_src[3]);

	printf(" IP Dest Address   = %d:%d:%d:%d\n\r",
			pIpHeader->ip_dst[0],
			pIpHeader->ip_dst[1],
			pIpHeader->ip_dst[2],
			pIpHeader->ip_dst[3]);
	printf("----------------------------------------\n\r");
}

/**
 * initialize the IP address of the board if not yet initialized
 */
static void garp_init_ip_addr(uint8_t *pData)
{
	uint32_t i;
	PArpHeader   pArp = (PArpHeader)(pData + 14 + GMAC_RCV_OFFSET);

	if (SWAP16(pArp->ar_op) == ARP_REQUEST) {

		if (gbIsIpAddrInit == 0) {

			printf("first arp request, Check @ip. src=%d.%d.%d.%d dst=%d.%d.%d.%d ",
					pArp->ar_spa[0], pArp->ar_spa[1], pArp->ar_spa[2], pArp->ar_spa[3],
					pArp->ar_tpa[0], pArp->ar_tpa[1], pArp->ar_tpa[2], pArp->ar_tpa[3]);

			if ((pArp->ar_tpa[0] == pArp->ar_spa[0]) &&
					(pArp->ar_tpa[1] == pArp->ar_spa[1]) &&
					(pArp->ar_tpa[2] == pArp->ar_spa[2]) &&
					(pArp->ar_tpa[3] >= 250) &&
					(pArp->ar_tpa[3] <= 254)) {
				for (i = 0; i < 4; i++)
					GIpAddress[i] = pArp->ar_tpa[i];

				printf("=> OK\n\r");
				gbIsIpAddrInit = 1;

			} else
				printf("=> KO!\n\r");
		}
	}
}
/**
 * Send ARP Request
 */
static void garp_request(uint8_t *pData)
{
	uint32_t i;
	uint8_t gmac_rc = GMACD_OK;

	PEthHeader   pEth = (PEthHeader)(pData + GMAC_RCV_OFFSET);
	PArpHeader   pArp = (PArpHeader)(pData + 14 + GMAC_RCV_OFFSET);

	pEth->et_protlen =  SWAP16(ETH_PROT_ARP);
	// ARP REPLY operation
	pArp->ar_hrd =  SWAP16(0x0001);
	pArp->ar_pro =  SWAP16(ETH_PROT_IP);
	pArp->ar_hln =  6;
	pArp->ar_pln =  4;
	pArp->ar_op =  SWAP16(ARP_REQUEST);

	/* Fill the dest address and src address */
	for (i = 0; i < 6; i++) {
		/* swap ethernet dest address and ethernet src address */
		pEth->et_dest[i] = 0xff;
		pEth->et_src[i]  = GMacAddress[i];
		pArp->ar_tha[i]  = 0x00;
		pArp->ar_sha[i]  = GMacAddress[i];
	}

	/* swap sender IP address and target IP address */
	for (i = 0; i < 4; i++) {
		pArp->ar_tpa[i] = GDesIpAddress[i];
		pArp->ar_spa[i] = GIpAddress[i];
	}

	gmac_rc = GMACD_Send(&gGmacd,
						(pData + GMAC_RCV_OFFSET),
						42,
						NULL, GMAC_QUE_0);

	if (gmac_rc != GMACD_OK)
		printf("-E- ARP_REQUEST Send - 0x%x\n\r", gmac_rc);
}

/**
 * Process the received ARP packet
 */
static void garp_process_packet(uint8_t *pData, uint32_t size)
{
	uint32_t i, j;
	uint8_t gmac_rc = GMACD_OK;

	PEthHeader   pEth = (PEthHeader)pData;
	PArpHeader   pArp = (PArpHeader)(pData + 14 + GMAC_RCV_OFFSET);

	if (SWAP16(pArp->ar_op) == ARP_REQUEST) {

		/* ARP REPLY operation */
		pArp->ar_op =  SWAP16(ARP_REPLY);

		/* Fill the destination address and source address */
		for (i = 0; i < 6; i++) {
			/* swap Ethernet destination address and ethernet source address */
			pEth->et_dest[i] = pEth->et_src[i];
			pEth->et_src[i]  = GMacAddress[i];
			pArp->ar_tha[i]  = pArp->ar_sha[i];
			pArp->ar_sha[i]  = GMacAddress[i];
		}

		/* swap sender IP address and target IP address */
		for (i = 0; i < 4; i++) {
			pArp->ar_tpa[i] = pArp->ar_spa[i];
			pArp->ar_spa[i] = GIpAddress[i];
		}

		gmac_rc = GMACD_Send(&gGmacd,
							 (pData + GMAC_RCV_OFFSET),
							 size,
							 NULL, GMAC_QUE_0);

		if (gmac_rc != GMACD_OK)
			printf("-E- ARP Send - 0x%x\n\r", gmac_rc);
	}

	if (SWAP16(pArp->ar_op) == ARP_REPLY) {
		/* check sender IP address and target IP address */
		for (i = 0, j = 0; i < 4; i++) {
			if (pArp->ar_tpa[i] != GIpAddress[i]) {j++; break;}

			if (pArp->ar_spa[i] != GDesIpAddress[i]) {j++; break;}
		}

		if (!j)
			gtotal_reply++;
	}
}

/**
 * Process the received IP packet
 */
static void gip_process_packet(uint8_t *pData)
{
	uint32_t i;
	uint32_t icmp_len;
	uint32_t gmac_rc = GMACD_OK;

	PEthHeader   pEth = (PEthHeader)pData;
	PIpHeader    pIpHeader = (PIpHeader)(pData + 14 + GMAC_RCV_OFFSET);

	PIcmpEchoHeader pIcmpEcho = (PIcmpEchoHeader)((char *)pIpHeader + 20);

	switch (pIpHeader->ip_p) {

	case IP_PROT_ICMP:

		/* if ICMP_ECHO_REQUEST ==> resp = ICMP_ECHO_REPLY */
		if (pIcmpEcho->type == ICMP_ECHO_REQUEST) {
			pIcmpEcho->type = ICMP_ECHO_REPLY;
			pIcmpEcho->code = 0;
			pIcmpEcho->cksum = 0;

			/* Checksum of the ICMP Message */
			icmp_len = (SWAP16(pIpHeader->ip_len) - 20);

			if (icmp_len % 2) {
				*((uint8_t *)pIcmpEcho + icmp_len) = 0;
				icmp_len ++;
			}

			icmp_len = icmp_len / sizeof(unsigned short);

			pIcmpEcho->cksum =
				SWAP16(IcmpChksum((unsigned short *) pIcmpEcho, icmp_len));

			/* Swap IP destination  address and IP Source address */
			for (i = 0; i < 4; i++) {
				GIpAddress[i] = pIpHeader->ip_dst[i];
				pIpHeader->ip_dst[i] = pIpHeader->ip_src[i];
				pIpHeader->ip_src[i] = GIpAddress[i];
			}

			/* Swap Eth destination  address and Source address */
			for (i = 0; i < 6; i++) {

				/* swap ethernet destination address and ethernet
				source address */
				pEth->et_dest[i] = pEth->et_src[i];
				pEth->et_src[i]  = GMacAddress[i];
			}

			/* send the echo_reply */

			gmac_rc = GMACD_Send(&gGmacd,
								(pData + GMAC_RCV_OFFSET),
								SWAP16(pIpHeader->ip_len) + 14 + GMAC_RCV_OFFSET,
								NULL, GMAC_QUE_0);

			if (gmac_rc != GMACD_OK)
				printf("-E- ICMP Send - 0x%x\n\r", (unsigned int) gmac_rc);
		}

		break;

	default:
		break;
	}
}


/**
 * Process the received GMAC packet
 */
static void geth_process_packet(uint8_t *pData, uint32_t size)
{
	uint16_t pkt_format;

	PEthHeader   pEth = (PEthHeader)(pData + GMAC_RCV_OFFSET);
	PIpHeader    pIpHeader = (PIpHeader)(pData + 14 + GMAC_RCV_OFFSET);
	IpHeader     ipHeader;

	pkt_format = SWAP16(pEth->et_protlen);

	switch (pkt_format) {
	/* ARP Packet format */
	case ETH_PROT_ARP:

		/* Dump the ARP header */
		GDisplayEthernetHeader(pEth, size);
		GDisplayArpHeader((PArpHeader)(pData + 14 + GMAC_RCV_OFFSET), size);

		/* initialize the IP address if not yet initialized */
		garp_init_ip_addr(pData);

		/* Process the ARP packet */
		if (gbIsIpAddrInit == 0) return;

		garp_process_packet(pData, size);

		/* Dump for ARP packet */
		break;

	/* IP protocol frame */
	case ETH_PROT_IP:

		if (gbIsIpAddrInit == 0) return;

		/* Backup the header */
		memcpy(&ipHeader, pIpHeader, sizeof(IpHeader));

		/* Process the IP packet */
		gip_process_packet(pData);

		/* Dump the IP header */
		GDisplayIpHeader(&ipHeader, size);
		break;

	default:
		break;
	}
}

/*---------------------------------------------------------------------------
 *         Global functions
 *---------------------------------------------------------------------------*/

/**
 *  \brief GMAC Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
int main(void)
{
	sGmacd  *pGmacd = &gGmacd;
	GMacb   *pGmacb = &gGmacb;
	uint32_t frmSize;
	uint32_t delay;
	sGmacInit Que0, Que;
	uint8_t  OrigiGMacAddr[16];
	uint32_t i;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	printf("-- GMAC Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* Configure systick for 1 ms. */
	TimeTick_Configure();

	/* Configure TWI pins. */
	PIO_Configure(twiPins, PIO_LISTSIZE(twiPins));
	/* Enable TWI */
	PMC_EnablePeripheral(BOARD_ID_TWI_EEPROM);
	TWI_ConfigureMaster(BOARD_BASE_TWI_EEPROM, TWCK, BOARD_MCK);
	TWID_Initialize(&twid, BOARD_BASE_TWI_EEPROM);
	/* Display MAC & IP settings */
	printf("-- EK`s IP  %d.%d.%d.%d\n\r",
			GIpAddress[0], GIpAddress[1], GIpAddress[2], GIpAddress[3]);
	printf("-- PC`s IP  %d.%d.%d.%d\n\r",
			GDesIpAddress[0], GDesIpAddress[1], GDesIpAddress[2], GDesIpAddress[3]);

	TWID_Read(&twid, AT24MAC_SERIAL_NUM_ADD, 0x9A, 1, OrigiGMacAddr, PAGE_SIZE, 0);

	if ((OrigiGMacAddr[0] == 0xFC) && (OrigiGMacAddr[1] == 0xC2)
		&& (OrigiGMacAddr[2] == 0x3D)) {
		for (i = 0; i < 6; i++)
			GMacAddress[i] = OrigiGMacAddr[i];
	}

	GMAC_SetAddress(gGmacd.pHw, 0, GMacAddress);
	printf("-- MAC %x:%x:%x:%x:%x:%x\n\r",
			GMacAddress[0], GMacAddress[1], GMacAddress[2],
			GMacAddress[3], GMacAddress[4], GMacAddress[5]);

	printf("Connect the board to a host PC via an ethernet cable\n\r");

	/* Initialize GMAC driver structure Queue 0*/
	memset(&Que0, 0, sizeof(Que0));
	Que0.bIsGem = 1;
	Que0.bDmaBurstLength = 4;
	Que0.pRxBuffer = pRxBuffer;
	Que0.pRxD = gRxDs;
	Que0.wRxBufferSize = BUFFER_SIZE;
	Que0.wRxSize = RX_BUFFERS;
	Que0.pTxBuffer = pTxBuffer;
	Que0.pTxD = gTxDs;
	Que0.wTxBufferSize = BUFFER_SIZE;
	Que0.wTxSize = TX_BUFFERS;
	Que0.pTxCb = gTxCbs;

	/* Initialize GMAC driver structure of other Queue */
	memset(&Que, 0, sizeof(Que));
	Que.bIsGem = 1;
	Que.bDmaBurstLength = 4;
	Que.pRxBuffer = pRxDummyBuffer;
	Que.pRxD = gDummyRxDs;
	Que.wRxBufferSize = DUMMY_BUFF_SIZE;
	Que.wRxSize = DUMMY_SIZE;
	Que.pTxBuffer = pTxDummyBuffer;
	Que.pTxD = gDummyTxDs;
	Que.wTxBufferSize = DUMMY_BUFF_SIZE;
	Que.wTxSize = DUMMY_SIZE;
	Que.pTxCb = gDummyTxCbs;

	GMACD_Init(pGmacd, GMAC, ID_GMAC, GMAC_CAF_ENABLE, GMAC_NBC_DISABLE);
	for(gmacQueList_t q = GMAC_QUE_1; q <= GMAC_QUE_MAX; q++)
  {
    /* All queues must be initialized regardless if they're used or not */
    GMACD_InitTransfer(pGmacd, &Que,  q);
  }
	GMACD_InitTransfer(pGmacd, &Que0, GMAC_QUE_0);


	/* Setup interrupts */
	NVIC_ClearPendingIRQ(GMAC_IRQn);
	NVIC_EnableIRQ(GMAC_IRQn);

	GMACB_Init(pGmacb, pGmacd, BOARD_GMAC_PHY_ADDR);
	GMACB_ResetPhy(pGmacb);

	/* PHY initialize */
	if (!GMACB_InitPhy(pGmacb, BOARD_MCK, &gmacResetPin, 1, gmacPins,
						PIO_LISTSIZE(gmacPins))) {
		printf("PHY Initialize ERROR!\n\r");
		return 0;
	}

	printf("GMACB_AutoNegotiate\n\r");

	if (!GMACB_AutoNegotiate(pGmacb)) {
		printf("Auto Negotiate ERROR!\n\r");
		return 0;
	}

	delay = 0;
	gtotal_request = 0;
	gtotal_reply = 0;

	while (1) {
		if (gtotal_request >= GMAX_ARP_REQUEST)break;

		if ((delay++) >= (BOARD_MCK / 1000000)) {
			delay = 0;
			gtotal_request++;
			printf("arp... \n\r");
			garp_request(GEthBuffer);
		}

		/* Process packets */
		if (GMACD_OK != GMACD_Poll(pGmacd, GEthBuffer, sizeof(GEthBuffer),
									&frmSize, GMAC_QUE_0))
			continue;

		if (frmSize > 0) {
			/* Handle input frame */
			printf("Process_packet ...\n\r");
			geth_process_packet(GEthBuffer, frmSize);
		}
	}

	/* Disable GMAC interrupts */
	NVIC_DisableIRQ(GMAC_IRQn);

	printf("EK sends out %d ARP request and gets %d reply\n\r",
			gtotal_request, gtotal_reply);

	while (1);
}
