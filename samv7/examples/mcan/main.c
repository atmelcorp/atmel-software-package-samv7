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
 *  \page mcan Controller Area Network (CAN) Example using MCAN
 *
 *  \section Purpose
 *
 *  The Controller Area Network (CAN) Example will help new users get familiar
 *  with the MCAN peripheral used in Atmel's Samv7 family of microcontrollers.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  The demonstration program periodically transmits several different CAN
 *  messages.  The program enables CAN loopback mode so that the transmitted
 *  messages are reflected back as received messages as well.  LED #0 is toggled
 *  on a successful transmission of one of the messages & LED #1 is toggles on a
 *  successful reception of the same message.  Message transmission is stopped
 *  while SWITCH #0 is held depressed, which in turns means that both LED's
 *  should stop blinking.  Depressing SWITCH #1 alternates between standard CAN
 *  operation and CAN-FD operation with bit rate switching.
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
 *  -# Start the application.
 *  -# LED(s) should start blinking on the board. In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- Controller Area Network (CAN) Example %s --\n\r", SOFTPACK_VERSION );
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -- LED0 toggles on CAN message reception
 *      -- LED1 toggles on CAN message transmission
 *      -- CAN message transmission stopped while SW0 is pushed
 *      -- SW1 pushes alternate between standard CAN and CAN-FD
 *     \endcode
 * -# Both LEDs should stop blinking while Button #0 is held depressed.
 *
 *  \section References
 *  - main.c
 *  - pio.h
 *  - pio_it.h
 *  - led.h
 *  - trace.h
 */

/** \file
 *
 *  This file contains all the specific code for the CAN example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

#define MSG_ID_0           0x555       // standard ID
#define MSG_ID_0_DATA1     0x81        // 1 byte message
#define MSG_ID_1           0x11111111  // extended ID
#define MSG_ID_1_DATA1_4   0x55555555  // 8 byte message
#define MSG_ID_1_DATA5_8   0x00FF00FF
#define MSG_ID_2           0x444       // standard ID
#define MSG_ID_2_MASK      0x7FC       // bits 0 & 1 are don't care
#define TX_BUFFER_0        0
#define TX_BUFFER_1        1
#define RX_BUFFER_0        0
#define RX_BUFFER_1        1
#define FILTER_0           0
#define FILTER_1           1

#define BUTTON_NOT_PUSHED  1
#define BUTTON_PUSHED      0

#define CAN_STANDARD       0
#define CAN_FD             1

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

static const Pin pushbutton[] = PINS_PUSHBUTTONS;

volatile uint8_t tick = 0;

uint32_t       txdCntr;
uint32_t       rxdCntr;
uint32_t       rxFifoCntr;
uint32_t     * txMailbox0;
uint32_t     * txMailbox1;
Mailbox8Type   rxMailbox0;
Mailbox8Type   rxMailbox1;
Mailbox8Type   rxMailbox2;
uint8_t        txData[8];

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/* Handler for MCAN0, Line 0 */
void MCAN0_Handler(void)
{
}

/* Handler for MCAN0, Line 1 */
void MCAN0_Line1_Handler(void)
{
}

/* Handler for MCAN1, Line 0 */
void MCAN1_Handler(void)
{
}

/* Handler for MCAN1, Line 1 */
void MCAN1_Line1_Handler(void)
{
	if (MCAN_IsMessageStoredToRxDedBuffer(&mcan1Config)) {
		MCAN_ClearMessageStoredToRxBuffer(&mcan1Config);
		if (MCAN_IsNewDataInRxDedBuffer(&mcan1Config, RX_BUFFER_0)) {
			MCAN_GetRxDedBuffer(&mcan1Config, RX_BUFFER_0, (Mailbox64Type *)
				&rxMailbox0);
		// verify data
			if (rxMailbox0.data[0] == MSG_ID_0_DATA1) {
				// Toggle LED #0 To Indicate New Message Received in Interrupt
				LED_Toggle(0);
				rxdCntr++;
			}
		}
		if (MCAN_IsNewDataInRxDedBuffer(&mcan1Config, RX_BUFFER_1)) {
			MCAN_GetRxDedBuffer(&mcan1Config, RX_BUFFER_1, (Mailbox64Type *)
				&rxMailbox1);
		rxdCntr++;
		}
	}
}

/**
 *  Interrupt handler for TC0 interrupt. Toggles the state of LED\#2.
 */
void TC0_Handler(void)
{
	volatile uint32_t dummy;
	/* Clear status bit to acknowledge interrupt */
	dummy = TC0->TC_CHANNEL[0].TC_SR;
	tick = 1;
}

/**
 *  Configure Timer Counter 0 to generate an interrupt every 250ms.
 */
static void _ConfigureTc(void)
{
	uint32_t div;
	uint32_t tcclks;

	/** Enable peripheral clock. */
	PMC_EnablePeripheral(ID_TC0);
	/** Configure TC for a 4Hz frequency and trigger on RC compare. */
	TC_FindMckDivisor(4, BOARD_MCK, &div, &tcclks, BOARD_MCK);

	TC_Configure(TC0, 0, tcclks | TC_CMR_CPCTRG);
	TC0->TC_CHANNEL[0].TC_RC = (BOARD_MCK / div) / 4;

	/* Configure and enable interrupt on RC compare */
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_EnableIRQ(TC0_IRQn);

	TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;

	/** Start the counter */
	TC_Start(TC0, 0);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main(void)
{
	uint32_t id_offset = 0;
	uint32_t fifo_entries;
	uint32_t button_state = BUTTON_PUSHED;
	uint32_t button_state2 = BUTTON_NOT_PUSHED;
	uint32_t can_mode = CAN_STANDARD;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Output example information */
	printf("\n\r-- Controller Area Network (CAN) Example %s --\n\r",
			SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r\n\r", __DATE__, __TIME__);
	printf("-- LED0 toggles on CAN message reception\n\r");
	printf("-- LED1 toggles on CAN message transmission\n\r");
	printf("-- CAN message transmission stopped while SW0 is pushed\n\r");
	printf("-- SW1 pushes alternate between standard CAN and CAN-FD\n\r");

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Configure systick for 1 ms. */
	TimeTick_Configure();

	/* Configure Board LED's */
	LED_Configure(0);
#if 2 == LED_NUM
	LED_Configure(1);
#endif

	/* Configure Timer/Counter for 250msec Tick */
	_ConfigureTc();

	/* Configure Board Push-buttons */
	// SW1 is a ERASE system function, switch it to port function
	MATRIX->CCFG_SYSIO |= (1u << 12);
	/* have to disable the pull down on PB12 for SW1 before the pull up can be
		enabled */
	PIOB->PIO_PPDDR = 1 << 12;
	PIO_Configure(pushbutton, PIO_LISTSIZE(pushbutton));
	/* Adjust pio denounce filter parameters, uses 10 Hz filter. */
	PIO_SetDebounceFilter(&pushbutton[0], 10);
#if 2 == BUTTON_NUM
	PIO_SetDebounceFilter(&pushbutton[1], 10000);
#endif

	SystemCoreClockUpdate();

	MCAN_Init(&mcan1Config);

	MCAN_InitFdBitRateSwitchEnable(&mcan1Config);
	MCAN_InitTxQueue(&mcan1Config);
	MCAN_InitLoopback(&mcan1Config);  // remove to disable loop back mode
	MCAN_Enable(&mcan1Config);

	MCAN_IEnableMessageStoredToRxDedBuffer(&mcan1Config, CAN_INTR_LINE_1);

	txMailbox0 = (uint32_t *)MCAN_ConfigTxDedBuffer(&mcan1Config, TX_BUFFER_0,
			MSG_ID_0, CAN_STD_ID, CAN_DLC_1);
	txMailbox1 = (uint32_t *)MCAN_ConfigTxDedBuffer(&mcan1Config, TX_BUFFER_1,
			MSG_ID_1, CAN_EXT_ID, CAN_DLC_8);
	MCAN_ConfigRxBufferFilter(&mcan1Config, RX_BUFFER_0, FILTER_0, MSG_ID_0,
			CAN_STD_ID);
	MCAN_ConfigRxBufferFilter(&mcan1Config, RX_BUFFER_1, FILTER_0, MSG_ID_1,
			CAN_EXT_ID);
	MCAN_ConfigRxClassicFilter(&mcan1Config, CAN_FIFO_0, FILTER_1, MSG_ID_2,
			CAN_STD_ID, MSG_ID_2_MASK);

	while (1) {
		 if (tick) {
			tick = 0;
			if (PIO_Get(&pushbutton[0]) == BUTTON_NOT_PUSHED) {
				if (button_state2 == BUTTON_NOT_PUSHED) {
					/* periodically transmit messages while SW0 is not pushed */
					// send standard ID from a dedicated buffer
					*txMailbox0 = MSG_ID_0_DATA1;  // write data into CAN mailbox
					MCAN_SendTxDedBuffer(&mcan1Config, TX_BUFFER_0, CAN_STANDARD);  // send data
					txdCntr++;
					// send extended ID from a dedicated buffer
					*txMailbox1 = MSG_ID_1_DATA1_4;  // write data into CAN mailbox
					*(txMailbox1 + 1) = MSG_ID_1_DATA5_8; // write data into CAN mailbox
					MCAN_SendTxDedBuffer(&mcan1Config, TX_BUFFER_1, CAN_STANDARD); // send the data
					txdCntr++;
					// send from Tx Queue
					MCAN_AddToTxFifoQ(&mcan1Config, MSG_ID_2 + id_offset, CAN_STD_ID,
							CAN_DLC_1, &txData[0], CAN_STANDARD);
					txdCntr++;
					// increment the offset so we send different ID's within the
					// range defined by the mask
					id_offset = (id_offset + 1) & (uint32_t) ~MSG_ID_2_MASK;
					// update data being sent for next time
					txData[0]++;
					button_state2 = BUTTON_NOT_PUSHED;
				}
			} else {
				if (button_state2 == BUTTON_NOT_PUSHED)
					button_state2 = BUTTON_PUSHED;
				else
					button_state2 = BUTTON_NOT_PUSHED;
			}
		}
		/* poll for TX'd message complete */
		 if (MCAN_IsTxComplete(&mcan1Config)) {
			MCAN_ClearTxComplete(&mcan1Config);
			if (MCAN_IsBufferTxd(&mcan1Config, TX_BUFFER_0)) {
		#if 2 == LED_NUM
				LED_Toggle(1);
		#endif
			}
		}
		/* Poll for new CAN messages in RX FIFO */
		do {
			fifo_entries = MCAN_GetRxFifoBuffer(&mcan1Config, CAN_FIFO_0,
					(Mailbox64Type *) &rxMailbox2);
			if (fifo_entries > 0) rxdCntr++;
		} while (fifo_entries > 1);

		/* SW1 used to alternate between standard CAN and CAN-FD operation */
#if 2 == BUTTON_NUM
		if (PIO_Get(&pushbutton[1]) == BUTTON_NOT_PUSHED) {
#else
		if (1) {
#endif
			 button_state = BUTTON_NOT_PUSHED;
		} else {
			if (button_state == BUTTON_NOT_PUSHED ) {
				// Switch on a NOT PUSHED to PUSHED edge
				button_state = BUTTON_PUSHED;
				if (can_mode == CAN_STANDARD) {
					can_mode = CAN_FD;
					MCAN_RequestFdBitRateSwitch( &mcan1Config );
				} else {
					can_mode = CAN_STANDARD;
					MCAN_RequestIso11898_1( &mcan1Config );
				}
			}
		}
#ifdef POLLKEYBOARD
	/* poll for keyboard entry */
	if (DBG_IsRxReady()) {
		keyboard = DBG_GetChar();
		if (((char) keyboard == 'L' ) || ( (char) keyboard == 'l')) {
			if (loopback == 1) {
				MCAN_LoopbackOff(&mcan1Config);
				loopback = 0;
			} else {
				MCAN_LoopbackOn(&mcan1Config);
				loopback = 1;
			}
		}
	}
#endif
	}
}
