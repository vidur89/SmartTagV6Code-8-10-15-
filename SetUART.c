#include "RFID.h"

void SetUART (void) {
//	Bit_Time = 768;	// 768 for 9600 baud, 64 for 115,200 baud

/*  pp413 from user's guide
	NOTE: Initializing or Re-Configuring the USCI Module
	The recommended USCI initialization/re-configuration process is:
	1. Set UCSWRST (BIS.B #UCSWRST,&UCAxCTL1)
	2. Initialize all USCI registers with UCSWRST = 1 (including UCAxCTL1)
	3. Configure ports.
	4. Clear UCSWRST via software (BIC.B #UCSWRST,&UCAxCTL1)
	5. Enable interrupts (optional) via UCAxRXIE and/or UCAxTXIE
*/

    /* Setting DCO to 1MHZ   */

	DCOCTL = 0;		// set to lowest speed for safety before changing
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;


	/* setting port0 for UART */
	UCA0CTL1 |= UCSWRST;			// Set UCSWRST = 1

	UCA0CTL0 = 0x00;  				// no Parity, LSB first, 8-bit, one stop bit, UART mode, Asynchronous

	UCA0CTL1 = 0x80 | 0x41;        		// SMCLK(= DCO), Reset

	UCA0MCTL = 0x02; //(3 << 1) | (0x00);	// Oversampling mode disabled, UCBRSx = 3 for 9600 baud from table 15-4 in pp 423 of User's guide

	UCA0BR0 = 104;            // 3 for 9600 baud.           16-bit value (= UCA0BR0 + UCA0BR1 * 256)
	UCA0BR1 = 0;

	UCA0STAT = 0;			// Loopback mode disabled

	P1SEL |= BIT1 | BIT2;	// set P1 for TX, RX of UART  page 43 of hardware data sheet
	P1SEL2 |= BIT1 | BIT2;

	UCA0CTL1 &= ~UCSWRST;	// Clear UCSWRST (=0)
	IE2 &= (~0x03);			// TX, RX interuppt diabled
}

void putC(char byte) {
	while (!(IFG2 & UCA0TXIFG));
	UCA0TXBUF = byte;
}

void putS(char *str) {
	int i = 0;

	while (str[i]) {
		putC(str[i++]);
	}
}

#include <stdlib.h>
void putI(int k) {
	int s[10];
	int  sPos = 0;
	div_t out;

	if (k < 0) {
		putC('-');
		k = -k;
	}

	do{
		out = div(k, 10);
		s[sPos++] = out.rem;
		k = out.quot;
	} while( k != 0);

	for (; sPos > 0; ) {
		putC('0' + s[--sPos]);
	}
}

void sub_putH(unsigned char byte) {
	if (byte > 9) putC(byte + ('A'-10));
	else putC(byte + '0');
}
void putH(unsigned char byte) {
	sub_putH(byte >> 4);
	sub_putH(byte & 0x0f);
}



