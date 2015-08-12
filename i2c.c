#include "RFID.h"

void initI2C(unsigned char targetAddress)

{
#define sclPin BIT6
#define sdaPin BIT7
#define i2cPorts (sclPin | sdaPin)

/*
	//  Setting DCO to 1MHZ
	DCOCTL = 0;					// set to lowest speed for safety before chaning
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;		// SMCLK = DOC
*/

/*
The recommended USCI initialization or reconfiguration process is:
1. Set UCSWRST (BIS.B #UCSWRST,&UCxCTL1)
2. Initialize all USCI registers with UCSWRST=1 (including UCxCTL1)
3. Configure ports.
4. Clear UCSWRST via software (BIC.B #UCSWRST,&UCxCTL1)
5. Enable interrupts (optional) via UCxRXIE and/or UCxTXIE
*/

UCB0CTL1 |= UCSWRST;        				// 1. USCI logic held in reset state.
UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     	// 2. 7-bit address, Master, I2C, synchronous mode = I2C and SPI, Asycnhronous mode = UART

UCB0CTL1 = UCSSEL_2 + UCTR + UCSWRST;       // Use SMCLK, TX mode, keep SW reset

UCB0BR0 = 12;                  				// fSCL = SMCLK/12 = ~100kHz,  16-bit value (= UCA0BR0 + UCA0BR1 * 256)
UCB0BR1 = 0;

UCB0I2CSA  = targetAddress;          		// define Target Address such as EEPROM.

UCB0I2COA = 0x0105;                       	// own address.

// 3. Configure ports.
P1SEL  |= i2cPorts;		// set P1.6 and P1.7 for I2C  page 43 of hardware data sheet
P1SEL2 |= i2cPorts;		//

/*  if do this, I2C will not work, a test for using internal resisters for pulling up of I2C Bus
P1REN |= i2cPorts;		//pullup/pulldown resistor enabled
P1OUT |= i2cPorts;      //the pin is pulled up
*/


// 4. Clear UCSWRST via software (BIC.B #UCSWRST,&UCxCTL1)

UCB0I2CIE = 0;			// I2C interrupt Enable Register
IE2 &= ~(0x0c);			// USCI_B0 transmit and receive interrupt

UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
}

void i2cTxBufByte(unsigned char buff) {
    UCB0TXBUF = buff;
    IFG2 &= ~UCB0TXIFG;
    while ( !(IFG2 & UCB0TXIFG) );		// waiting for sending the data to be done with sending.
}

void i2cTxBufInt(unsigned int buff) {
    i2cTxBufByte((unsigned char) (buff >> 8));
    i2cTxBufByte((unsigned char) buff);
}

void i2cTransmitterMode() {
    UCB0CTL1 |= (UCTXSTT | UCTR);   // I2C start condition, If UCTR == 1, then transmitter mode
    								// by doing this, UCB0TXIFG = 1, USB0TXBUF discarded.
}

void i2cReceiverMode() {
	UCB0CTL1 &= ~UCTR;				 // If UCTR == 1, then transmitter mode, If UCTR == 0, Receiver mode.
    UCB0CTL1 |= UCTXSTT;             // I2C start condition		The hardware waits for free of the communicaiton line.
}

void i2cStop() {
    UCB0CTL1 |= UCTXSTP;             // I2C stop condition
    while (UCB0CTL1 & UCTXSTP);      // Ensure stop condition got sent
}

void i2cStopAfterStart() {
	while (UCB0CTL1 & UCTXSTT);      // Start condition sent?
	i2cStop();
}

unsigned char i2cRead() {

    i2cReceiverMode();

    i2cStopAfterStart();

    return UCB0RXBUF;
}

unsigned char i2cAddressRead(unsigned int address) {

	i2cTransmitterMode();

	i2cTxBufInt(address);

	return i2cRead();
}

void i2cWrite(unsigned int address, unsigned int data) {

	i2cTransmitterMode();

	i2cTxBufInt(address);		// address has to be even.
	i2cTxBufInt(data);

	i2cStop();


    do {						// waitting for interal writing to be done.
    	i2cReceiverMode();
	    i2cStopAfterStart();
    } while(UCB0STAT & UCNACKIFG);	// If ACK is received , UCNACKIFG = 0; If not received, UCNACKIFG = 1;
}


void i2cTest() {	// find a correct address and initialize i2c with it.
	unsigned char h;
	unsigned char address;
	for (h = 0xa0; h <= 0xaf; h++) {
	    initI2C((h >> 1));				// because of the RFID chip
		putH(h);

		i2cReceiverMode();
	    i2cStopAfterStart();

		putC('-');
		putH(UCB0STAT & UCNACKIFG);		// If ACK is received , UCNACKIFG = 0; If not received, UCNACKIFG = 1;
		if (!(UCB0STAT & UCNACKIFG)) {
			address = h >> 1;
		}
		putC(' ');
	}
	putS("\r\n");
	initI2C(address);
}


