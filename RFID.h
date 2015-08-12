/*
 * RFID.h
 *
 *  Created on: Feb 5, 2015
 *      Author: Young
 */

#ifndef RFID_H_
#define RFID_H_

#include <msp430.h>



void SetUART (void);
void putS(char *str);
void putC(char byte);
void putI(int k);
void initI2C(unsigned char);

void putH(unsigned char byte);

//unsigned char randomRead(unsigned int Address);

void i2cTest();
unsigned char i2cRead();
unsigned char i2cAddressRead(unsigned int address);

void i2cWrite(unsigned int address, unsigned int data);

#endif /* RFID_H_ */
