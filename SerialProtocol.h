/*
 * SerialProtocol.h
 * constants to be defined in the serial protocol
 *
 * Created: 9/9/2015 11:23:33 PM
 *  Author: Lim Zi H
 */ 


#ifndef SERIALPROT_H_
#define SERIALPROT_H_

#include <Arduino.h>

#define SERIALPORT Serial

#define START_HEADER				0x01                
#define PACKET_SIZE					21
#define HEADER_SIZE_LENGTH			1
#define DATA_SIZE_LENGTH			1
#define DATA_LENGTH					18
#define CHECKSUM_LENGTH				1

#define DEFAULT_BAUDRATE			9600

#define SERIAL_ERROR				-1

typedef unsigned char byte_t;

#endif /* SERIALPROT_H_ */