/*
 * SerialComm.cpp
 *
 * Created: 9/10/2015 2:12:34 AM
 *  Author: Lim Zi H
 */ 

#include "SerialProtocol.h"
#include "SerialPkt.h"
#include "SerialComm.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

void createPacket(byte_t * MsgQ, int dataLength, DataPacket * datapacket, int type)
{
	byte_t HEADER = 0x00;
	if(type==1) HEADER = 0x01;
	datapacket->PacketHeader = HEADER;
	datapacket->Length = dataLength;
	
	// copies the data from the message queue to the packet
	memcpy(datapacket->Data, MsgQ, dataLength);
	
	// populate unused memory in the packet to 0s
	memset(datapacket->Data + dataLength, 0, DATA_LENGTH - dataLength);
	
	// generates the checksum for all empty data bytes 
	datapacket->Checksum = generateCheckSum(MsgQ, HEADER_SIZE_LENGTH + DATA_SIZE_LENGTH + dataLength);  
}

byte_t generateCheckSum(byte_t * MsgQ, int datalength)
{
	byte_t checksum = 0;
	int i;
	for(i=0; i<dataLength; i++){
		checksum = (checksum ^ MsgQ[i]); // xor operation for the last bit of every data byte in the data field
	}
	return checksum;
}

void SendPacket(DataPacket * datapacket)
{
	SERIALPORT.write(datapacket, PACKET_SIZE);
}