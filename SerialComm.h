/*
 * SerialComm.h
 *
 * Created: 9/10/2015 12:35:10 AM
 *  Author: Lim Zi H
 */ 


#ifndef SERIALCOMM_H_
#define SERIALCOMM_H_


#include "SerialPkt.h"
#include "SerialProtocol"

void createPacket(byte_t * MsgQ, int dataLength, DataPacket * datapacket, int type);

void SendPacket(DataPacket * datapacket);

byte_t generateCheckSum(byte_t * MsgQ, int datalength);

#endif /* SERIALCOMM_H_ */