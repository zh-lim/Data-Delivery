/*
 * SerialAPI.h
 *
 * Created: 9/10/2015 10:42:25 PM
 *  Author: Lim Zi H
 */ 


#ifndef SERIALAPI_H_
#define SERIALAPI_H_

void SerialInit(long baudrate);

void SerialSend(byte_t * sendData);



#endif /* SERIALAPI_H_ */