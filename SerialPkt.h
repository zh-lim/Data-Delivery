/*
 * SerialPkt.h
 * defines the structure of the packet to be sent from arduino to Raspberry 
 *
 * Created: 9/9/2015 8:45:36 PM
 *  Author: Lim Zi H
 */ 


#ifndef SERIALPKT_H_
#define SERIALPKT_H_

#include "SerialProtocol.h"

/*
Singular Packet Structure

+------+--------+--------+------------+
|   M  |   LEN  |  DATA  |  CHECKSUM  |
+------+--------+--------+------------+

M = Marker
LEN = Data Length

*/

typedef struct  
{
	byte_t PacketHeader;					// contains the bit pattern to notify the type of data
	byte_t Length;							// Length of data being transmitted
	byte_t Data [DATA_LENGTH];				// Array containing data
	byte_t Checksum;						// Checksum for errors
} DataPacket;

#endif /* SERIALPKT_H_ */