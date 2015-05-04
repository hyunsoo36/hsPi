#ifndef _HS_SERIAL_HPP__
#define _HS_SERIAL_HPP__

#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
using namespace std;

#define HS_PACKET_LENGTH_MAX 12
#define HS_PACKET_HEADER1 0xEF
#define HS_PACKET_HEADER2 0xFE
#define HS_PACKET_TAIL 0xFF


class SerialhsWing {
private:
	int fd;
	char packet[HS_PACKET_LENGTH_MAX];

public:
	SerialhsWing();
	~SerialhsWing();

	int initSerial();
	int makePacket(char* data, int len);
	int sendPacket();
	
	int recvPacket(char* data);


};





#endif
