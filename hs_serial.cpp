#include "hs_serial.hpp"

SerialhsWing::SerialhsWing() {
	
}

SerialhsWing::~SerialhsWing() {

}

int SerialhsWing::initSerial() {

	if( wiringPiSetup() == -1 ) {
		return -1;
	}
	if((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0) {
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return -1;
	}

	return 1;
}
int SerialhsWing::makePacket(char* data, int len) {
	packet[0] = HS_PACKET_HEADER1;
	packet[1] = HS_PACKET_HEADER2;
	for( int i=0; i<len; i++ ) {
		packet[2+i] = data[i];
	}
	packet[2+len] = HS_PACKET_TAIL;


}

int SerialhsWing::sendPacket() {
	serialPuts(fd, packet);

}


