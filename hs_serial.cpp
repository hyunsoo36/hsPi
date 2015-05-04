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
		//fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return -1;
	}
	
	serialFlush(fd);
	
	return 1;
}
int SerialhsWing::makePacket(char* data, int len) {
	packet[0] = HS_PACKET_HEADER1;
	packet[1] = HS_PACKET_HEADER2;
	packet[2] = len;
	for( int i=0; i<len; i++ ) {
		packet[3+i] = data[i];
	}
	packet[3+len] = HS_PACKET_TAIL;


}

int SerialhsWing::sendPacket() {
	//serialPuts(fd, packet);
	write(fd, packet, HS_PACKET_LENGTH_MAX);

}
int SerialhsWing::recvPacket(char* data) {
		
	char recv_packet[HS_PACKET_LENGTH_MAX] = {0, };
	int packet_len = serialDataAvail(fd);

	//cout << "packet length : " << packet_len << endl;
	
	if( packet_len == -1 ) {
		return -1;
	}else if( packet_len == 0 ) {
		return 0;
	}else if( packet_len > HS_PACKET_LENGTH_MAX ) {
		serialFlush(fd);
		return 9999;
	}
	
	read(fd, recv_packet, packet_len);
	/*
	for(int i=0; i<packet_len; i++) {
		//recv_packet[i] = serialGetchar(fd);
		cout << (unsigned int)recv_packet[i] << "\t";
	}
	cout << endl;
	*/
	/*
	// check packet form
	if( recv_packet[0] == HS_PACKET_HEADER1 && recv_packet[1] == HS_PACKET_HEADER2 ) {	// head
		if( recv_packet[3+recv_packet[3]] == HS_PACKET_TAIL ) {	// tail
			for(int i=0; i<recv_packet[3]; i++) {
				data[i] = recv_packet[3+i];
			}
			return recv_packet[3];
		}else {
			return 0;
		}
		
	}else {
		return 0;
	}
		*/
	
	
}














