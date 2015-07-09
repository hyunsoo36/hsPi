#include "hs_serial.hpp"
#include <string.h>
#include <unistd.h>

SerialhsWing::SerialhsWing() {
	
}

SerialhsWing::~SerialhsWing() {

}

int SerialhsWing::initSerial() {

	if( wiringPiSetup() == -1 ) {
		return -1;
	}
	//if((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0) {
	if((fd = serialOpen("/dev/ttyS2", 115200)) < 0) {
		//fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return -1;
	}
	
	serialFlush(fd);
	
	pinMode(SERIAL_LED, OUTPUT);
	
	
	
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
	/*
	for(int i=0; i<HS_PACKET_LENGTH_MAX; i++) {
		
		cout << (unsigned int)packet[i] << " ";
	}
	cout << endl;
	*/
	
	//serialPuts(fd, packet);
	write(fd, packet, HS_PACKET_LENGTH_MAX);
	
	digitalWrite(SERIAL_LED, iLed);  
	iLed = (iLed == 1) ? 0 : 1;

}
int SerialhsWing::recvPacket(signed char* data) {
		
	//char recv_buffer[HS_BUFFER_LENGTH] = {0, };
	int serial_len = serialDataAvail(fd);

	//cout << "packet length : " << packet_len << endl;
	
	if( serial_len == -1 ) {
		return -1;
	}else if( serial_len == 0 ) {
		return 0;
	}else if( serial_len > HS_BUFFER_LENGTH ) {
		char tmp[1024];
		read(fd, tmp, serial_len);
		cout << serial_len << " : Serial Buffer is Full" << endl;
		return 9999;
	}
	read(fd, serial_buf, serial_len);
	memcpy( &buffer[0], &buffer[serial_len], HS_BUFFER_LENGTH-serial_len );
	memcpy( &buffer[HS_BUFFER_LENGTH-serial_len], &serial_buf[0], serial_len );
	
	/*
	for(int i=0; i<HS_BUFFER_LENGTH; i++) {
		//buffer[i] = serialGetchar(fd);
		cout << (unsigned int)buffer[i] << " ";
	}
	cout << endl;
	*/
	
	// check packet form
	if( buffer[0] == HS_PACKET_HEADER1 && buffer[1] == HS_PACKET_HEADER2 ) {	// head
		if( buffer[3+buffer[2]] == HS_PACKET_TAIL ) {	// tail
			for(int i=0; i<buffer[2]; i++) {
				data[i] = buffer[3+i];
			}
			return buffer[2];
		}else {
			return 0;
		}
		
	}else {
		return 0;
	}
		
	
	
}














