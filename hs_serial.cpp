#include "hs_serial.hpp"
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>

SerialhsWing::SerialhsWing() {
	
}

SerialhsWing::~SerialhsWing() {

}

int SerialhsWing::initSerial() {

	if( wiringPiSetup() == -1 ) {
		return -1;
	}
	//if((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0) {	// for Raspberry Pi
	if((fd = serialOpen("/dev/ttyS2", 115200)) < 0) {	// for Odroid
		//fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return -1;
	}
	
	serialFlush(fd);
	
	pinMode(SERIAL_LED, OUTPUT);
	
	
	
	return 1;
}
int SerialhsWing::makePacket(char* data, int len) {
	
	data_size = len;
	
	packet[0] = HS_PACKET_HEADER1;
	packet[1] = HS_PACKET_HEADER2;
	for( int i=0; i<len; i++ ) {
		packet[2+i] = data[i];
	}
	packet[2+len] = HS_PACKET_TAIL;


}

int SerialhsWing::sendPacket(int VTOL_state, int udp_err_flag) {
	

	/*
	for(int i=0; i<data_size + 3; i++) {
		//buffer[i] = serialGetchar(fd);
		cout << (unsigned int)packet[i] << " ";
	}
	cout << endl;
	*/
	
	write(fd, packet, data_size + 3);	// 3 bytes are header and tail bytes.
	
	// Blink LED
	if( VTOL_state == 2 ) {
		digitalWrite(SERIAL_LED, iSerialLed);  
		iSerialLed = (iSerialLed == 1) ? 0 : 1;
	}else if( VTOL_state == 1 || udp_err_flag == 2) {
		digitalWrite(SERIAL_LED, 1);  
	}else if( VTOL_state == 0 ) {
		digitalWrite(SERIAL_LED, 0);  
	}

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
		//cout << serial_len << " : Serial Buffer is Full" << endl;
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














