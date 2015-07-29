#ifndef _HS_SERIAL_HPP__
#define _HS_SERIAL_HPP__


using namespace std;

#define HS_BUFFER_LENGTH 18

// Minimum of packet size is (data size + 3).
// Additional sizes(3) are headers and tail bytes.
#define HS_PACKET_LENGTH_MAX 8		// send

// We choice 2 header bytes and 1 tail byte.
#define HS_PACKET_HEADER1 0xEF
#define HS_PACKET_HEADER2 0xFE
#define HS_PACKET_TAIL 0xFF

#define HS_RECV_DATA_LENGTH		14

// Use odroid GPIO using WiringPi Lib.
#define SERIAL_LED 7

class SerialhsWing {
private:
	int fd;
	
	// Sand part variable
	char packet[HS_PACKET_LENGTH_MAX];
	int data_size;
	
	// Recv part variable
	char serial_buf[HS_BUFFER_LENGTH];
	char buffer[HS_BUFFER_LENGTH];
	
public:
	int iSerialLed;
	
public:
	SerialhsWing();
	~SerialhsWing();

	int initSerial();
	int makePacket(char* data, int len);
	int sendPacket(int VTOL_state, int udp_err_flag);
	
	int recvPacket(signed char* data);


};





#endif
