#ifndef _HS_UDPSERVER_H__
#define _HS_UDPSERVER_H__

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define HS_UDP_PORT 		9949





class UDPServer {
private:


	// Declaration and definition
	int i; 			
	int sd;		// Socket descriptor
	int nr;		// Number of bytes received

	char buffer[20];						// Data buffer
	struct sockaddr_in serverAddr;	// Socket address
	struct sockaddr_in clientAddr;	// Client address
	socklen_t clAddrLen;							// Length of client Address
	
public:
	UDPServer();
	~UDPServer();
	
	int CreateSocket();
	int BindSocket();
	int ReceiveData(char* udp_data);

	
	
	
	
};





#endif
