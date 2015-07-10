#ifndef _HS_EXTERN_H__
#define _HS_EXTERN_H__

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>

using namespace std;

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
