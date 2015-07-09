#include "hs_udpserver.hpp"
#include <sys/ioctl.h>


UDPServer::UDPServer() {

}

UDPServer::~UDPServer() {

}

int UDPServer::CreateSocket() {
	int opt = 1;
	u_long inputMode = 1;
	
	// Create socket
	sd = socket(PF_INET, SOCK_DGRAM, 0);
	setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
	
	//ioctlsocket(sd, FIOBIO, &inputMode);
	
	if(sd == -1) {
		return 0;
	}
	return 1;
	

}

int UDPServer::BindSocket() {
	// Bind socket to local address and port
	memset(&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);		// Default address
	serverAddr.sin_port = htons(HS_UDP_PORT);				// We assume port 
	if(bind(sd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {	
		return 0;
	}
	return 1;

}

int UDPServer::ReceiveData(char* udp_data) {
	clAddrLen = sizeof(clientAddr);
	nr = recvfrom(sd, udp_data, 20, MSG_DONTWAIT, (struct sockaddr*)&clientAddr, &clAddrLen);	
	udp_data[nr] = 0;
	//fputs(buffer, stdout);
	//fputc('\n', stdout);
	//sendto(sd, udp_data, nr, 0, (struct sockaddr*)&clientAddr, sizeof(clientAddr));
	return nr;

}
