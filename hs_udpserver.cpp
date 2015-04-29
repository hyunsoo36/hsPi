#include "hs_udpserver.hpp"


UDPServer::UDPServer() {

}

UDPServer::~UDPServer() {

}

int UDPServer::CreateSocket() {
	int opt = 1;
	
	// Create socket
	sd = socket(PF_INET, SOCK_DGRAM, 0);
	setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
	
	if(sd == -1) {
		return 0;
	}
	

}

int UDPServer::BindSocket() {
	// Bind socket to local address and port
	memset(&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);		// Default address
	serverAddr.sin_port = htons(HS_UDP_PORT);				// We assume port 9949
	if(bind(sd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {	
		return 0;
	}
	

}

char* UDPServer::ReceiveData() {
	clAddrLen = sizeof(clientAddr);
	nr = recvfrom(sd, buffer, 20, 0, (struct sockaddr*)&clientAddr, &clAddrLen);	
	buffer[nr] = 0;
	//fputs(buffer, stdout);
	//fputc('\n', stdout);
	sendto(sd, buffer, nr, 0, (struct sockaddr*)&clientAddr, sizeof(clientAddr));
	return buffer;

}
