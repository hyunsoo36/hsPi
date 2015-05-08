#include "hs_thread.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include "hs_udpserver.hpp"
#include "hs_serial.hpp"

extern VideoCapture* cap;
extern Mat frame, img;

extern double roll_sp, pitch_sp, yaw_sp, alt_sp;

void *thread_cv(void *arg) {
	/*
	cap = new VideoCapture(-1);

	if(!cap->isOpened()) {
		cout << "Cannot open camera" << endl;
		return -1;
	}

	cap->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap->set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	namedWindow("Output",CV_WINDOW_AUTOSIZE);
	*/
	while(1) {
		/*
		
		if( run_cv_th == true ) {
			bool bSuccess = cap->read(frame);

			if (!bSuccess)
			{
				cout << "Cannot read a frame from camera" << endl;
				break;
			}
		}
		
		*/
		
		delay(25);

	}

}
void *thread_serial(void *arg) {

	SerialhsWing *hsSerial = new SerialhsWing();
	int sendCnt = 0;

	if( hsSerial->initSerial() == -1 ) {
		cout << "Wiring Pi init failed" << endl;
	}

	while(1) {
		
		if( sendCnt > 0 ) {
			sendCnt = 0;
			
			char tmpStr[8];
			tmpStr[0] = (char)(( ((short)(roll_sp*10.0)) & 0xFF00 ) >> 8);
			tmpStr[1] = (char)(( ((short)(roll_sp*10.0)) & 0x00FF ) >> 0);
			tmpStr[2] = (char)(( ((short)(pitch_sp*10.0)) & 0xFF00 ) >> 8);
			tmpStr[3] = (char)(( ((short)(pitch_sp*10.0)) & 0x00FF ) >> 0);
			tmpStr[4] = (char)(( ((short)(yaw_sp*10.0)) & 0xFF00 ) >> 8);
			tmpStr[5] = (char)(( ((short)(yaw_sp*10.0)) & 0x00FF ) >> 0);
			tmpStr[6] = (char)(( ((short)(alt_sp*10.0)) & 0xFF00 ) >> 8);
			tmpStr[7] = (char)(( ((short)(alt_sp*10.0)) & 0x00FF ) >> 0);
			hsSerial->makePacket(tmpStr, 8);
			hsSerial->sendPacket();
			
		}else {
			sendCnt ++;
		}
		
		char recvData[HS_PACKET_LENGTH_MAX];
		int recvDataLen;
		recvDataLen = hsSerial->recvPacket(recvData);
		if( recvDataLen == -1 ) {
			cout << "serial error..." << endl;
		}else if( recvDataLen == 0 ) {
			
		}else if( recvDataLen == 9999 ) {
			
		}else {
			/*
			for(int i=0; i<recvDataLen; i++) {
				cout << (int)recvData[i] <<  "  ";
			}
			cout << "" << endl;
			*/
		}

		delay(25);


	}
}

void *thread_udp(void *arg) {

	UDPServer *udp = new UDPServer();
	//char *data;

	if( udp->CreateSocket() == 0 ) {
		cout << "socket creating error " << endl;
		return NULL;
	}
	cout << "create listening Socket" << endl;

	if( udp->BindSocket() == 0 ) {
		cout << "binding error" << endl;
		return NULL;
	}
	cout << "bind success " << endl;

	while(1) {
		udp->ReceiveData(udp_data);
		cout << "udp : " << udp_data << endl;
		
		delay(25);
		
	}
}
