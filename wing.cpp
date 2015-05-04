#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <pthread.h>
#include "hs_udpserver.hpp"
#include "hs_serial.hpp"

#define LED1 4
#define LED2 5


#define LOOP_TIME 66 // [ms]

using namespace std;
using namespace cv;

void* thread_udp(void *arg);

unsigned long ulTimeBegin = 0;
unsigned long ulElapsedTime = 0;

int main(int argc, char** argv){
	
	int fd, data;
	pthread_t udp_thread;
	
	// thread
	if(pthread_create(&udp_thread, NULL, thread_udp, NULL)) {
		cout << "Cannot creating thread" << endl;
		return -1;
	}
	
	VideoCapture cap(-1);
	if (!cap.isOpened())
	{
		cout << "Cannot open camera" << endl;
		return -1;
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	namedWindow("Output",CV_WINDOW_AUTOSIZE);

	SerialhsWing *hsSerial = new SerialhsWing();

	if( hsSerial->initSerial() == -1 ) {
		cout << "Wiring Pi init failed" << endl;
	}
	
	Mat frame, img;
	
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);


	while (1)
	{
		
		ulTimeBegin = millis();
		
		bool bSuccess = cap.read(frame);
		

		if (!bSuccess)
		{
			cout << "Cannot read a frame from camera" << endl;
			break;
		}
		
		//pyrDown(frame, img, Size(frame.cols/2, frame.rows/2));

		//imshow("Output", frame);

		//if (waitKey(33) == 27)
		//{
		//	cout << "Exit" << endl;
		//	break;
		//}
		
		
		
		
		
		//digitalWrite(LED1, 0);        
		//digitalWrite(LED2, 0);        
		//delay(500);
		//digitalWrite(LED1, 1);        
		//digitalWrite(LED2, 1);        
		//delay(20);
		
		
		double roll = 10.3, pitch = 2.5, yaw = 36.4, alt = 80.0;
		char tmpStr[8];
		tmpStr[0] = (char)(( ((short)(roll*10.0)) & 0xFF00 ) >> 8);
		tmpStr[1] = (char)(( ((short)(roll*10.0)) & 0x00FF ) >> 0);
		tmpStr[2] = (char)(( ((short)(pitch*10.0)) & 0xFF00 ) >> 8);
		tmpStr[3] = (char)(( ((short)(pitch*10.0)) & 0x00FF ) >> 0);
		tmpStr[4] = (char)(( ((short)(yaw*10.0)) & 0xFF00 ) >> 8);
		tmpStr[5] = (char)(( ((short)(yaw*10.0)) & 0x00FF ) >> 0);
		tmpStr[6] = (char)(( ((short)(alt*10.0)) & 0xFF00 ) >> 8);
		tmpStr[7] = (char)(( ((short)(alt*10.0)) & 0x00FF ) >> 0);
		hsSerial->makePacket(tmpStr, 8);
		hsSerial->sendPacket();
		
		char recvData[HS_PACKET_LENGTH_MAX];
		int recvDataLen;
		recvDataLen = hsSerial->recvPacket(recvData);
		if( recvDataLen == -1 ) {
			//cout << "serial error..." << endl;
		}else if( recvDataLen == 0 ) {
			
		}else if( recvDataLen == 9999 ) {
			//cout << "Serial Buffer is Full" << endl;
		}else {
			for(int i=0; i<3; i++) {
				//cout << (int)recvData[i] <<  "\t";
			}
			//cout << "" << endl;
		}
		
		
		// Loop Timming
		ulElapsedTime = millis() - ulTimeBegin;
		if( ulElapsedTime < LOOP_TIME ) {
			delay( LOOP_TIME - ulElapsedTime );
		}else if( ulElapsedTime > LOOP_TIME ) {
			cout << "Wiarnning!! --- : ";
			cout << ulElapsedTime << endl;
			//cout <<	1000.0/(double)(millis() - ulTimeBegin) << " FPS" << endl;
		}
		//cout <<	1000.0/(double)(millis() - ulTimeBegin) << " FPS" << endl;
		//cout << millis() - ulTimeBegin << endl;
		
	}
	return 0;
}

void *thread_udp(void *arg) {

	UDPServer *udp = new UDPServer();
	char *data;

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
		
		data = udp->ReceiveData();
		cout << "udp : " << data << endl;

	}
}

