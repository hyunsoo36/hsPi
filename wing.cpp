#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include "hs_udpserver.hpp"
#include "hs_serial.hpp"

#include "hs_thread.hpp"

#define LED1 4
#define LED2 5


#define LOOP_TIME 66 // [ms]
#define SERIAL_TIME 50

using namespace std;
using namespace cv;

void *thread_udp(void *arg);
void *thread_serial(void *arg);
void *thread_cv(void *arg);

unsigned long ulTimeBegin = 0;
unsigned long ulElapsedTime = 0;

double profileTime = 0;

double serial_time = 0;

VideoCapture* cap;
Mat frame, img;

double roll_sp, pitch_sp, yaw_sp, alt_sp;

char udp_data[1024] = {0, };


int main(int argc, char** argv){
	
	int fd, data;
	pthread_t udp_thread, serial_thread, cv_thread;

	

	// thread
	if(pthread_create(&udp_thread, NULL, thread_udp, NULL)) {
		cout << "Cannot creating udp thread" << endl;
		return -1;
	}
	
	if(pthread_create(&serial_thread, NULL, thread_serial, NULL)) {
		cout << "Cannot creating serial thread" << endl;
		return -1;
	}
	
	if(pthread_create(&cv_thread, NULL, thread_cv, NULL)) {
		cout << "Cannot creating cv thread" << endl;
		return -1;
	}
	
	
	
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	
	//delay(5000);

	while (1)
	{
		
		ulTimeBegin = millis();
		
		
		
		
		
		//digitalWrite(LED1, 0);        
		//digitalWrite(LED2, 0);        
		//delay(500);
		//digitalWrite(LED1, 1);        
		//digitalWrite(LED2, 1);        
		//delay(20);
		
		//make profile
		
		roll_sp = 0;
		pitch_sp = 0;
		yaw_sp = 0;
		alt_sp = 0;
		
		if( udp_data[0] == 'a' ) {
			
			if( profileTime >= 0.0 && profileTime < 1.0 ) {
				
			}
			//roll_sp = -sin(profileTime*0.392)*5;
			//pitch_sp = cos(profileTime*0.392)*5;
			//roll_sp = -sin(profileTime*1.57)*5;
			//pitch_sp = cos(profileTime*1.57)*5;
			
			profileTime += LOOP_TIME / 1000.0;
			
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
