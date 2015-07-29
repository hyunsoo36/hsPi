#if 1
#include "hs_thread.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include "wiringPi.h"
#include "hs_udpserver.hpp"
#include "hs_serial.hpp"

char rawWindow[] = "Raw Video";
char robustWindow[] = "Robust Window";

using namespace cv;

double profileTime = 0;

double serial_time = 0;

double roll_sp, pitch_sp, yaw_sp, alt_sp;
double roll, pitch, yaw, alt, ax, ay, az;
double lastRoll, lastPitch;

char udp_data[1024] = {0, };

int udp_err_flag = 0;
int VTOL_state = 0;

void *thread_cv(void *arg) {

#if 0
	
	VideoCapture cap(0);
	
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 240);//320);//640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);//240);//480);
	cap.set(CV_CAP_PROP_FPS, 10);
	cap.set(CV_CAP_PROP_EXPOSURE, 10);
	
	Mat frame;
	Mat grayFrame, rgbFrame, prevGrayFrame;
	Mat opticalFlowFrame = Mat(cap.get(CV_CAP_PROP_FRAME_HEIGHT), cap.get(CV_CAP_PROP_FRAME_WIDTH), CV_32FC3);
	

	int i, k, vel_cnt = 1, vel_cnt1 = 1;
	//TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);

	namedWindow(rawWindow, CV_WINDOW_AUTOSIZE);


	char str[50];
	CvFont font;

	// Timming Variable
	clock_t tStart, tEnd; 
	double tElapsed;

	while (1) 
	{
		tEnd = millis();
		tElapsed = tEnd - tStart;
		tStart = millis();
		//cout << "Elapsed Time : " << tElapsed << "ms" << endl;
		//cout << "FPS : " << 1000.0/tElapsed << "fps" << endl;
		
		
		
		cap >> frame;
		
		//cvtColor(frame, grayFrame, CV_BGR2GRAY);
		

		//calcOpticalFlowPyrLK(prevGrayFrame, grayFrames, points2, points1, status, err, winSize, 3, termcrit, 0, 0.001);
		

		imshow(rawWindow, frame);

		//grayFrame.copyTo(prevGrayFrame);
		
		waitKey(1);
		
	}
#endif

}

void *thread_serial(void *arg) {
	
	SerialhsWing *hsSerial = new SerialhsWing();
	int sendCnt = 0;

	if( hsSerial->initSerial() == -1 ) {
		cout << "Wiring Pi init failed" << endl;
		
	}else {
		cout << "Wiring Pi init success" << endl;
	}
	
	char sendData[8] = {0, };
	signed char recvData[HS_RECV_DATA_LENGTH] = {0, };
	int recvDataLen;
	
	while(1) {
		if( sendCnt > 0 ) {	// sand : recv = 1 : 2
			sendCnt = 0;
			
			//pthread_mutex_lock(&mutex);
			sendData[0] = (char)(( ((short)(VTOL_state)) & 0x00FF ) >> 0);
			sendData[1] = (char)(( ((short)(roll_sp)) & 0x00FF ) >> 0);
			sendData[2] = (char)(( ((short)(pitch_sp)) & 0x00FF ) >> 0);
			sendData[3] = (char)(( ((short)(yaw_sp)) & 0x00FF ) >> 0);
			sendData[4] = (char)(( ((short)(alt_sp*10.0)) & 0x00FF ) >> 0);
			//pthread_mutex_unlock(&mutex);
			
			hsSerial->makePacket(sendData, 5);
			hsSerial->sendPacket(VTOL_state, udp_err_flag);
			
		}else {
			sendCnt ++;
		}
		
		
		recvDataLen = hsSerial->recvPacket(recvData);
		
		if( recvDataLen == -1 ) {
			cout << "serial error..." << endl;
		}else if( recvDataLen == 0 ) {
			//cout << "serial 0" << endl;
		}else if( recvDataLen == 9999 ) {
			//cout << "serial 9999" << endl;
		}else {
			
			lastRoll = roll;
			lastPitch = pitch;
			
			roll = (((short)recvData[0] << 8) | ((unsigned char)recvData[1] << 0 )) / 10.0;
			pitch = (((short)recvData[2] << 8) | ((unsigned char)recvData[3] << 0 )) / 10.0;
			yaw = (((short)recvData[4] << 8) | ((unsigned char)recvData[5] << 0 )) / 10.0;
			alt = (((short)recvData[6] << 8) | ((unsigned char)recvData[7] << 0 )) / 10.0;
			ax = (((short)recvData[8] << 8) | ((unsigned char)recvData[9] << 0 )) / 100.0;
			ay = (((short)recvData[10] << 8) | ((unsigned char)recvData[11] << 0 )) / 100.0;
			az = (((short)recvData[12] << 8) | ((unsigned char)recvData[13] << 0 )) / 100.0;
			
			//cout << (short)recvData[0] << "\t" << (unsigned char)recvData[1] << "\t" << roll << endl;
			//cout << roll << "\t" << pitch << "\t" << yaw << "\t" << alt << "\t" << ax << "\t"
			//	<< ay << "\t" << az << endl;
			
			
			//for(int i=0; i<recvDataLen; i++) {
			//	cout << (int)recvData[i] <<  "  ";
			//}
			//cout << "" << endl;
			
			
		}
		
#ifdef WIRELESS_DEBUGGING
		delay(5);
#else
		delay(20);
#endif



	}
}

void *thread_udp(void *arg) {
	
	delay(500);
	
	int recvLen = 0;
	UDPServer *udp = new UDPServer();
	//char *data;

	if( udp->CreateSocket() == 0 ) {
		cout << "socket creating error " << endl;
		udp_err_flag = 1;	
		return NULL;
	}
	cout << "create listening Socket" << endl;

	if( udp->BindSocket() == 0 ) {
		cout << "binding error" << endl;
		return NULL;
	}
	cout << "bind success " << endl;
	
	udp_err_flag = 2;	// OK flag
	
	while(1) {
		
		recvLen = udp->ReceiveData(udp_data);
		
		if( VTOL_state == 0 && (((int)((signed char)udp_data[3]) == 1) || ((int)((signed char)udp_data[3]) == 2)) ) {	// if touch take off button
			cout << "STATE : READY Mode.." << endl;
		}else if( VTOL_state == 1 && (int)((signed char)udp_data[3]) == 2) {	// if touch take off button
			cout << "STATE : FLYING Mode.." << endl;
		}else if( VTOL_state == 2 && VTOL_state != (int)((signed char)udp_data[3])) { // if touch langing button
			cout << "STATE : WAIT Mode.." << endl;
		}
		
		VTOL_state = 0;	// for testing communication
		
		//pthread_mutex_lock(&mutex);
		roll_sp = (double)((int)((signed char)udp_data[0]));
		pitch_sp = (double)((int)((signed char)udp_data[1]));
		alt_sp = (double)((int)((signed char)udp_data[2]));
		VTOL_state = (int)((signed char)udp_data[3]);
		//pthread_mutex_unlock(&mutex);
		
		
		
/*
		cout << "data[0] : " << (int)((signed char)udp_data[0])  << "\t";
		cout << "data[1] : " << (int)((signed char)udp_data[1])  << "\t";
		cout << "data[2] : " << (int)((signed char)udp_data[2])  << "\t";
		cout << "data[3] : " << (int)((signed char)udp_data[3]) << endl;
*/
		delay(10);
		
	}
}

#endif
