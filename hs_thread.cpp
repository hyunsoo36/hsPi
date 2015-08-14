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
#include "hsNavi.hpp"

#define SERIAL_LOOP_DELAY	20

#define BLOCK_COUNT	10
#define BLOCK_COUNT	10

#define MAX_COUNT 10

char rawWindow[] = "Raw Video";
char robustWindow[] = "Robust Window";

using namespace cv;

double profileTime = 0;

double serial_time = 0;

//double roll_sp, pitch_sp, yaw_sp, alt_sp;
//double roll, pitch, yaw, alt, ax, ay, az;
//double lastRoll, lastPitch;

char udp_data[1024] = {0, };

int udp_err_flag = 0;

VTOL_data wingwing2;
Phone_data phone2;

extern Command_data cmd_data;
extern HSNavi hsNavi;

double getVelocitybyRotate(double angularVelocity, double z);	// [deg/s], [m]


void *thread_cv(void *arg) {

#if 1
	
	VideoCapture cap(0);
	
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 240);//320);//640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);//240);//480);
	cap.set(CV_CAP_PROP_FPS, 10);
	cap.set(CV_CAP_PROP_EXPOSURE, 10);
	
	Mat frame_prev;
	Mat frame, frame_gray;
	Vector<Mat>img_prev;
	Vector<Mat>img;
	//Mat opticalFlowFrame = Mat(cap.get(CV_CAP_PROP_FRAME_HEIGHT), cap.get(CV_CAP_PROP_FRAME_WIDTH), CV_32FC3);
	
	cap >> frame;
	cvtColor(frame, frame_gray, CV_RGB2GRAY);
	frame_prev = frame_gray.clone();

	int h = frame.size().height / BLOCK_COUNT;
	int w = frame.size().width / BLOCK_COUNT;
	
	vector<Point2f> featurePrev, featureNext;
	
	vector<uchar> isFound;
	vector<float> err;
	Size winSize(5, 5);
	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	
	featurePrev.clear();
	for (int i=0; i<BLOCK_COUNT; i++) {
			for (int j=0; j<BLOCK_COUNT; j++) {
				featurePrev.push_back(Point2f( i * w + (w/2), j * h + (h/2) ));
			}
	}
	
	
	featureNext = featurePrev;
	
	// Timming Variable
	clock_t tStart, tEnd; 
	double tElapsed;

	while (1) 
	{
		tEnd = millis();
		tElapsed = tEnd - tStart;
		tStart = millis();
		float fps = 1000.0/tElapsed;
		//cout << "Elapsed Time : " << tElapsed << "ms" << endl;
		//cout << "FPS : " << 1000.0/tElapsed << "fps" << endl;
		
		
		cap >> frame;
		
		cvtColor(frame, frame_gray, CV_RGB2GRAY);

		
		int sum_x=0, sum_y=0;
		int flow_cnt = 0;
		calcOpticalFlowPyrLK(frame_prev, frame_gray, featurePrev, featureNext, isFound, err, winSize, 3, termcrit, 0, 0.01);
		
		
		
		for (int i=0; i<featurePrev.size(); i++) {
			float flow_x = featureNext[i].x - featurePrev[i].x;
			float flow_y = featureNext[i].y - featurePrev[i].y;
			if( flow_x*flow_x + flow_y*flow_y < w*h ) {
				line(frame, featurePrev[i], featureNext[i], Scalar(0, 0, 255), 1, 1, 0);
				sum_x += flow_x;
				sum_y += flow_y;
				flow_cnt ++;
			}
		}
		
		float alt = wingwing2.alt * 0.01;	// [cm] -> [m]
		
		float vel_ofx = (((float)sum_x/(float)(flow_cnt))*fps) * (alt * 0.828) / 240.0f;
		float vel_ofy = (((float)sum_y/(float)(flow_cnt))*fps) * (alt * 0.828) / 240.0f;
		
		hsNavi.setOpticalFlowResult( vel_ofx, vel_ofy );
		
	
		//goodFeaturesToTrack(frame_gray, featurePrev, MAX_COUNT, 0.01, 5, Mat(), 3, 0, 0.04);
		

		frame_prev = frame_gray.clone();
		
		//Mat frame_x2;
		//pyrUp(frame, frame_x2, Size(frame.cols*2, frame.rows*2));
		//imshow("Ori", frame);
		
		
		

		if(waitKey(1) == 27) {
			break;
		}
		
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
		if( sendCnt > 0 ) {	// sand freq : recv freq = 1 : 2
			sendCnt = 0;
			
			//pthread_mutex_lock(&mutex);
			sendData[0] = (char)(( ((short)(phone2.VTOL_state)) & 0x00FF ) >> 0);
			sendData[1] = (char)(( ((short)(cmd_data.roll_sp)) & 0x00FF ) >> 0);
			sendData[2] = (char)(( ((short)(cmd_data.pitch_sp)) & 0x00FF ) >> 0);
			sendData[3] = (char)(( ((short)(cmd_data.yaw_sp)) & 0x00FF ) >> 0);
			sendData[4] = (char)(( ((short)(cmd_data.alt_sp*10.0)) & 0x00FF ) >> 0);
			//pthread_mutex_unlock(&mutex);
			
			hsSerial->makePacket(sendData, 5);
			hsSerial->sendPacket(phone2.VTOL_state, udp_err_flag);
			
			
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
			
			wingwing2.lastRoll = wingwing2.roll;
			wingwing2.lastPitch = wingwing2.pitch;
			
			wingwing2.roll = (((short)recvData[0] << 8) | ((unsigned char)recvData[1] << 0 )) / 100.0;
			wingwing2.pitch = (((short)recvData[2] << 8) | ((unsigned char)recvData[3] << 0 )) / 100.0;
			wingwing2.yaw = (((short)recvData[4] << 8) | ((unsigned char)recvData[5] << 0 )) / 10.0;
			wingwing2.alt = (((short)recvData[6] << 8) | ((unsigned char)recvData[7] << 0 )) / 10.0;
			wingwing2.ax = (((short)recvData[8] << 8) | ((unsigned char)recvData[9] << 0 )) / 100.0;
			wingwing2.ay = (((short)recvData[10] << 8) | ((unsigned char)recvData[11] << 0 )) / 100.0;
			wingwing2.az = (((short)recvData[12] << 8) | ((unsigned char)recvData[13] << 0 )) / 100.0;
			
			//cout << (short)recvData[0] << "\t" << (unsigned char)recvData[1] << "\t" << roll << endl;
			//cout << wingwing2.roll << "\t" << wingwing2.pitch << "\t" << wingwing2.yaw << "\t" << wingwing2.alt << "\t" << wingwing2.ax << "\t"
			//	<< wingwing2.ay << "\t" << wingwing2.az << endl;
			
			
			//for(int i=0; i<recvDataLen; i++) {
			//	cout << (int)recvData[i] <<  "  ";
			//}
			//cout << "" << endl;
			
			double rollVelocity = (wingwing2.roll - wingwing2.lastRoll) / (SERIAL_LOOP_DELAY / 1000.0);
			double pitchVelocity = (wingwing2.pitch - wingwing2.lastPitch) / (SERIAL_LOOP_DELAY / 1000.0);
			
			cout << hsNavi.vel_ax << "\t" << hsNavi.vel_ay << "\t" 
			<< hsNavi.vel_vy << "\t" << hsNavi.vel_vx <<"\t" 
			<< hsNavi.hori_vel_x << "\t" << getVelocitybyRotate( pitchVelocity, wingwing2.alt/100.0 )<< endl;
			
		}
		
#ifdef WIRELESS_DEBUGGING
		delay(5);
#else
		delay(SERIAL_LOOP_DELAY);
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
		
		if( phone2.VTOL_state == 0 && (((int)((signed char)udp_data[3]) == 1) || ((int)((signed char)udp_data[3]) == 2)) ) {	// if touch take off button
			//cout << "STATE : READY Mode.." << endl;
		}else if( phone2.VTOL_state == 1 && (int)((signed char)udp_data[3]) == 2) {	// if touch take off button
			//cout << "STATE : FLYING Mode.." << endl;
			
			hsNavi.landingInitalize();
			//initOdometrybyAccel(wingwing2.ax, wingwing2.ay, wingwing2.az-1.0);
			
		}else if( phone2.VTOL_state == 2 && phone2.VTOL_state != (int)((signed char)udp_data[3])) { // if touch langing button
			//cout << "STATE : WAIT Mode.." << endl;
		}
		
		phone2.VTOL_state = 0;	// for testing communication
		
		//pthread_mutex_lock(&mutex);
		phone2.roll_sp = (double)((int)((signed char)udp_data[0]));
		phone2.pitch_sp = (double)((int)((signed char)udp_data[1]));
		phone2.alt_sp = (double)((int)((signed char)udp_data[2]));
		phone2.VTOL_state = (int)((signed char)udp_data[3]);
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

double getVelocitybyRotate(double angularVelocity, double z) {	// [deg/s], [m]
	return tan( angularVelocity / 57.23 ) * z ;
}

#endif
