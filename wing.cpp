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
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include "wiringPi.h"
#include "hs_udpserver.hpp"
#include "hs_serial.hpp"

//#include "hs_thread.hpp"

#define LED1 1
#define LED2 5

//#define WIRELESS_DEBUGGING

#define LOOP_TIME 50 // [ms]
#define SERIAL_TIME 50

/////////////////////////////////////
#define MAX_COUNT	100
#define AOVWIDTH	53.5		// Angle of view(width)
#define AOVHEIGHT	41.5		// Angle of view(height)

char rawWindow[] = "Raw Video";
char robustWindow[] = "Robust Window";
char imageFileName[32];
long imageIndex = 0;
char keyPressed;

int velocity_x, velocity_y;

/////////////////////////////////////
using namespace std;
using namespace cv;

void *thread_udp(void *arg);
void *thread_serial(void *arg);
void *thread_cv(void *arg);

double getVelocitybyRotate(double angularVelocity, double z);	// [deg/s], [m]

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


unsigned long ulTimeBegin = 0;
unsigned long ulElapsedTime = 0;

double profileTime = 0;

double serial_time = 0;

VideoCapture* cap;
Mat frame, img;

double roll_sp, pitch_sp, yaw_sp, alt_sp;
double roll, pitch, yaw, alt, ax, ay, az;
double lastRoll, lastPitch;

char udp_data[1024] = {0, };

int udp_err_flag = 0;
int VTOL_state = 0;


int main(int argc, char** argv){
	
	int fd, data;
	pthread_t udp_thread, serial_thread, cv_thread;
	
	// UDP waiting
	UDPServer *tmp_udp = new UDPServer();
	while( tmp_udp->CreateSocket() == 0 ) {
			delay(50);
	}
	
	
	
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
	cout.fill(' ');
	cout.precision(3);
	

	while (1)
	{
		
		ulTimeBegin = millis();
		
	
		
		double rollVelocity = (roll - lastRoll) / (LOOP_TIME / 1000.0);
		double pitchVelocity = (pitch - lastPitch) / (LOOP_TIME / 1000.0);
		getVelocitybyRotate( rollVelocity, alt/100.0 );
		getVelocitybyRotate( pitchVelocity, alt/100.0 );
		
		//cout << "roll velocity : " << (double)getVelocitybyRotate( rollVelocity, alt/100.0 ) 
		//	<< "\tpitch velocity : " << (double)getVelocitybyRotate( pitchVelocity, alt/100.0 ) << endl;

		
		//make profile
		
		//roll_sp = 4;
		//pitch_sp = 5;
		//yaw_sp = 0;
		//alt_sp = 0;
		
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
	
	pthread_mutex_destroy(&mutex);
	
	return 0;
}

double getVelocitybyRotate(double angularVelocity, double z) {	// [deg/s], [m]
	return tan( angularVelocity / 57.23 ) * z ;
}
void *thread_cv(void *arg) {

#if 1

	double hroll, hpitch, hyaw, halt;
	double prev_hroll, prev_hpitch;
	double dx;
	double Vr_x, Vr1_x, Vr2_x, Vr_y, Vr1_y, Vr2_y;
	
	
	VideoCapture cap(0);
	
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 240);//320);//640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);//240);//480);
	Mat frame, grayFrames, rgbFrames, prevGrayFrame, robustFrames, testFrames;
	Mat opticalFlow = Mat(cap.get(CV_CAP_PROP_FRAME_HEIGHT), cap.get(CV_CAP_PROP_FRAME_HEIGHT), CV_32FC3);

	vector<Point2f> points1;
	vector<Point2f> points2;

	Point2f diff;

	vector<uchar> status, ransac_status;
	vector<float> err;

	RNG rng(12345);
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),	rng.uniform(0, 255));
	bool needToInit = true;

	int i, k, vel_cnt = 1, vel_cnt1 = 1;
	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	Size subPixWinSize(13, 13), winSize(13, 13);
	namedWindow(rawWindow, CV_WINDOW_AUTOSIZE);
//	namedWindow(robustWindow, CV_WINDOW_AUTOSIZE);

	double angle;

	double nvel_x, nvel_y;
	double rvel_x, rvel_y;

	double p2m_width, p2m_height;

	//OpenCV 화면에 글자쓰기
	char s_output_result[50];
	CvFont font;

	clock_t start, finish; //수행시간체크
	double duration;

	while (1) 
	{
		start=clock();
		cap >> frame;
		hroll = roll;
		hpitch = pitch;
		halt = alt;
		p2m_width = halt * tan(AOVWIDTH/2) / frame.cols/2 ;
		p2m_height = halt * tan(AOVHEIGHT/2) / frame.rows/2 ;
		
		frame.copyTo(rgbFrames);
		frame.copyTo(robustFrames);
		cvtColor(rgbFrames, grayFrames, CV_BGR2GRAY);
		
		if (needToInit) {
			goodFeaturesToTrack(grayFrames, points1, MAX_COUNT, 0.01, 5, Mat(), 3, 0, 0.04);
//			cornerSubPix(grayFrames, points1, subPixWinSize, Size(-1, -1), termcrit);
			Vr1_x = tan(hroll) * halt;
			Vr1_y = tan(hpitch) * halt;
			needToInit = false;
		}
		
		else if (!points2.empty()) {
			calcOpticalFlowPyrLK(prevGrayFrame, grayFrames, points2, points1, status, err, winSize, 3, termcrit, 0, 0.001);
		
//			testFrames = findFundamentalMat(points2, points1, FM_RANSAC, 3, 0.99, ransac_status);
			Vr_x = Vr1_x - Vr2_x;
			Vr_y = Vr1_y - Vr2_y;
			
			for (i = k = 0; i < points2.size(); i++){
				
				if(status[i] == 1){
//					line(rgbFrames, points1[i], points2[i], Scalar(0, 0, 255), 1, 1, 0);
//					circle(rgbFrames, points1[i], 2, Scalar(255, 0, 0), 1, 1, 0);
					nvel_x = nvel_x + (points1[i].x - points2[i].x);
					nvel_y = nvel_y + (points1[i].y - points2[i].y);
					vel_cnt++;
				}
				
				line(rgbFrames, points1[i], points2[i], Scalar(0, 0, 255), 1, 1, 0);
//				circle(rgbFrames, points1[i], 2, Scalar(255, 0, 0), 1, 1, 0);

//				cout << sqrt( pow((points2[i].x - points1[i].x), 2) + pow((points2[i].y - points1[i].y), 2) ) << endl;

				points1[k++] = points1[i];

			}

			goodFeaturesToTrack(grayFrames, points1, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
//			cornerSubPix(grayFrames, points1, subPixWinSize, Size(-1, -1), termcrit);
			Vr1_x = tan(hroll) * halt;
			Vr1_y = tan(hpitch) * halt;
		}
		nvel_x = nvel_x / vel_cnt;
		nvel_y = nvel_y / vel_cnt;
		
		velocity_x = nvel_x - Vr_x;
		velocity_y = nvel_y - Vr_y;

		cout << hroll << " " << hpitch << " " << nvel_x << " " << nvel_y << " " << Vr_x << " " << Vr_y
		<< " " <<velocity_x << " " << velocity_y << endl;

		vel_cnt = vel_cnt1 = 1;
		nvel_x = nvel_y = 0;
		imshow(rawWindow, rgbFrames);
//		imshow(robustWindow, robustFrames);
		swap(points2, points1);
		points1.clear();
		
		Vr2_x = Vr1_x;
		Vr2_y = Vr1_y;
		Vr1_x = Vr1_y = 0;
		
		
		grayFrames.copyTo(prevGrayFrame);

		keyPressed = waitKey(5);
		if (keyPressed == 27) {
			break;
		} else if (keyPressed == 'r') {
			opticalFlow = Mat(cap.get(CV_CAP_PROP_FRAME_HEIGHT),
				cap.get(CV_CAP_PROP_FRAME_HEIGHT), CV_32FC3);
		}

		finish=clock();

//		duration=(double)(finish-start)/CLOCKS_PER_SEC;

//		cout << duration << endl;
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
