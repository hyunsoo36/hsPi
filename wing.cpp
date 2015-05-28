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
#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include "hs_udpserver.hpp"
#include "hs_serial.hpp"

//#include "hs_thread.hpp"

#define LED1 4
#define LED2 5

#define WIRELESS_DEBUGGING

#define LOOP_TIME 66 // [ms]
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

unsigned long ulTimeBegin = 0;
unsigned long ulElapsedTime = 0;

double profileTime = 0;

double serial_time = 0;

VideoCapture* cap;
Mat frame, img;

double roll_sp, pitch_sp, yaw_sp, alt_sp;
double roll, pitch, yaw, alt, ax, ay, az;

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

#if 0
	double hroll, hpitch, hyaw, halt;
	double prev_hroll, prev_hpitch;
	double dx;
	double Vr_x, Vr1_x, Vr2_x, Vr_y, Vr1_y, Vr2_y;
	
	
	VideoCapture cap(0);
	
	
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);//640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);//480);
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
//	namedWindow(rawWindow, CV_WINDOW_AUTOSIZE);
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
		
		if (needToInit) 
		{
			goodFeaturesToTrack(grayFrames, points1, MAX_COUNT, 0.01, 5, Mat(), 3, 0, 0.04);
//			cornerSubPix(grayFrames, points1, subPixWinSize, Size(-1, -1), termcrit);
			Vr1_x = tan(hroll) * halt;
			Vr1_y = tan(hpitch) * halt;
			needToInit = false;
		}
		
		else if (!points2.empty()) 
		{
//			cout << "\n\n\nCalculating  calcOpticalFlowPyrLK\n\n\n\n\n";
			calcOpticalFlowPyrLK(prevGrayFrame, grayFrames, points2, points1, status, err, winSize, 3, termcrit, 0, 0.001);
		
//			testFrames = findFundamentalMat(points2, points1, FM_RANSAC, 3, 0.99, ransac_status);
			Vr_x = Vr1_x - Vr2_x;
			Vr_y = Vr1_y - Vr2_y;
			
			for (i = k = 0; i < points2.size(); i++)
			{
				
				if(status[i] == 1)
				{
//					line(rgbFrames, points1[i], points2[i], Scalar(0, 0, 255), 1, 1, 0);
//					circle(rgbFrames, points1[i], 2, Scalar(255, 0, 0), 1, 1, 0);
					nvel_x = nvel_x + (points1[i].x - points2[i].x);
					nvel_y = nvel_y + (points1[i].y - points2[i].y);
					vel_cnt++;
				}
				/*
				if(ransac_status[i] == 1)
				{
					line(robustFrames, points1[i], points2[i], Scalar(0, 0, 255), 1, 1, 0);
					circle(robustFrames, points1[i], 2, Scalar(255, 0, 0), 1, 1, 0);
					rvel_x = rvel_x + (points1[i].x - points2[i].x);
					rvel_y = rvel_y + (points1[i].y - points2[i].y);
					vel_cnt1++;
				}
				*/
//				line(rgbFrames, points1[i], points2[i], Scalar(0, 0, 255), 1, 1, 0);
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
//		rvel_x = rvel_x / vel_cnt1;
//		rvel_y = rvel_y / vel_cnt1;
//		cout << nvel_x << " " << nvel_y << endl;
//		cout << rvel_x << " " << rvel_y << endl;
//		cout << rvel_x*p2m_width << " " << rvel_y*p2m_height << endl;
		cout << hroll << " " << hpitch << " " << nvel_x << " " << nvel_y << " " << Vr_x << " " << Vr_y
		<< " " <<velocity_x << " " << velocity_y << endl;
//		cout << Vr1_x << " " << Vr1_y << " " << Vr2_x << " " << Vr2_y << " " << halt << endl;
//		cout << velocity_x << " " << velocity_y << endl;
		vel_cnt = vel_cnt1 = 1;
		nvel_x = nvel_y = 0;
//		imshow(rawWindow, rgbFrames);
//		imshow(robustWindow, robustFrames);
		swap(points2, points1);
		points1.clear();
		
		Vr2_x = Vr1_x;
		Vr2_y = Vr1_y;
		Vr1_x = Vr1_y = 0;
		
		
		grayFrames.copyTo(prevGrayFrame);

		keyPressed = waitKey(10);
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
	}
	cout << "Wiring Pi init success" << endl;

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
		
		signed char recvData[HS_PACKET_LENGTH_MAX];
		int recvDataLen;
		recvDataLen = hsSerial->recvPacket(recvData);
		if( recvDataLen == -1 ) {
			cout << "serial error..." << endl;
		}else if( recvDataLen == 0 ) {
			//cout << "serial 0" << endl;
		}else if( recvDataLen == 9999 ) {
			cout << "serial 9999" << endl;
		}else {
			roll = (((short)recvData[0] << 8) | ((unsigned char)recvData[1] << 0 )) / 10.0;
			pitch = (((short)recvData[2] << 8) | ((unsigned char)recvData[3] << 0 )) / 10.0;
			yaw = (((short)recvData[4] << 8) | ((unsigned char)recvData[5] << 0 )) / 10.0;
			alt = (((short)recvData[6] << 8) | ((unsigned char)recvData[7] << 0 )) / 10.0;
			ax = (((short)recvData[8] << 8) | ((unsigned char)recvData[9] << 0 )) / 100.0;
			ay = (((short)recvData[10] << 8) | ((unsigned char)recvData[11] << 0 )) / 100.0;
			az = (((short)recvData[12] << 8) | ((unsigned char)recvData[13] << 0 )) / 100.0;
			//cout << (short)recvData[0] << "\t" << (unsigned char)recvData[1] << "\t" << roll << endl;
//			cout << roll << "\t" << pitch << "\t" << yaw << "\t" << alt << "\t" << ax << "\t"
//				<< ay << "\t" << az << endl;
			
			/*
			for(int i=0; i<recvDataLen; i++) {
				cout << (int)recvData[i] <<  "  ";
			}
			cout << "" << endl;
			*/
		}
#ifndef WIRELESS_DEBUGGING
		delay(25);
#else
		delay(5);
#endif


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
