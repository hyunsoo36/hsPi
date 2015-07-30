/*
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
*/
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
//#include "hs_udpserver.hpp"
//#include "hs_serial.hpp"
#include "hsNavi.hpp"
#include "hs_thread.hpp"


#define LED1 1
#define LED2 5

//#define WIRELESS_DEBUGGING

#define LOOP_TIME 10 // [ms]
#define SERIAL_TIME 50

/////////////////////////////////////
#define MAX_COUNT	100
#define AOVWIDTH	53.5		// Angle of view(width)
#define AOVHEIGHT	41.5		// Angle of view(height)


char imageFileName[32];
long imageIndex = 0;
char keyPressed;

int velocity_x, velocity_y;

/////////////////////////////////////
using namespace std;


//void *thread_udp(void *arg);
//void *thread_serial(void *arg);
//void *thread_cv(void *arg);

double getVelocitybyRotate(double angularVelocity, double z);	// [deg/s], [m]

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


unsigned long ulTimeBegin = 0;
unsigned long ulElapsedTime = 0;

//extern double roll_sp, pitch_sp, yaw_sp, alt_sp;
//extern double roll, pitch, yaw, alt, ax, ay, az;
extern VTOL_data wingwing2;
extern Phone_data phone2;

int main(int argc, char** argv){
	
	int fd, data;
	pthread_t udp_thread, serial_thread, cv_thread;
	
	// UDP waiting
	//UDPServer *tmp_udp = new UDPServer();
	//while( tmp_udp->CreateSocket() == 0 ) {
	//		delay(50);
	//}
	
	
	
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
	
	HSNavi *haNavi = new HSNavi();
	
	
	while (1)
	{
		
		ulTimeBegin = millis();
		//cout << roll << ", " << pitch << ", " << az << ", " << endl;
		haNavi->estimateVelbyAccel(wingwing2.roll, wingwing2.pitch, wingwing2.ax, wingwing2.ay, wingwing2.az, LOOP_TIME/1000.0);
		
		//double rollVelocity = (roll - lastRoll) / (LOOP_TIME / 1000.0);
		//double pitchVelocity = (pitch - lastPitch) / (LOOP_TIME / 1000.0);
		//getVelocitybyRotate( rollVelocity, alt/100.0 );
		//getVelocitybyRotate( pitchVelocity, alt/100.0 );
		
		//cout << "roll velocity : " << (double)getVelocitybyRotate( rollVelocity, alt/100.0 ) 
		//	<< "\tpitch velocity : " << (double)getVelocitybyRotate( pitchVelocity, alt/100.0 ) << endl;

		
		//make profile
		
		//roll_sp = 4;
		//pitch_sp = 5;
		//yaw_sp = 0;
		//alt_sp = 0;
		/*
		if( udp_data[0] == 'a' ) {
			
			if( profileTime >= 0.0 && profileTime < 1.0 ) {
				
			}
			//roll_sp = -sin(profileTime*0.392)*5;
			//pitch_sp = cos(profileTime*0.392)*5;
			//roll_sp = -sin(profileTime*1.57)*5;
			//pitch_sp = cos(profileTime*1.57)*5;
			
			profileTime += LOOP_TIME / 1000.0;
			
		}
		*/
		

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
