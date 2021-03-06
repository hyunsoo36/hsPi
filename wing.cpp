// git test in samsung
// crash test!!010101010101010101

//02020202020202020
//03030303030
//04040404
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

#define LOOP_TIME 20 // [ms]
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
void generateFileName(char *f_name);

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


unsigned long ulTimeBegin = 0;
unsigned long ulElapsedTime = 0;

//extern double roll_sp, pitch_sp, yaw_sp, alt_sp;
//extern double roll, pitch, yaw, alt, ax, ay, az;

HSNavi hsNavi;

extern VTOL_data wingwing2;
extern Phone_data phone2;

double profileTime = 0;

int main(int argc, char** argv){
	
	int data;
	pthread_t udp_thread, serial_thread, cv_thread, file_thread;
	
	// UDP waiting
	//UDPServer *tmp_udp = new UDPServer();
	//while( tmp_udp->CreateSocket() == 0 ) {
	//		delay(50);
	//}
	
	delay(1000);
	
	
	
	
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
	
	if(pthread_create(&file_thread, NULL, thread_file, NULL)) {
		cout << "Cannot creating cv thread" << endl;
		return -1;
	}
	
	cout.fill(' ');
	cout << fixed;
	cout.precision(3);
	
	
	int vel_ctrl_cnt = 0;
	
	while (1)
	{
		
		ulTimeBegin = millis();
		//cout << roll << ", " << pitch << ", " << az << ", " << endl;
		hsNavi.setParameters(wingwing2, LOOP_TIME/1000.0f);
		hsNavi.estimateVelbyAccel();
		hsNavi.CF_velocity();
		hsNavi.estimateLocalPosition();
		
		//profileTime += LOOP_TIME / 1000.0;
		//if(profileTime < 10.0f) {
			//hsNavi.velocityController(phone2.pitch_sp/40.0f, -phone2.roll_sp/40.0f);
		//}else {
		//	hsNavi.velocityController(0.6, -phone2.roll_sp/20.0f);
		//}
		vel_ctrl_cnt ++;
		if( vel_ctrl_cnt >= 10 ) {
			vel_ctrl_cnt = 0;
			hsNavi.velocityController(phone2.pitch_sp/40.0f, -phone2.roll_sp/40.0f);
		
			hsNavi.updateCMDdata();
			//hsNavi.updateCMDdataManual();
		}
		//hsNavi.updateCMDdataManual();
		
		
		
		
		
		
				/*
		cout << hsNavi.hori_vel_x;
		cout << "\t";
		cout << hsNavi.vel_x_lpf;
		cout << "\t";
		cout << hsNavi.sp_x_lpf;
		cout << "\t";
		cout << hsNavi.p_x;
		cout << "\t";
		cout << hsNavi.i_x;
		cout << "\t";
		cout << hsNavi.pid_x;
		cout << endl;
	*/
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


