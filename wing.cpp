#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <pthread.h>
#include "hs_udpserver.hpp"

#define LED1 4
#define LED2 5

using namespace std;
using namespace cv;

void* thread_udp(void *arg);

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
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);//640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);//480);

	namedWindow("Output",CV_WINDOW_AUTOSIZE);

	if( wiringPiSetup() == -1 ) {
		return 1;
	}
	if((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0) {
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return 1;
	}
	
	Mat frame, img;
	unsigned long a = 0, b = 0;
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);


	while (1)
	{

		bool bSuccess = cap.read(frame);
		

		if (!bSuccess)
		{
			cout << "Cannot read a frame from camera" << endl;
			break;
		}
		
		//pyrDown(frame, img, Size(frame.cols/2, frame.rows/2));

		imshow("Output", frame);

		if (waitKey(30) == 27)
		{
			cout << "Exit" << endl;
			break;
		}
		
//		cout << cnt++ << "\t";
//		cout << millis()-b << endl;
		//cout <<	1000.0/(double)(millis()-b) << " fps" << endl;
		cout << micros()-b << endl;
		//a = clock();
		b = micros();

		//digitalWrite(LED1, 0);        
		//digitalWrite(LED2, 0);        
		//delay(500);
		//digitalWrite(LED1, 1);        
		//digitalWrite(LED2, 1);        
		//delay(20);

		serialPutchar(fd, 'a');

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

