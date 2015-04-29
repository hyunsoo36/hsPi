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
#include "hs_udpserver.hpp"

#define LED1 4
#define LED2 5

using namespace std;
using namespace cv;

int main(int argc, char** argv){
	int fd, data;
	
	VideoCapture cap(-1);
	if (!cap.isOpened())
	{
		cout << "Cannot open camera" << endl;
		return -1;
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	namedWindow("Output",CV_WINDOW_AUTOSIZE);

	if( wiringPiSetup() == -1 ) {
		return 1;
	}
	if((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0) {
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return 1;
	}
	
	Mat frame, img;
	int a = 0, b = 0;
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
		cout << clock()-a << endl;
		a = clock();

		//digitalWrite(LED1, 0);        
		//digitalWrite(LED2, 0);        
		//delay(500);
		//digitalWrite(LED1, 1);        
		//digitalWrite(LED2, 1);        
		//delay(500);

		serialPutchar(fd, 'a');

	}
	return 0;
}



