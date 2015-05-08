#ifndef _HS_THREAD_HPP__
#define _HS_THREAD_HPP__


#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include "hs_udpserver.hpp"
#include "hs_serial.hpp"

VideoCapture* cap;
Mat frame, img;

void *thread_udp(void *arg);
void *thread_serial(void *arg);
void *thread_cv(void *arg);


#endif
