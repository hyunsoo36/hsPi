#ifndef _HS_NAVI_H__
#define _HS_NAVI_H__

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <iostream>

using namespace std;

void initOdometrybyAccel(double offset_x, double offset_y, double offset_z);

class HSNavi {
private:
	
	
	
	
public:
	
	void estimateVelbyAccel(double roll, double pitch, double ax, double ay, double az, double dt);
	
	
	
	
};





#endif
