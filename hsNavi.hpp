#ifndef _HS_NAVI_H__
#define _HS_NAVI_H__

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <iostream>
#include "hsDataStruct.hpp"

using namespace std;


class HSNavi {
public:
	VTOL_data wing2;
	float dt;
	float err_x, err_x_last;
	float err_y, err_y_last;
	float kp, ki, kd;
	float vel_ax, vel_ay, vel_az;	// velocity using accelerometer. [m/s]
	float zeroG_ax, zeroG_ay, zeroG_az;
	float pid_x, pid_y;
	float vel_vx, vel_vy;			// velocity using vertical camera. [m/s]
	
	double vel_ax_lpf, vel_ay_lpf, vel_az_lpf;
	
	// cf variable
	
	float hori_vel_x, hori_vel_y;
	float cf_kp;
	
public:
	HSNavi();
	void setParameters(VTOL_data w2, float t);
	void estimateVelbyAccel();
	void velocityController(float sp_x, float sp_y);
	void landingInitalize();
	void updateCMDdata();
	void setOpticalFlowResult(float x, float y);	// be used only optical flow process.
	void CF_AccelOpticalFlow();
	
};





#endif
