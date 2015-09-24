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
	float vel_ax, vel_ay, vel_az;	// velocity using a accelerometer. [m/s]
	float zeroG_ax, zeroG_ay, zeroG_az;
	float pid_x, pid_y;
	float vel_vx, vel_vy;			// velocity using a vertical camera. [m/s]
	float vel_rx, vel_ry;			// velocity using a gyro sensor. [m/s]
	
	float local_pos_x, local_pos_y;
	
	double vel_x_lpf, vel_y_lpf, vel_z_lpf;
	float sp_x_lpf, sp_y_lpf;
	
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
	void calcRotateVelocity();
	void CF_velocity();
	
	void estimateLocalPosition();
	
};





#endif
