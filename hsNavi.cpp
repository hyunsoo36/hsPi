#include "hsNavi.hpp"
#include <math.h>



// The i parameter is sensitive.
// Experimentally, 
// 0.002 reduced integral drift.
#define HDR_ACC_I	0.003f

float offset_ax=0, offset_ay=0, offset_az=0;
float offset_roll=0, offset_pitch=0;

Command_data cmd_data;

void HSNavi::velocityController(float sp_x, float sp_y) {
	vel_ax_lpf = vel_ax_lpf * 0.95f + vel_ax * 0.05f;
	vel_ay_lpf = vel_ay_lpf * 0.95f + vel_ay * 0.05f;
	
	err_x = sp_x - vel_ax_lpf;
	err_y = -(sp_y - vel_ay_lpf);
	
	float p_x = err_x * kp;
	float p_y = err_y * kp;
	
	float d_x = (err_x - err_x_last) * kd / dt;
	float d_y = (err_y - err_y_last) * kd / dt;
	
	
	pid_x = p_x + d_x;
	pid_y = p_y + d_y;
	
	err_x_last = err_x;
	err_y_last = err_y;
	/*
	cout << vel_ax;
	cout << "\t";
	cout << vel_ax_lpf;
	cout << "\t";
	cout << p_x;
	cout << "\t";
	cout << d_x;
	cout << "\t";
	cout << pid_x;
	cout << endl;
	*/
	
}


void HSNavi::estimateVelbyAccel() {
	cout.fill(' ');
	cout.precision(3);
	
	//double weight_body = 106;	// [g]
	
	float roll_rad = wing2.roll/57.3f;
	float pitch_rad = wing2.pitch/57.3f;
	
	zeroG_ax = (wing2.ax/*-offset_ax*/) - (-sin(pitch_rad));
	zeroG_ay = (wing2.ay/*-offset_ay*/) - ( cos(pitch_rad) * sin(roll_rad) );
	zeroG_az = (wing2.az/*-offset_az*/) - ( cos(pitch_rad) * cos(roll_rad) );
	
	//cout << wing2.pitch << "\t" << wing2.ax << "\t" << -sin(pitch_rad) << endl;
	
	vel_ax = vel_ax + (zeroG_ax * 9.8f * dt);
	vel_ay = vel_ay + (zeroG_ay * 9.8f * dt);
	vel_az = vel_az + (zeroG_az * 9.8f * dt);
	
	//vel_ax += ( (vel_ax>0.0f) ? -HDR_ACC_I : ((vel_ax<0.0f) ? HDR_ACC_I : 0.0f) );
	//vel_ay += ( (vel_ay>0.0f) ? -HDR_ACC_I : ((vel_ay<0.0f) ? HDR_ACC_I : 0.0f) );
	//vel_az += ( (vel_az>0.0f) ? -HDR_ACC_I : ((vel_az<0.0f) ? HDR_ACC_I : 0.0f) );
	
	//cout << roll << "\t" << pitch << "\t";
	//cout << vel_ax << "\t" << vel_ay << "\t" << vel_az << endl;
	
	
}

void HSNavi::CF_AccelOpticalFlow() {
	float cf_err_x = hori_vel_x - vel_vx;
	float cf_p_val_x = cf_err_x * cf_kp;
	hori_vel_x += ((zeroG_ax * 9.8f) - cf_p_val_x) * dt;
	
}

void HSNavi::setOpticalFlowResult(float x, float y) {
	vel_vx = x;
	vel_vy = y;
}

void HSNavi::updateCMDdata() {
	cmd_data.roll_sp = pid_y;
	cmd_data.pitch_sp = pid_x;
	
}

void HSNavi::setParameters(VTOL_data w2, float t) {
	memcpy(&wing2, &w2, sizeof(VTOL_data));
	dt = t;
}

void HSNavi::landingInitalize() {
	vel_ax = 0;
	vel_ay = 0;
	vel_az = 0;
	
	hori_vel_x = 0;
	hori_vel_y = 0;
	
}


HSNavi::HSNavi() {
	kp = 0.0f;//40.0f;
	ki = 0.0f;
	kd = 15.0f;
	
	cf_kp = 2.0f;
	
}

