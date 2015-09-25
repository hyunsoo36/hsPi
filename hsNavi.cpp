#include "hsNavi.hpp"
#include <math.h>



// The i parameter is sensitive.
// Experimentally, 
// 0.002 reduced integral drift.
#define HDR_ACC_I	0.003f
#define GYRO_VEL_CLIPPING 0.2
#define INTEGRAL_WINDUP		3.0f

float offset_ax=0, offset_ay=0, offset_az=0;
float offset_roll=0, offset_pitch=0;

Command_data cmd_data;


void HSNavi::estimateLocalPosition() {
	local_pos_x += hori_vel_x * dt;
	local_pos_y += hori_vel_y * dt;
	
	local_pos_vx += vel_vx * dt;
	local_pos_vy += vel_vy * dt;
	
	
	/*
	cout << local_pos_x;
	cout << "\t";
	cout << local_pos_y;
	cout << "\t";
	cout << hori_vel_x;
	cout << "\t";
	cout << hori_vel_y;
	cout << endl;
	*/
}


void HSNavi::velocityController(float sp_x, float sp_y) {
	sp_x_lpf = sp_x_lpf * 0.8f + sp_x * 0.2f;
	sp_y_lpf = sp_y_lpf * 0.8f + sp_y * 0.2f;
	
	vel_x_lpf = vel_x_lpf * 0.50f + hori_vel_x * 0.50f;
	vel_y_lpf = vel_y_lpf * 0.50f + hori_vel_y * 0.50f;
	
	err_x = sp_x_lpf - vel_x_lpf;
	err_y = -(sp_y_lpf - vel_y_lpf);
	
	p_x = err_x * kp;
	p_y = err_y * kp;
	
	if( err_x > 0.2 || err_x < -0.2 ) {
		integral_x += err_x;
	}
	
	if( err_y > 0.2 || err_y < -0.2 ) {
		integral_y += err_y;
	}
	
	i_x = integral_x * ki;
	i_y = integral_y * ki;
	
	// integral wind up
	if( i_x > INTEGRAL_WINDUP ) { i_x = INTEGRAL_WINDUP; }
	if( i_x < -INTEGRAL_WINDUP ) { i_x = -INTEGRAL_WINDUP; }
	if( i_y > INTEGRAL_WINDUP ) { i_y = INTEGRAL_WINDUP; }
	if( i_y < -INTEGRAL_WINDUP ) { i_y = -INTEGRAL_WINDUP; }
	
	
	d_x = (err_x - err_x_last) * kd / dt;
	d_y = (err_y - err_y_last) * kd / dt;
	
	
	pid_x = p_x + i_x + d_x;
	pid_y = p_y + i_y + d_y;
	
	err_x_last = err_x;
	err_y_last = err_y;
	/*
	cout << hori_vel_x;
	cout << "\t";
	cout << vel_x_lpf;
	cout << "\t";
	cout << p_x;
	cout << "\t";
	cout << sp_x;
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

void HSNavi::CF_velocity() {
	
	calcRotateVelocity();
	gyroVelClipping();
	
	float cf_err_x = hori_vel_x - (vel_vx - (-vel_rx));
	float cf_p_val_x = cf_err_x * cf_kp;
	hori_vel_x += ((zeroG_ax * 9.8f) - cf_p_val_x) * dt;
	
	float cf_err_y = hori_vel_y - (vel_vy - (vel_ry));
	float cf_p_val_y = cf_err_y * cf_kp;
	hori_vel_y += ((zeroG_ay * 9.8f) - cf_p_val_y) * dt;
	
	cout << hori_vel_x << "\t";
	cout << vel_vx << "\t";
	cout << vel_rx << "\t";
	cout << vel_ax << "\t";
	cout << endl;
	
}
void HSNavi::gyroVelClipping() {
	if((vel_rx < GYRO_VEL_CLIPPING) && (vel_rx > -GYRO_VEL_CLIPPING)) {
		vel_rx = 0;
	}else {
		if( vel_rx >= GYRO_VEL_CLIPPING ) {
			vel_rx = vel_rx - GYRO_VEL_CLIPPING;
		}else if( vel_rx <= -GYRO_VEL_CLIPPING ){
			vel_rx = vel_rx + GYRO_VEL_CLIPPING;
		}
	}
	if((vel_ry < GYRO_VEL_CLIPPING) && (vel_ry > -GYRO_VEL_CLIPPING)) {
		vel_ry = 0;
	}else {
		if( vel_ry >= GYRO_VEL_CLIPPING ) {
			vel_ry = vel_ry - GYRO_VEL_CLIPPING;
		}else if( vel_ry <= -GYRO_VEL_CLIPPING ){
			vel_ry = vel_ry + GYRO_VEL_CLIPPING;
		}
	}
}
void HSNavi::setOpticalFlowResult(float x, float y) {
	vel_vx = x;
	vel_vy = y;
}
void HSNavi::calcRotateVelocity() {
	
	vel_rx = sin( wing2.gy / 57.23 ) * (wing2.alt*0.01f);
	vel_ry = sin( wing2.gx / 57.23 ) * (wing2.alt*0.01f);
}

void HSNavi::updateCMDdata() {
	cmd_data.roll_sp = pid_y;
	cmd_data.pitch_sp = pid_x;
	
}
void HSNavi::updateCMDdataManual() {
	cmd_data.roll_sp = sp_y_lpf;
	cmd_data.pitch_sp = sp_x_lpf;
	
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
	
	pid_y = 0;
	pid_x = 0;
	
	vel_vx = 0;
	vel_vy = 0;
	
	local_pos_x = 0;
	local_pos_y = 0;
	
	sp_x_lpf = 0;
	sp_y_lpf = 0;
	
	integral_x = 0;
	integral_y = 0;
	
}


HSNavi::HSNavi() {
	kp = 35.0f;
	ki = 0.2f;
	kd = 0.0f;//12.0f;
	
	cf_kp = 10.0f;
	
}

double getVelocitybyRotate(double angularVelocity, double z) {	// [deg/s], [m]
	return sin( angularVelocity / 57.23 ) * z ;
}

