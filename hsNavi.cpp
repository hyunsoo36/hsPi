#include "hsNavi.hpp"
#include <math.h>

float val_ax = 0;
float val_ay = 0;
float val_az = 0;

double val_ax_tmp = 0;
double val_ay_tmp = 0;
double val_az_tmp = 0;

#define HDR_ACC_I	0.005f

float offset_ax, offset_ay, offset_az;
float offset_roll, offset_pitch;

void HSNavi::estimateVelbyAccel(double roll, double pitch, double ax, double ay, double az, double dt) {
	cout.fill(' ');
	cout.precision(3);
	
	//double weight_body = 106;	// [g]
	
	float roll_rad = roll/57.3f;
	float pitch_rad = pitch/57.3f;
	
	float zeroG_ax = (ax-offset_ax) - (-sin(pitch_rad));
	float zeroG_ay = (ay-offset_ay) - ( cos(pitch_rad) * sin(roll_rad) );
	float zeroG_az = (az-offset_az) - ( cos(pitch_rad) * cos(roll_rad) );
	
	//cout << (ax-offset_ax) << "\t" << sin(pitch_rad) << endl;
	//cout << ax << "\t" << ay << "\t" << az << "\t";
	//cout << offset_ax << "\t" << offset_ay << "\t" << offset_az << endl;
	//cout << ax-offset_ax << "\t" << ay-offset_ay << "\t" << az-offset_az << endl;
	//cout << zeroG_ax << "\t" << zeroG_ay << "\t" << zeroG_az << endl;
	
	val_ax_tmp = val_ax_tmp + (zeroG_ax * 9.8f * dt);
	val_ay_tmp = val_ay_tmp + (zeroG_ay * 9.8f * dt);
	val_az_tmp = val_az_tmp + (zeroG_az * 9.8f * dt);
	
	
	
	cout << val_ax_tmp << "\t" << val_ay_tmp << "\t" << val_az_tmp <<  "\t";
	
	val_ax = val_ax + (zeroG_ax * 9.8f * dt);
	val_ay = val_ay + (zeroG_ay * 9.8f * dt);
	val_az = val_az + (zeroG_az * 9.8f * dt);
	
	val_ax += ( (val_ax>0.0f) ? -HDR_ACC_I : ((val_ax<0.0f) ? HDR_ACC_I : 0.0f) );
	val_ay += ( (val_ay>0.0f) ? -HDR_ACC_I : ((val_ay<0.0f) ? HDR_ACC_I : 0.0f) );
	val_az += ( (val_az>0.0f) ? -HDR_ACC_I : ((val_az<0.0f) ? HDR_ACC_I : 0.0f) );
	
	//cout << roll << "\t" << pitch << "\t";
	cout << val_ax << "\t" << val_ay << "\t" << val_az << endl;
	
	
	
	
}

void initOdometrybyAccel(double offset_x, double offset_y, double offset_z) {
	offset_ax = offset_x;
	offset_ay = offset_y;
	offset_az = offset_z;
	//cout << offset_ax << "\t" << offset_ay << "\t" << offset_az << endl;
	//val_ax = 0;
	//val_ay = 0;
	//val_az = 0;
	//val_ax_tmp = 0;
	//val_ay_tmp = 0;
	//val_az_tmp = 0;
}

