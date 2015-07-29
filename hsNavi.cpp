#include "hsNavi.hpp"
#include <math.h>

double val_ax = 0;
double val_ay = 0;
double val_az = 0;

double val_ax_tmp = 0;
double val_ay_tmp = 0;
double val_az_tmp = 0;

#define HDR_ACC_I	0.01

void HSNavi::estimateVelbyAccel(double roll, double pitch, double ax, double ay, double az, double dt) {
	
	double weight_body = 106;	// [g]
	
	double roll_rad = roll/57.3;
	double pitch_rad = pitch/57.3;
	
	double zeroG_ax = ax - (-sin(pitch_rad));
	double zeroG_ay = ay - ( cos(pitch_rad) * sin(roll_rad) );
	double zeroG_az = az - ( cos(pitch_rad) * cos(roll_rad) );
	
	//cout << roll << "\t" << pitch << "\t";
	//cout << ax << "\t" << ay << "\t" << az << "\t";
	//cout << zeroG_ax << "\t" << zeroG_ay << "\t" << zeroG_az << "\t";
	
	val_ax_tmp = val_ax_tmp + (zeroG_ax * 9.8 * dt);
	val_ay_tmp = val_ay_tmp + (zeroG_ay * 9.8 * dt);
	val_az_tmp = val_az_tmp + (zeroG_az * 9.8 * dt);
	
	
	cout << val_ax_tmp << "\t" << val_ay_tmp << "\t" << val_az_tmp <<  "\t";
	
	val_ax = val_ax + (zeroG_ax * 9.8 * dt);
	val_ay = val_ay + (zeroG_ay * 9.8 * dt);
	val_az = val_az + (zeroG_az * 9.8 * dt);
	
	
	val_ax += ( (val_ax>0) ? -HDR_ACC_I : ((val_ax<0) ? HDR_ACC_I : 0) );
	val_ay += ( (val_ay>0) ? -HDR_ACC_I : ((val_ay<0) ? HDR_ACC_I : 0) );
	val_az += ( (val_az>0) ? -HDR_ACC_I : ((val_az<0) ? HDR_ACC_I : 0) );
	
	cout << val_ax << "\t" << val_ay << "\t" << val_az << endl;
	
	
}

