
#ifndef _HS_DATA_STRUCT_HPP__
#define _HS_DATA_STRUCT_HPP__

// To arduino from odroid
class VTOL_data {	
public:
	float roll;
	float pitch;
	float yaw;
	float alt;
	float ax;
	float ay;
	float az;
	double lastRoll;
	double lastPitch;
};

// To phone from odroid	
// To odroid from arduino with manual mode
class Phone_data {	
public:
	int VTOL_state;
	double roll_sp;
	double pitch_sp;
	double yaw_sp;
	double alt_sp;

};

// To odroid from arduino
class Command_data {
public:
	int VTOL_state;
	double roll_sp;
	double pitch_sp;
	double yaw_sp;
	double alt_sp;
	
};

#endif
