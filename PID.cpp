#include "PID.h"

#define DELTA_T 0.004
#define KI KP*0.157894736
#define KD KP*0.071052631

PID::PID(){
	diff[0] = 0;
	diff[1] = 0;
	integral = 0;
}

void PID::init(){
	diff[0] = 0;
	diff[1] = 0;
	integral = 0;
}

// PID§Œä‚Åù‰ñ‘¬“x‚ðŒvŽZ‚·‚é
float PID::calculation(U16 sensor_val, U16 target_val, float kp){
	float KP = kp;
	static float p,i,d,val;
	
	diff[0] = diff[1];
	diff[1] = sensor_val - target_val;
	integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;
	
	p = KP * diff[1];
	i = KI * integral;
	d = KD * (diff[1] - diff[0]) / DELTA_T;
	val = p + i + d;
	
	if(val > 100.0){
		val = 100.0;
	}
	else if(val < -100.0){
		val = -100.0;
	}
	
	return val;
}

