#ifndef PID_H
#define PID_H

extern "C"{
	#include "ecrobot_interface.h"
};

class PID{
private:
	S32 diff[2];
	F32 integral;
public:
	PID();
	void init();
	float calculation(U16 sensor_val, U16 target_val, float kp);
};

#endif
