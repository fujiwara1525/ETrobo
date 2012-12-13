#ifndef MOTOR_H
#define MOTOR_H

extern "C"{
#include "ecrobot_interface.h"
#include "kernel.h"
};

class Motor{
private:
	MOTOR_PORT_T outputPort;
public:
	Motor();
	Motor(MOTOR_PORT_T _outputPort);
	void resetAngle();
	long getAngle();
	void rotate(float pwm);
};

#endif
