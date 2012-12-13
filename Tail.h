#ifndef TAIL_H
#define TAIL_H

extern "C"{
	#include "kernel.h"
	#include "ecrobot_interface.h"
};

class Tail{
private:
	MOTOR_PORT_T outputPort;
public:
	Tail();
	Tail(MOTOR_PORT_T _outputPort);
	void control(float angle);
};

#endif
