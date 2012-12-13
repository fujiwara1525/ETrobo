#ifndef SONARSENSOR_H
#define SONARSENSOR_H

extern "C"{
	#include "ecrobot_interface.h"
	#include "kernel.h"
};

class SonarSensor{
private:
	int counter;
	SENSOR_PORT_T inputPort;
public:
	SonarSensor();
	SonarSensor(SENSOR_PORT_T _inputPort);
	S32 getSonarSensor();
};

#endif
