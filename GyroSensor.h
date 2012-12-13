#ifndef GYROSENSOR_H
#define GYROSENSOR_H

extern "C"{
#include "ecrobot_interface.h"
#include "kernel.h"
};

class GyroSensor{
private:
	SENSOR_PORT_T inputPort;
public:
	GyroSensor();
	GyroSensor(SENSOR_PORT_T _inputPort);
	float getGyroSensor();
};

#endif
