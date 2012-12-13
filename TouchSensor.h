#ifndef TOUCHSENSOR_H
#define TOUCHSENSOR_H

extern "C"{
	#include "ecrobot_interface.h"
	#include "kernel.h"
};

class TouchSensor{
private:
	SENSOR_PORT_T inputPort;
public:
	TouchSensor();
	TouchSensor(SENSOR_PORT_T inputPort);
	BOOL isPressed();
};

#endif
