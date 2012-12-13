#ifndef LIGHTSENSOR_H
#define LIGHTSENSOR_H

extern "C"{
	#include "ecrobot_interface.h"
	#include "kernel.h"
};

class LightSensor{
private:
	SENSOR_PORT_T inputPort;
public:
	LightSensor();
	LightSensor(SENSOR_PORT_T _inputPort);
	U16 getBrightness();
};

#endif
