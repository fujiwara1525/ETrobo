#ifndef INPUT_H
#define INPUT_H

#include "GyroSensor.h"
#include "LightSensor.h"
#include "SonarSensor.h"

extern "C"{
	#include "ecrobot_interface.h"
	#include "kernel.h"
}

class Input{
private:
	float gyro;
	U16 light;
	S32 sonar;
	GyroSensor *gyroSensor;
	LightSensor *lightSensor;
	SonarSensor *sonarSensor;
public:
	Input();
	Input(GyroSensor _gyroSensor, LightSensor _lightSensor, SonarSensor _sonarSensor);
	void getInput();
	float getGyroSensor();
	U16 getBrightness();
	S32 getSonarSensor();
};

#endif
