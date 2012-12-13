#ifndef UI_H
#define UI_H

#include "TouchSensor.h"
#include "BlueTooth.h"
#include "LightSensor.h"
#include "Tail.h"

extern "C"{
#include "kernel.h"
#include "ecrobot_interface.h"
};

class UI{
private:
	TouchSensor *touchSensor;
	BlueTooth *blueTooth;
	LightSensor *lightSensor;
	Tail *tail;	
public:
	UI();
	UI(TouchSensor _touchSensor, BlueTooth _blueTooth, LightSensor _lightSensor, Tail _tail);
	void waitStart(float angle);
	U16 calibrate(float angle);
	int courseSelect();
};

#endif
