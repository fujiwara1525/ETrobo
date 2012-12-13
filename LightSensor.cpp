#include "LightSensor.h"

LightSensor::LightSensor(void){
}

LightSensor::LightSensor(SENSOR_PORT_T _inputPort)
{
	inputPort = _inputPort;
}

// ‹P“x‚ð“¾‚é
U16 LightSensor::getBrightness(){
	return ecrobot_get_light_sensor(inputPort);
} 
