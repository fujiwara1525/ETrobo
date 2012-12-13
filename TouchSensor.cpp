#include "TouchSensor.h"

TouchSensor::TouchSensor(void){
}

TouchSensor::TouchSensor(SENSOR_PORT_T _inputPort){
	inputPort = _inputPort;
}

// タッチセンサの入力値を得る
BOOL TouchSensor::isPressed(){
	return ecrobot_get_touch_sensor(inputPort);
}
