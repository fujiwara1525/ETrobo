#include "TouchSensor.h"

TouchSensor::TouchSensor(void){
}

TouchSensor::TouchSensor(SENSOR_PORT_T _inputPort){
	inputPort = _inputPort;
}

// �^�b�`�Z���T�̓��͒l�𓾂�
BOOL TouchSensor::isPressed(){
	return ecrobot_get_touch_sensor(inputPort);
}
