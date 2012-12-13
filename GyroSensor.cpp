#include "GyroSensor.h"

GyroSensor::GyroSensor(){
}

GyroSensor::GyroSensor(SENSOR_PORT_T _inputPort){
	inputPort = _inputPort;
}

// ジャイロセンサ値を得る
float GyroSensor::getGyroSensor(){
	return ecrobot_get_gyro_sensor(inputPort);
}
