#include "GyroSensor.h"

GyroSensor::GyroSensor(){
}

GyroSensor::GyroSensor(SENSOR_PORT_T _inputPort){
	inputPort = _inputPort;
}

// �W���C���Z���T�l�𓾂�
float GyroSensor::getGyroSensor(){
	return ecrobot_get_gyro_sensor(inputPort);
}
