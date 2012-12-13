#include "SonarSensor.h"

SonarSensor::SonarSensor(){
}

SonarSensor::SonarSensor(SENSOR_PORT_T _inputPort){
	counter = 0;
	inputPort = _inputPort;
}

// 超音波センサ計測距離データを得る
S32 SonarSensor::getSonarSensor(){
	static S32 sonar = -1;
	
	if(counter == 9){
		sonar = ecrobot_get_sonar_sensor(inputPort);
		counter = 0;
	}
	else{
		counter++;
	}
	
	return sonar;
} 
