#include "Input.h"

Input::Input(){
}

Input::Input(GyroSensor _gyroSensor, LightSensor _lightSensor, SonarSensor _sonarSensor)
{
	gyro = 0;
	light = 0;
	sonar = 0;
	*gyroSensor = _gyroSensor;
	*lightSensor = _lightSensor;
	*sonarSensor = _sonarSensor;
}

// �e���͒l�𓾂�
void Input::getInput(){
	gyro = gyroSensor->getGyroSensor();
	light = lightSensor->getBrightness();
	sonar = sonarSensor->getSonarSensor();
}

// �W���C���Z���T�̓��͒l��Ԃ�
float Input::getGyroSensor(){
	return gyro;
}

// ���Z���T�̓��͒l��Ԃ�
U16 Input::getBrightness(){
	return light;
}

// �����g�Z���T�̓��͒l��Ԃ�
S32 Input::getSonarSensor(){
	return sonar;
}


