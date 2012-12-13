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

// 各入力値を得る
void Input::getInput(){
	gyro = gyroSensor->getGyroSensor();
	light = lightSensor->getBrightness();
	sonar = sonarSensor->getSonarSensor();
}

// ジャイロセンサの入力値を返す
float Input::getGyroSensor(){
	return gyro;
}

// 光センサの入力値を返す
U16 Input::getBrightness(){
	return light;
}

// 超音波センサの入力値を返す
S32 Input::getSonarSensor(){
	return sonar;
}


