#include "Motor.h"

Motor::Motor(){
}

Motor::Motor(MOTOR_PORT_T _outputPort)
{
	outputPort = _outputPort;
}

// ‰ñ“]Šp“x‚ðƒŠƒZƒbƒg‚·‚é
void Motor::resetAngle()
{
	return nxt_motor_set_count(outputPort, 0);
}

// ‰ñ“]Šp“x‚ð“¾‚é
long Motor::getAngle()
{
	return nxt_motor_get_count(outputPort);
}

// ‰ñ“]‚³‚¹‚é
void Motor::rotate(float pwm)
{
	nxt_motor_set_speed(outputPort, (int)pwm, 1);
} 
