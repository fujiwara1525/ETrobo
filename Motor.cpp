#include "Motor.h"

Motor::Motor(){
}

Motor::Motor(MOTOR_PORT_T _outputPort)
{
	outputPort = _outputPort;
}

// ��]�p�x�����Z�b�g����
void Motor::resetAngle()
{
	return nxt_motor_set_count(outputPort, 0);
}

// ��]�p�x�𓾂�
long Motor::getAngle()
{
	return nxt_motor_get_count(outputPort);
}

// ��]������
void Motor::rotate(float pwm)
{
	nxt_motor_set_speed(outputPort, (int)pwm, 1);
} 
