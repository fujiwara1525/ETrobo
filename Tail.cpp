#include "Tail.h"

#define P_GAIN      5.0F /* ���S��~�p���[�^������W�� */
#define PWM_ABS_MAX 60 /* ���S��~�p���[�^����PWM��΍ő�l */

Tail::Tail(void){
}

Tail::Tail(MOTOR_PORT_T _outputPort){
	outputPort = _outputPort;
}

// �����ۂ���]������
void Tail::control(float angle){
	float pwm = (float)(angle - nxt_motor_get_count(outputPort))*P_GAIN; /* ��ᐧ�� */
	/* PWM�o�͖O�a���� */
	if (pwm > PWM_ABS_MAX)
	{
		pwm = PWM_ABS_MAX;
	}
	else if (pwm < -PWM_ABS_MAX)
	{
		pwm = -PWM_ABS_MAX;
	}

	nxt_motor_set_speed(outputPort, (signed char)pwm, 1);
}
