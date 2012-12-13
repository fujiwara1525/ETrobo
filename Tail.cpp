#include "Tail.h"

#define P_GAIN      5.0F /* Š®‘S’âŽ~—pƒ‚[ƒ^§Œä”ä—áŒW” */
#define PWM_ABS_MAX 60 /* Š®‘S’âŽ~—pƒ‚[ƒ^§ŒäPWMâ‘ÎÅ‘å’l */

Tail::Tail(void){
}

Tail::Tail(MOTOR_PORT_T _outputPort){
	outputPort = _outputPort;
}

// ‚µ‚Á‚Û‚ð‰ñ“]‚³‚¹‚é
void Tail::control(float angle){
	float pwm = (float)(angle - nxt_motor_get_count(outputPort))*P_GAIN; /* ”ä—á§Œä */
	/* PWMo—Í–O˜aˆ— */
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
