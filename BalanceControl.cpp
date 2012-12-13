#include "BalanceControl.h"

BalanceControl::BalanceControl(){
	pwm_l = 0;
	pwm_r = 0;
	isInitialized = FALSE;
}

// �|���U�q������s��
void BalanceControl::calculation(char forward, char turn, float gyro, float gyro_offset, long motor_ang_l, long motor_ang_r){
	if(! isInitialized)
	{
		// �|���U�q���䏉����
		balance_init();
	
		// �������ςɂ���
		isInitialized = TRUE;
	}

	balance_control(
		(float)forward,
		(float)turn,
		(float)gyro, 
		(float)gyro_offset,
		(float)motor_ang_l, 
		(float)motor_ang_r,
		(float)ecrobot_get_battery_voltage(),
		(signed char*)&pwm_l, 
		(signed char*)&pwm_r
	);
}

// �����[�^�̏o�͒l��Ԃ�
char BalanceControl::getLeft(){
	return pwm_l;
}

// �E���[�^�̏o�͒l��Ԃ�
char BalanceControl::getRight(){
	return pwm_r;
}

// ���������ɂ���
void BalanceControl::init(){
	isInitialized = FALSE;
}
