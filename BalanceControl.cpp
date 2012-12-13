#include "BalanceControl.h"

BalanceControl::BalanceControl(){
	pwm_l = 0;
	pwm_r = 0;
	isInitialized = FALSE;
}

// 倒立振子制御を行う
void BalanceControl::calculation(char forward, char turn, float gyro, float gyro_offset, long motor_ang_l, long motor_ang_r){
	if(! isInitialized)
	{
		// 倒立振子制御初期化
		balance_init();
	
		// 初期化済にする
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

// 左モータの出力値を返す
char BalanceControl::getLeft(){
	return pwm_l;
}

// 右モータの出力値を返す
char BalanceControl::getRight(){
	return pwm_r;
}

// 未初期化にする
void BalanceControl::init(){
	isInitialized = FALSE;
}
