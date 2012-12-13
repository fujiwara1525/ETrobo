#ifndef BLANCECONTROL_H
#define BLANCECONTROL_H

extern "C"{
	#include "ecrobot_interface.h"
	#include "kernel.h"
	#include "balancer.h" // �|���U�q����p�w�b�_�t�@�C��
};

class BalanceControl{
private:
	char pwm_l;
	char pwm_r;
	BOOL isInitialized;
public:
	BalanceControl();
	void calculation(char forward, char turn, float gyro, float gyro_offset, long motor_ang_l, long motor_ang_r);
	char getLeft();
	char getRight();
	void init();
};

#endif
