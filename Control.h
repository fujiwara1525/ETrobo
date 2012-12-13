#ifndef CONTROL_H
#define CONTROL_H

#include "PID.h"
#include "BalanceControl.h"
#include "Motor.h"
#include "Tail.h"

extern "C"{
	#include "kernel.h"
};

class Control{
private:
	PID *pid;
	BalanceControl *balanceControl;
	Motor *leftMotor;
	Motor *rightMotor;
	Tail *tail;
	int safety;
public:
	Control();
	Control(PID _pid, BalanceControl _balanceControl,
		Motor _leftMotor, Motor _rightMotor, Tail _tail);
	void bonusRun(float forward, float turn, float tail, float gyro_offset, BOOL pid_flag, U16 pid_target, F32 KP, BOOL balance_flag, float gyro, U16 light);
	void basicRun(float forward, float turn, float tail, float gyro_offset, BOOL pid_flag, U16 pid_target, F32 KP, BOOL balance_flag, float gyro, U16 light);
	void resetBalance();
};

#endif
