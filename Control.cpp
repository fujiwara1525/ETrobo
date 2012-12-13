#include "Control.h"

Control::Control(){

}

Control::Control(PID _pid, BalanceControl _balanceControl, Motor _leftMotor, Motor _rightMotor, Tail _tail){
	*pid = _pid;
	*balanceControl = _balanceControl;
	*leftMotor = _leftMotor;
	*rightMotor = _rightMotor;
	*tail = _tail;
	safety = 0;
}

// �e����Əo�͂��s��
void Control::bonusRun(float forward, float turn, float tail_angle, 
	float gyro_offset, BOOL pid_flag, U16 pid_target, F32 KP, BOOL balance_flag, float gyro, U16 light){
		
		static float Turn;
	
		// �����ۂ���]������
		tail->control(tail_angle);
		
		// pid_flag��TRUE�̂Ƃ��APID����Ő��񑬓x���v�Z����
		if(pid_flag){
			Turn = pid->calculation(light, pid_target, KP);
		}
		else{
			Turn = turn;
		}
		
		// balance_flag��TRUE�̂Ƃ��A�|���U�q����Ń��[�^�o�͒l���v�Z����
		if(balance_flag){
			balanceControl->calculation((char)forward, (char)Turn, gyro, gyro_offset, leftMotor->getAngle(), rightMotor->getAngle());
			// ���[�^����]������
			leftMotor->rotate(balanceControl->getLeft());
			rightMotor->rotate(balanceControl->getRight());
		}
		else{
			// ���[�^����]������
			leftMotor->rotate(forward + Turn);
			rightMotor->rotate(forward - Turn);
		}

		if(light > 670){
			if(++safety >= 1250)
				while(1){
					leftMotor->rotate(0);
					rightMotor->rotate(0);
				}
		}
		else{
			safety = 0;
		}
		
		/* ���M���O */
		ecrobot_bt_data_logger(0, 0);
}

void Control::basicRun(float forward, float turn, float tail_angle, 
	float gyro_offset, BOOL pid_flag, U16 pid_target, F32 KP, BOOL balance_flag, float gyro, U16 light){

		static float Forward, Turn;

		// �����ۂ���]������
		tail->control(tail_angle);
		
		// pid_flag��TRUE�̂Ƃ��APID����Ő��񑬓x���v�Z����
		if(pid_flag){
			Turn = pid->calculation(light, pid_target, KP);
		}
		else{
			Turn = turn;
		}
		
		if(Turn <= -70 || Turn >= 70){
			Forward = forward - 30;
		}
		else{
			Forward = forward;
		}
		
		// balance_flag��TRUE�̂Ƃ��A�|���U�q����Ń��[�^�o�͒l���v�Z����
		if(balance_flag){
			balanceControl->calculation((char)Forward, (char)Turn, gyro, gyro_offset, leftMotor->getAngle(), rightMotor->getAngle());
			// ���[�^����]������
			leftMotor->rotate(balanceControl->getLeft());
			rightMotor->rotate(balanceControl->getRight());
		}
		else{
			// ���[�^����]������
			leftMotor->rotate(Forward + Turn);
			rightMotor->rotate(Forward - Turn);
		}

		if(light > 670){
			if(++safety >= 1250)
				while(1){
					leftMotor->rotate(0);
					rightMotor->rotate(0);
				}
		}
		else{
			safety = 0;
		}
		
		/* ���M���O */
		ecrobot_bt_data_logger(0, 0);
}

// �|���U�q����ƃ��[�^��]�p�x�̏�����
void Control::resetBalance(){
	balanceControl->init();
	leftMotor->resetAngle();
	rightMotor->resetAngle();
}
