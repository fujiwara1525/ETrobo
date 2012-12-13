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

// Še§Œä‚Æo—Í‚ðs‚¤
void Control::bonusRun(float forward, float turn, float tail_angle, 
	float gyro_offset, BOOL pid_flag, U16 pid_target, F32 KP, BOOL balance_flag, float gyro, U16 light){
		
		static float Turn;
	
		// ‚µ‚Á‚Û‚ð‰ñ“]‚³‚¹‚é
		tail->control(tail_angle);
		
		// pid_flag‚ªTRUE‚Ì‚Æ‚«APID§Œä‚Åù‰ñ‘¬“x‚ðŒvŽZ‚·‚é
		if(pid_flag){
			Turn = pid->calculation(light, pid_target, KP);
		}
		else{
			Turn = turn;
		}
		
		// balance_flag‚ªTRUE‚Ì‚Æ‚«A“|—§UŽq§Œä‚Åƒ‚[ƒ^o—Í’l‚ðŒvŽZ‚·‚é
		if(balance_flag){
			balanceControl->calculation((char)forward, (char)Turn, gyro, gyro_offset, leftMotor->getAngle(), rightMotor->getAngle());
			// ƒ‚[ƒ^‚ð‰ñ“]‚³‚¹‚é
			leftMotor->rotate(balanceControl->getLeft());
			rightMotor->rotate(balanceControl->getRight());
		}
		else{
			// ƒ‚[ƒ^‚ð‰ñ“]‚³‚¹‚é
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
		
		/* ƒƒMƒ“ƒO */
		ecrobot_bt_data_logger(0, 0);
}

void Control::basicRun(float forward, float turn, float tail_angle, 
	float gyro_offset, BOOL pid_flag, U16 pid_target, F32 KP, BOOL balance_flag, float gyro, U16 light){

		static float Forward, Turn;

		// ‚µ‚Á‚Û‚ð‰ñ“]‚³‚¹‚é
		tail->control(tail_angle);
		
		// pid_flag‚ªTRUE‚Ì‚Æ‚«APID§Œä‚Åù‰ñ‘¬“x‚ðŒvŽZ‚·‚é
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
		
		// balance_flag‚ªTRUE‚Ì‚Æ‚«A“|—§UŽq§Œä‚Åƒ‚[ƒ^o—Í’l‚ðŒvŽZ‚·‚é
		if(balance_flag){
			balanceControl->calculation((char)Forward, (char)Turn, gyro, gyro_offset, leftMotor->getAngle(), rightMotor->getAngle());
			// ƒ‚[ƒ^‚ð‰ñ“]‚³‚¹‚é
			leftMotor->rotate(balanceControl->getLeft());
			rightMotor->rotate(balanceControl->getRight());
		}
		else{
			// ƒ‚[ƒ^‚ð‰ñ“]‚³‚¹‚é
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
		
		/* ƒƒMƒ“ƒO */
		ecrobot_bt_data_logger(0, 0);
}

// “|—§UŽq§Œä‚Æƒ‚[ƒ^‰ñ“]Šp“x‚Ì‰Šú‰»
void Control::resetBalance(){
	balanceControl->init();
	leftMotor->resetAngle();
	rightMotor->resetAngle();
}
