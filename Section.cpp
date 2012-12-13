#include "Section.h"

Section::Section(){
}

Section::Section(float _forward, float _forward_var, float _turn, float _tail, float _tail_var, 
	int _gyro_offset, BOOL _pid_flag, U16 _pid_target, F32 _KP, BOOL _balance_flag, BOOL _balance_init,
	Input _input, Control _control, Detection _detection){
		forward = _forward;
		forward_var = _forward_var;
		turn = _turn;
		tail = _tail;
		tail_var = _tail_var;
		gyro_offset = _gyro_offset;
		pid_flag = _pid_flag;
		pid_target = _pid_target;
		KP = _KP;
		balance_flag = _balance_flag;
		balance_init = _balance_init;
		
		*input = _input;
		*control = _control;
		*detection = _detection;
		
		end = TRUE;
}

void Section::setNext(Section _next){
	*next = _next;
	end = FALSE;
}

Section Section::getNext(){
	return *next;
}

BOOL Section::getEnd(){
	return end;
}

// ���s����
void Section::bonusRun(){
	if(balance_init){
		control->resetBalance();
		balance_init = FALSE;
	}

	while(1){
		// �e�Z���T������͒l�𓾂�
		input->getInput();

		// forward���v�Z����
		forward += forward_var;
		if(forward >= 100){
			forward = 100;
		}
		else if(forward <= -100){
			forward = -100;
		}

		// tail���v�Z����
		tail += tail_var;

		// Control�Ɋe����Əo�͂��˗�����
		control->bonusRun(
			forward,
			turn,
			tail,
			gyro_offset,
			pid_flag,
			pid_target,
			KP,
			balance_flag,
			input->getGyroSensor(),
			input->getBrightness()
			);

		// ��ԑJ�ڏ����𖞂����Ă���ꍇ�A���̋�Ԃ̑��s���I������
		if(detection->detect(
			input->getGyroSensor(), 
			input->getBrightness(),
			input->getSonarSensor()
			)){
			break;
		}

		// 4msec�E�F�C�g����
		systick_wait_ms(4);
	}
}

void Section::basicRun(){
	if(balance_init){
		control->resetBalance();
		balance_init = FALSE;
	}
	
	while(1){
		// �e�Z���T������͒l�𓾂�
		input->getInput();

		// forward���v�Z����
		forward += forward_var;
		if(forward >= 100){
			forward = 100;
		}
		else if(forward <= -100){
			forward = -100;
		}

		// tail���v�Z����
		tail += tail_var;

		// Control�Ɋe����Əo�͂��˗�����
		control->basicRun(
			forward,
			turn,
			tail,
			gyro_offset,
			pid_flag,
			pid_target,
			KP,
			balance_flag,
			input->getGyroSensor(),
			input->getBrightness()
			);

		// ��ԑJ�ڏ����𖞂����Ă���ꍇ�A���̋�Ԃ̑��s���I������
		if(detection->detect(
			input->getGyroSensor(), 
			input->getBrightness(),
			input->getSonarSensor()
			)){
			break;
		}

		// 4msec�E�F�C�g����
		systick_wait_ms(4);
	}
}

float Section::searchObstacle(){
	static int count = 0;
	static float search = 1;

	while(count <= 250){
		// �e�Z���T������͒l�𓾂�
		input->getInput();
		
		control->bonusRun(
			forward,
			turn,
			tail,
			gyro_offset,
			pid_flag,
			pid_target,
			KP,
			balance_flag,
			input->getGyroSensor(),
			input->getBrightness()
			);

		// 4msec�E�F�C�g����
		systick_wait_ms(4);

		count++;
	}

	while(count <= 500){
		// �e�Z���T������͒l�𓾂�
		input->getInput();

		control->bonusRun(
			forward,
			turn,
			tail,
			gyro_offset,
			pid_flag,
			pid_target,
			KP,
			balance_flag,
			input->getGyroSensor(),
			input->getBrightness()
			);

		if(detection->searchObstacle(input->getSonarSensor())){
			search = -1;
		}

		// 4msec�E�F�C�g����
		systick_wait_ms(4);

		count++;

	}

		return search;
}
