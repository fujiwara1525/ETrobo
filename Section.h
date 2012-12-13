#ifndef SECTION_H
#define SECTION_H

extern "C"{
	#include "kernel.h"
	#include "ecrobot_interface.h"
};

#include "Input.h"
#include "Control.h"
#include "Detection.h"

class Section{
private:
	float forward;
	float forward_var;
	float turn;
	float tail;
	float tail_var;
	int gyro_offset;
	BOOL pid_flag;
	U16 pid_target;
	F32 KP;
	BOOL balance_flag;
	BOOL balance_init;

	Input *input;
	Control *control;
	Detection *detection;
	
	Section *next;
	BOOL end;
public:
	Section();
	Section(float _forward, float _forward_var, float _turn, float _tail, float _tail_var, 
		int _gyro_offset, BOOL _pid_flag, U16 _pid_target, F32 _KP, BOOL _balance_flag, BOOL _balance_init,
	Input _input, Control _control, Detection _detection);
	void setNext(Section _next);
	Section getNext();
	BOOL getEnd();
	void bonusRun();
	void basicRun();
	float searchObstacle();
};

#endif
