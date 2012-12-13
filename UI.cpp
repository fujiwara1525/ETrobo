#include "UI.h"

UI::UI(void){
}

UI::UI(TouchSensor _touchSensor, BlueTooth _blueTooth, LightSensor _lightSensor, Tail _tail){
	*touchSensor = _touchSensor;
	*blueTooth = _blueTooth;
	*lightSensor = _lightSensor;
	*tail = _tail;
}

// ���͂�҂�
void UI::waitStart(float angle){
	while(1){
		tail->control(angle);
		if(touchSensor->isPressed()){
			display_goto_xy(6, 5);
			display_string("GO !!");
			display_update();
			break;
		}
		if(blueTooth->isReceived() == 1){
			display_goto_xy(6, 5);
			display_string("GO !!");
			display_update();
			break;
		}
	}
}

// �L�����u���[�V�������s��
U16 UI::calibrate(float angle){
	// �^�b�`�Z���T�̓��͂�҂�
	while(1){
		tail->control(angle);
		if(touchSensor->isPressed()){
			break;
		}
		// 10msec�E�F�C�g����
		systick_wait_ms(10); 	
	}

	// 1000msec�E�F�C�g����
	systick_wait_ms(1000);

	return lightSensor->getBrightness();
}

int UI::courseSelect(){
	int course = -1;

	display_goto_xy(4, 1);

	while(1){
		if(ecrobot_is_RUN_button_pressed()){
			course = course * (-1);

			if(course == 1){
				display_string(" IN COURSE");
				display_goto_xy(4, 1);
			}
			else{
				display_string("OUT COURSE");
				display_goto_xy(4, 1);
			}
		}		

		if(ecrobot_is_ENTER_button_pressed()){
			display_goto_xy(6, 3);
			display_string("READY");
			break;
		}
		
		display_update();
		
		// 500msec�E�F�C�g����
		systick_wait_ms(500); 	
	}

	display_update();

	return course;
}
