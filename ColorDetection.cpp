#include "ColorDetection.h"

ColorDetection::ColorDetection(){
}

// �F���m���s��
BOOL ColorDetection::detect(U16 light, U16 light_threshold, int l){
	// l��0�Ȃ�P�x��臒l�ȏ��TRUE��Ԃ�
	// l��1�Ȃ�P�x��臒l�ȉ���TRUE��Ԃ�
	if(light >= light_threshold && l == 0){
		return TRUE;
	}
	else if(light <= light_threshold && l == 1){
		return TRUE;
	}
	else{
		return FALSE;
	}
}
