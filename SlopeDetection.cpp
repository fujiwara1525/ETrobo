#include "SlopeDetection.h"

SlopeDetection::SlopeDetection(){
}

// �X�����m���s��
BOOL SlopeDetection::detect(float gyro, float gyro_threshold, int g){
	// g��0�Ȃ�W���C����臒l�ȏ��TRUE��Ԃ�
	// g��1�Ȃ�W���C����臒l�ȉ���TRUE��Ԃ�
	if(gyro >= gyro_threshold && g == 0){
		return TRUE;
	}
	else if(gyro <= gyro_threshold && g == 1){
		return TRUE;
	}
	else{
		return FALSE;
	}
}
