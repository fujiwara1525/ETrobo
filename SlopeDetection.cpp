#include "SlopeDetection.h"

SlopeDetection::SlopeDetection(){
}

// 傾き検知を行う
BOOL SlopeDetection::detect(float gyro, float gyro_threshold, int g){
	// gが0ならジャイロが閾値以上でTRUEを返す
	// gが1ならジャイロが閾値以下でTRUEを返す
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
