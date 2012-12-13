#include "ColorDetection.h"

ColorDetection::ColorDetection(){
}

// 色検知を行う
BOOL ColorDetection::detect(U16 light, U16 light_threshold, int l){
	// lが0なら輝度が閾値以上でTRUEを返す
	// lが1なら輝度が閾値以下でTRUEを返す
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
