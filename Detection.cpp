#include "Detection.h"

Detection::Detection(){
}

Detection::Detection(float _gyro_threshold, int _g, U16 _light_threshold, int _l, int _s, int _count,
	SlopeDetection _slopeDetection, ColorDetection _colorDetection, ObstacleDetection _obstacleDetection){
	gyro_threshold = _gyro_threshold;
	g = _g;
	light_threshold = _light_threshold;
	l = _l;
	s = _s;
	count = _count;
	counter = 0;
	*slopeDetection = _slopeDetection;
	*colorDetection = _colorDetection;
	*obstacleDetection = _obstacleDetection;
}

// ŠeŒŸ’m‚ðs‚¤
BOOL Detection::detect(float gyro, U16 light, S32 sonar){
	int detection;

	detection = 0;

	if(counter++ >= count){
		while(detection < 3){
			switch(detection){
			case 0:
				if(g == -1){
					detection = 1;
				}
				else{
					if(slopeDetection->detect(gyro, gyro_threshold, g)){
						detection = 1;
					}
					else{
						return FALSE;
					}
				}
				break;
			case 1:
				if(s == -1){
					detection = 2;
				}
				else{
					if(obstacleDetection->detect(sonar)){
						detection = 2;
					}
					else{
						return FALSE;
					}
				}
				break;
			case 2:
				if(l == -1){
					detection = 3;
				}
				else{
					if(colorDetection->detect(light, light_threshold, l)){
						detection = 3;
					}
					else{
						return FALSE;
					}
				}
				break;
			}
	
		}
		return TRUE;
	}
	else{
		return FALSE;
	}
}

BOOL Detection::searchObstacle(S32 sonar){
	return obstacleDetection->detect(sonar);
}
