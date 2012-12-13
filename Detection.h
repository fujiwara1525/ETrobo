#ifndef DETECTION_H
#define DETECTION_H

#include <stdlib.h>
#include "SlopeDetection.h"
#include "ColorDetection.h"
#include "ObstacleDetection.h"

extern "C"{
	#include "kernel.h"
	#include "ecrobot_interface.h"
};

class Detection{
private:
	float gyro_threshold;
	int g;
	U16 light_threshold;
	int l;
	int s;
	int count;
	int counter;
	SlopeDetection *slopeDetection;
	ColorDetection *colorDetection;
	ObstacleDetection *obstacleDetection;
public:
	Detection();
	Detection(float _gyro_threshold, int _g, U16 _light_threshold, int _l, int _s, int _count,
		SlopeDetection _slopeDetection, ColorDetection _colorDetection, ObstacleDetection _obstacleDetection);
	BOOL detect(float gyro, U16 light, S32 sonar);
	BOOL searchObstacle(S32 sonar);
};

#endif
