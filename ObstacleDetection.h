#ifndef OBSTACLEDETECTION_H
#define OBSTACLEDETECTION_H

extern "C"{
	#include "kernel.h"
	#include "ecrobot_interface.h"
};

class ObstacleDetection{
private:
	int counter;
public:
	ObstacleDetection();
	BOOL detect(S32 sonar);
};

#endif
