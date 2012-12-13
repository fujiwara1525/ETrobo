#ifndef SLOPEDETECTION_H
#define SLOPEDETECTION_H

extern "C"{
	#include "kernel.h"
}

class SlopeDetection{
private:
	
public:
	SlopeDetection();
	BOOL detect(float gyro, float gyro_threshold, int g);
};

#endif
