#ifndef COLORDETECTION_H
#define COLORDETECTION_H

extern "C"{
	#include "kernel.h"
	#include "ecrobot_interface.h"
};

class ColorDetection{
private:
	
public:
	ColorDetection();
	BOOL detect(U16 light, U16 light_threshold, int l);
};

#endif
