#ifndef BLUETOOTH_H
#define BLUETOOTH_H

extern "C"{
	#include "ecrobot_interface.h"
	#include "kernel.h"
};

class BlueTooth{
private:
	
public:
	BlueTooth();
	unsigned int isReceived();
};

#endif
