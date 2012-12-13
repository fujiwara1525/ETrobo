#include "ObstacleDetection.h"

#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] */

ObstacleDetection::ObstacleDetection(){
	counter = 0;
}

// 障害物検知を行う
BOOL ObstacleDetection::detect(S32 sonar){
	BOOL alert = FALSE;

	// 約40msec周期毎に障害物検知
	if (counter == 9)
	{
		if ((sonar <= SONAR_ALERT_DISTANCE) && (sonar >= 0))
		{
			alert = TRUE; /* 障害物を検知 */
		}
		else
		{
			alert = FALSE; /* 障害物無し */
		}
		counter = 0;
	}
	else{
		counter++;
	}

	return alert;
}
