#include "ObstacleDetection.h"

#define SONAR_ALERT_DISTANCE 30 /* �����g�Z���T�ɂ���Q�����m����[cm] */

ObstacleDetection::ObstacleDetection(){
	counter = 0;
}

// ��Q�����m���s��
BOOL ObstacleDetection::detect(S32 sonar){
	BOOL alert = FALSE;

	// ��40msec�������ɏ�Q�����m
	if (counter == 9)
	{
		if ((sonar <= SONAR_ALERT_DISTANCE) && (sonar >= 0))
		{
			alert = TRUE; /* ��Q�������m */
		}
		else
		{
			alert = FALSE; /* ��Q������ */
		}
		counter = 0;
	}
	else{
		counter++;
	}

	return alert;
}
