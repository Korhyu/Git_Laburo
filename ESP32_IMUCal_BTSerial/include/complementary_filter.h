#ifndef CF_H
#define CF_H

#define G_CTE		9.807		//Constante de la gravedad

#define CF_ALPHA	0.2f		//Constante alfa de filtro complementario
//#define CF_ALPHA	0.1f		//Constante alfa de filtro complementario

#define RAD2DEG     57.29577951f	//Constante de conversion radianes a grados
#define DEG2RAD     0.017453292f	//Constante de conversion grados a radianes 

#include "Arduino.h"

struct cfData {
	float phiHat_acc;
	float thetaHat_acc;

	float p_rps;
	float q_rps;
	float r_rps;

	float phiDot;
	float thetaDot;

	float phiHat_rad;
	float thetaHat_rad;
	float yaw_rad;

	float phiHat_deg;
	float thetaHat_deg;
	float yaw_deg;
};

/*
cfData cfCalculate (imuData imu);
attitudeData simpleIntegral (imuData imu);
attitudeData simpleEulerIntegral (imuData imu);
*/
String cfData2String (cfData * datos);


#endif