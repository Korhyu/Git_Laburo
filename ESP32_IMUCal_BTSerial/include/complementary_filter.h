#ifndef CF_H
#define CF_H

#define IMU_SAMPLE_TIME_MS 200  //Delay entre muestras del IMU

#define G_CTE       9.80247f    //Acceleracion de la gravedad

#define CF_ALPHA	0.1f		//Constante alfa de filtro complementario

#define RAD2DEG     57.2958f    //Constante de conversion radianes a grados
#define DEG2RAD     0.0174533f  //Constante de conversion grados a radianes 


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






#endif