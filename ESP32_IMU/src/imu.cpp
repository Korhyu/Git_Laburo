
#include "imu.h"

/*
void printIMUData (imuData * datos){
	static int headers = 0;

	if (headers == 0)
	{
		headers = 1;
		Serial.println("acc_X;acc_Y;acc_Z;gyr_X;gyr_Y;gyr_Z;acc_RMS");
	}

	Serial.print(datos->acc_x,4);
	Serial.print(";");
	Serial.print(datos->acc_y,4);
	Serial.print(";");
	Serial.print(datos->acc_z,4);
	Serial.print(";");
	Serial.print(datos->gyr_x,4);
	Serial.print(";");
	Serial.print(datos->gyr_y,4);
	Serial.print(";");
	Serial.print(datos->gyr_z,4);
	Serial.print(";");
	Serial.println(datos->acc_rms,4);
}

void printCFData (cfData * datos){
	Serial.println("phiHat_acc / thetaHat_acc");
	Serial.print(datos->phiHat_acc);
	Serial.print("\t");
	Serial.println(datos->thetaHat_acc);

	Serial.println("p_rps / q_rps / r_rps");
	Serial.print(datos->p_rps);
	Serial.print("\t");
	Serial.print(datos->q_rps);
	Serial.print("\t");
	Serial.println(datos->r_rps);

	Serial.println("phiDot / thetaDot");
	Serial.print(datos->phiDot);
	Serial.print("\t");
	Serial.println(datos->thetaDot);
	
	Serial.println("phiHat_rad / thetaHat_rad");
	Serial.print(datos->phiHat_rad);
	Serial.print("\t");
	Serial.println(datos->thetaHat_rad);

	Serial.println("*******************************************");
	Serial.println("*******************************************");
	Serial.println("*******************************************");
}

void printAllData (imuData * imu_raw, imuData * imu_fil, cfData * cf){
	static int headers = 0;

	if (headers == 0)
	{
		headers = 1;
		Serial.print("accX_RAW;accY_RAW;accZ_RAW;gyrX_RAW;gyrY_RAW;gyrZ_RAW;");
		Serial.print("accX_FIL;accY_FIL;accZ_FIL;gyrX_FIL;gyrY_FIL;gyrZ_FIL;");
		Serial.println("phiHat_acc;thetaHat_acc;p_rps;q_rps;r_rps;phiDot;thetaDot;phiHat_rad;thetaHat_rad");
	}
	//Datos IMU crudos 
	Serial.print(imu_raw->acc_x,6);
	Serial.print(";");
	Serial.print(imu_raw->acc_y,6);
	Serial.print(";");
	Serial.print(imu_raw->acc_z,6);
	Serial.print(";");
	Serial.print(imu_raw->gyr_x,6);
	Serial.print(";");
	Serial.print(imu_raw->gyr_y,6);
	Serial.print(";");
	Serial.print(imu_raw->gyr_z,6);
	Serial.print(";");

	//Datos IMU filtrados
	Serial.print(imu_fil->acc_x,6);
	Serial.print(";");
	Serial.print(imu_fil->acc_y,6);
	Serial.print(";");
	Serial.print(imu_fil->acc_z,6);
	Serial.print(";");
	Serial.print(imu_fil->gyr_x,6);
	Serial.print(";");
	Serial.print(imu_fil->gyr_y,6);
	Serial.print(";");
	Serial.print(imu_fil->gyr_z,6);
	Serial.print(";");

	//Datos filtro complementario
	Serial.print(cf->phiHat_acc,6);
	Serial.print(";");
	Serial.print(cf->thetaHat_acc,6);
	Serial.print(";");
	Serial.print(cf->p_rps,6);
	Serial.print(";");
	Serial.print(cf->q_rps,6);
	Serial.print(";");
	Serial.print(cf->r_rps,6);
	Serial.print(";");
	Serial.print(cf->phiDot,6);
	Serial.print(";");
	Serial.print(cf->thetaDot,6);
	Serial.print(";");
	Serial.print(cf->phiHat_rad,6);
	Serial.print(";");
	Serial.println(cf->thetaHat_rad,6);
}

void plotAttitude (imuData * imu_raw, imuData * imu_fil, attitudeData * attitude){
	static int headers = 0;

	if (headers == 0)
	{
		headers = 1;
		Serial.print("accX_RAW;accY_RAW;accZ_RAW;gyrX_RAW;gyrY_RAW;gyrZ_RAW;");
		Serial.print("accX_FIL;accY_FIL;accZ_FIL;gyrX_FIL;gyrY_FIL;gyrZ_FIL;");
		Serial.println("pitch;roll");
	}

	//Datos IMU filtrados
	Serial.print(imu_fil->acc_x,3);
	Serial.print(";");
	Serial.print(imu_fil->acc_y,3);
	Serial.print(";");
	Serial.print(imu_fil->acc_z,3);
	Serial.print(";");
	Serial.print(imu_fil->gyr_x,3);
	Serial.print(";");
	Serial.print(imu_fil->gyr_y,3);
	Serial.print(";");
	Serial.print(imu_fil->gyr_z,3);
	Serial.print(";");

	//Datos filtro complementario
	Serial.print(attitude->pitch,2);
	Serial.print(";");
	Serial.println(attitude->roll,2);
}

void plotAttitude (cfData * cfDatos){
	static int headers = 0;

	if (headers == 0)
	{
		headers = 1;
		Serial.println("pitch;roll;yaw");
	}
	
	//Datos filtro complementario
	Serial.print(cfDatos->phiHat_deg,2);
	Serial.print(";");
	Serial.print(cfDatos->thetaHat_deg,2);
	Serial.print(";");
	Serial.println(cfDatos->yaw_deg,2);
}
*/
