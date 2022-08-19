#include "complementary_filter.h"
#include "imu.h"
#include "math.h"

int cfPushData (imuData newData){
    //Inserta los datos nuevos en el buffer del filtro

	return 0;
}


//Metodo de Phills Lab https://www.youtube.com/watch?v=BUW2OdAtzBw
// VERIFICADA
cfData cfCalculate (imuData imu){
    //Realiza los calculos con los datos enviados
	
	/* GLOSARIO
	p, q y r son los angulos del objeto a monitorear, pero no son relativos al marco de referencia
	phiDot, thetaDot, psiDot son los angulos de euler que son los angulos entre el marco de referencia y el objeto
	Estos ultimos son los angulos a mirar

	Hat significa ESTIMADO
	Dot significa
	*/

	//Variables
	float phiHat_acc, thetaHat_acc;
	float p_rps, q_rps, r_rps;
	float phiDot, thetaDot;
	static float phiHat_rad, thetaHat_rad;

	//Estos son los angulos estimados por el acelerometro
	phiHat_acc   = atan(imu.acc_y / imu.acc_z);		
	thetaHat_acc = asin(imu.acc_x / G_CTE);					//Por falta de calibracion este cociente puede dar mas de 1 y por lo tanto el asin puede dar nan

	//Correccion de thetaHat_acc - Cuando se calibre deberiamos eliminar esto
	if(isnan(thetaHat_acc))
	{
		//Factor de ajuste
		float adjFactor = G_CTE / imu.acc_rms;
		thetaHat_acc = asin((imu.acc_x * adjFactor) / G_CTE);
	}


	//Ahora paso al gyroscopo
	p_rps = imu.gyr_x;
	q_rps = imu.gyr_y;
	r_rps = imu.gyr_z;

	//Paso a angulos de Euler
	phiDot   = p_rps + tan(thetaHat_acc) * (sin(phiHat_acc) * q_rps + cos(phiHat_acc) * r_rps);
	thetaDot =   0   +         1         * (cos(phiHat_acc) * q_rps - sin(phiHat_acc) * r_rps);
		
	phiHat_rad	= CF_ALPHA * phiHat_acc   + (1.0 - CF_ALPHA) * (phiHat_rad   + (IMU_SAMPLE_TIME_MS / 1000.0) * phiDot);  
	thetaHat_rad= CF_ALPHA * thetaHat_acc + (1.0 - CF_ALPHA) * (thetaHat_rad + (IMU_SAMPLE_TIME_MS / 1000.0) * thetaDot);

	cfData resultados;

	resultados.phiHat_acc = phiHat_acc;
	resultados.thetaHat_acc = thetaHat_acc;
	resultados.p_rps = p_rps;
	resultados.q_rps = q_rps;
	resultados.r_rps = r_rps;
	resultados.phiDot = phiDot;
	resultados.thetaDot = thetaDot;
	resultados.phiHat_rad = phiHat_rad;
	resultados.thetaHat_rad = thetaHat_rad;
	resultados.yaw_rad = 0;
	resultados.phiHat_deg = phiHat_rad * RAD2DEG;
	resultados.thetaHat_deg = thetaHat_rad * RAD2DEG;
	resultados.yaw_deg = 0 * RAD2DEG;

	return resultados;
}


//Integra las salidas del IMU - VERIFICADA
attitudeData simpleIntegral (imuData imu) {

	static attitudeData resultados;

	resultados.pitch = resultados.pitch + ((IMU_SAMPLE_TIME_MS / 1000.0f) * imu.gyr_x);
	resultados.roll = resultados.roll + ((IMU_SAMPLE_TIME_MS / 1000.0f) * imu.gyr_y);

	return resultados;
}

//Integra las salidas del IMU usando angulos de Euler
// A VERIFICAR - Falta verificar si funciona bien
attitudeData simpleEulerIntegral (imuData imu) {

	static attitudeData resultados;

	float phiHat_acc, thetaHat_acc;
	float phiDot, thetaDot;
	float phiHat_rad, thetaHat_rad;

	//Tomo las mediciones de giroscopo
	float p_rps = imu.gyr_x;
	float q_rps = imu.gyr_y;
	float r_rps = imu.gyr_z;

	//Transformo a Euler
	float phiDot_rps   = p_rps + tan(thetaHat_rad) * (sin(phiHat_rad) * q_rps + cos(phiHat_rad) * r_rps);
	float thetaDot_rps =							 (cos(phiHat_rad) * q_rps - sin(phiHat_rad) * r_rps);

	//Integro los valores de Euler
	phiHat_rad   = phiHat_rad   + ((IMU_SAMPLE_TIME_MS / 1000.0f) * phiDot_rps);
	thetaHat_rad = thetaHat_rad + ((IMU_SAMPLE_TIME_MS / 1000.0f) * thetaDot_rps);

	resultados.pitch = phiHat_rad;
	resultados.roll =  thetaHat_rad;

	return resultados;
}