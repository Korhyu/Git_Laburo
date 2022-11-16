#include "imu.h"



/********************* VARIABLES *********************/
static int stateIMU = IN_USE;
static int calibrationSubState = CALSUB_PRINT_AXIS;
static int axis = CAL_Zp;

extern int imuReadTimer;
extern String bufferOutBT;			//Buffer de salida a BT

imuData sampleIMU;
imuData imuCalValues[IMU_CAL_SIZE*6];
imuData imuAux;

cfData  cfLastData;

RCFilter lpfAcc[4];
RCFilter lpfGyr[4];


int IMUmachineState ()
{
	static int i = 0;			//Contador auxiliar
	/*
	TODO Falta cerrar la logica de como manejar la calibracion.
	Estas sobre el final ya tomando las muestras y analizando como hacer para
	no depender del delay y usar todo con la maquina de estados y guardar las
	muestras para despues computar la matriz.
	*/

	switch (stateIMU)
	{
		case UNCALIBRATED:
			//El IMU esta descalibrado y debo avisar que se tiene que calibrar.
			Serial.println("IMU descalibrado comenzando calibracion");
			stateIMU = IN_CALIBRATION;
			
			break;
		
		case IN_CALIBRATION:
			switch (calibrationSubState)
			{
				case CALSUB_PRINT_AXIS:
					switch (axis)
					{
						case CAL_Zp:
							//Comienzo por calibrar el eje +Z
							Serial.println("Calibrando +Z");
							calibrationSubState = CALSUB_ALIGINING;
							imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
							break;
							
						case CAL_Zn:
							Serial.println("Calibrando -Z");
							calibrationSubState = CALSUB_ALIGINING;
							imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
							break;
							
						case CAL_Xp:
							Serial.println("Calibrando +X");
							calibrationSubState = CALSUB_ALIGINING;
							imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
							break;

						case CAL_Xn:
							Serial.println("Calibrando -X");
							calibrationSubState = CALSUB_ALIGINING;
							imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
							break;

						case CAL_Yp:
							Serial.println("Calibrando +Y");
							calibrationSubState = CALSUB_ALIGINING;
							imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
							break;

						case CAL_Yn:
							Serial.println("Calibrando -Y");
							calibrationSubState = CALSUB_ALIGINING;
							imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
							break;
					}
					break;

				case CALSUB_ALIGINING:
					if( i<20 )
					{
						imuAux = getIMUSample();
						Serial.printf("AccX = %f \t AccY = %f \t AccZ = %f\n",imuAux.acc_x, imuAux.acc_y, imuAux.acc_z);
						imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
						i++;
					}
					else
					{
						//Termino el tiempo de alineacion y comienzo a tomar muestras
						calibrationSubState = CALSUB_TAKING_SAMP;
						i=0;
						Serial.println("Tomando muestras");
					}
					break;

				case CALSUB_TAKING_SAMP:
					if ( i < IMU_CAL_SIZE)
					{
						//Guardo todos los valores para poder medir los K tambien
						imuCalValues[i+(axis*IMU_CAL_SIZE)] = getIMUSample();
						i++;
						if( (i % 50) == 0)
							Serial.printf("Muestra %d \n", i);
					
						imuReadTimer = timerLoad(IMU_CAL_SAMP_MS);
					}
					else
					{
						//Paso al siguiente eje
						axis++;
						i=0;
						if (axis > CAL_Yn)
						{
							//Termine con los ejes, paso a calcular los valores
							calibrationSubState = CALSUB_CALCULATE;
							axis = CAL_Zp;
						}
						else
						{
							//Si no llegue al ultimo eje repito
							calibrationSubState = CALSUB_PRINT_AXIS;
						}
					}
					break;

				case CALSUB_CALCULATE:
					//Tome todas las muestras y puedo pasar a calcular los valores de BIAS y Ks

					//Primero encuentro los maximos y minimos
					float accMAX[3] = {0,0,0};
					float accMIN[3] = {0,0,0};
					float avgMAX[3] = {0,0,0};
					float avgMIN[3] = {0,0,0};
					float accCalBiasNew[3];
					float accCalFactorNew[3];
				
					//Busco maximos y minimos en los 3+3 ejes
					for( i=0 ; i < IMU_CAL_SIZE*6 ; i++ )
					{
						int aux = i/IMU_CAL_SIZE;	//Esto me da el numero de eje que estoy calibrando

						/* Si aux es "par", estoy buscando un maximo porque
						primero calibre los ejes + y despues los -
						0 - 2 - 4 son Zp Xp y Yp
						1 - 3 - 5 son Zn Xn y Yn
						*/

						switch (aux)
						{
						case 0:
							// Busco el Zp maximo
							if(accMAX[2] > imuCalValues[i].acc_z)
								accMAX[2] = imuCalValues[i].acc_z;
							//Ademas calculo la suma para despues calcular el promedio
							avgMAX[2] += imuCalValues[i].acc_z;
							break;
						
						case 1:
							// Busco el Zn maximo
							if(accMIN[2] < imuCalValues[i].acc_z)
								accMIN[2] = imuCalValues[i].acc_z;
							//Ademas calculo la suma para despues calcular el promedio
							avgMIN[2] += imuCalValues[i].acc_z;
							break;
						
						case 2:
							// Busco el Xp maximo
							if(accMAX[0] > imuCalValues[i].acc_x)
								accMAX[0] = imuCalValues[i].acc_x;
							//Ademas calculo la suma para despues calcular el promedio
							avgMAX[0] += imuCalValues[i].acc_x;
							break;
						
						case 3:
							// Busco el Xn maximo
							if(accMIN[0] < imuCalValues[i].acc_x)
								accMIN[0] = imuCalValues[i].acc_x;
							//Ademas calculo la suma para despues calcular el promedio
							avgMIN[0] += imuCalValues[i].acc_z;
							break;
						
						case 4:
							// Busco el Yp maximo
							if(accMAX[1] > imuCalValues[i].acc_y)
								accMAX[1] = imuCalValues[i].acc_y;
							//Ademas calculo la suma para despues calcular el promedio
							avgMAX[1] += imuCalValues[i].acc_y;
							break;
						
						case 5:
							// Busco el Yn maximo
							if(accMIN[1] < imuCalValues[i].acc_y)
								accMIN[1] = imuCalValues[i].acc_y;
							//Ademas calculo la suma para despues calcular el promedio
							avgMIN[1] += imuCalValues[i].acc_y;
							break;
						
						
						default:
							break;
						}
					}

					//Ya tengo los maximos y minimos y sumas paso a calcular los BIAS
					avgMAX[0] = avgMAX[0] / IMU_CAL_SIZE;
					avgMAX[1] = avgMAX[1] / IMU_CAL_SIZE;
					avgMAX[2] = avgMAX[2] / IMU_CAL_SIZE;

					avgMIN[0] = avgMIN[0] / IMU_CAL_SIZE;
					avgMIN[1] = avgMIN[1] / IMU_CAL_SIZE;
					avgMIN[2] = avgMIN[2] / IMU_CAL_SIZE;

					accCalBiasNew[0] = (avgMAX[0] + avgMIN[0]) / 2;
					accCalBiasNew[1] = (avgMAX[1] + avgMIN[1]) / 2;
					accCalBiasNew[2] = (avgMAX[2] + avgMIN[2]) / 2;

					//Con los Bias calculados puedo calcular los K, pero primero calculo los nuevos promedios
					avgMAX[0] = avgMAX[0] + accCalBiasNew[0];
					avgMAX[1] = avgMAX[1] + accCalBiasNew[1];
					avgMAX[2] = avgMAX[2] + accCalBiasNew[2];
					avgMIN[0] = avgMIN[0] + accCalBiasNew[0];
					avgMIN[1] = avgMIN[1] + accCalBiasNew[1];
					avgMIN[2] = avgMIN[2] + accCalBiasNew[2];

					accCalFactorNew[0] = GRAV_CTE / avgMAX[0];
					accCalFactorNew[1] = GRAV_CTE / avgMAX[1];
					accCalFactorNew[2] = GRAV_CTE / avgMAX[2];

					Serial.printf("accCalBiasNew[0] = %f, accCalBiasNew[1] = %f, accCalBiasNew[2] = %f",
									accCalBiasNew[0], accCalBiasNew[1], accCalBiasNew[2] );
					Serial.printf("\n");
					Serial.printf("accCalFactorNew[0] = %f, accCalFactorNew[1] = %f, accCalFactorNew[2] = %f",
									accCalFactorNew[0], accCalFactorNew[1], accCalFactorNew[2] );
					Serial.printf("\n");

					stateIMU = IN_USE;

					break;
			}
			break;


		case IN_USE:
			//Tomando mediciones y realizo los calculos
			sampleIMU = getIMUSample();						//Tomo la muestra del sensor
			//printIMUData(&sampleIMU);
			sampleIMU = compensateIMUValues(&sampleIMU);	//Compenso BIAS y k
			//printIMUData(&sampleIMU);
			sampleIMU = filterIMUSample(&sampleIMU);		//LPF las señales
			//printIMUData(&sampleIMU);
			cfLastData = cfCalculate(&sampleIMU);			//Aplico filtro complementario
			//printCFData(&cfLastData);

			//Publico el dato por BT
			bufferOutBT = cfData2String(&cfLastData);		//TODO: Esta llegando nan al BT
			
			imuReadTimer = timerLoad(IMU_SAMPLE_TIME_MS);

			break;
		
		default:
			break;
	}

	return 0;
}

imuData getIMUSample (void)
{
	extern ICM42688 IMU;
	imuData raw;

	// read the sensor
	IMU.readSensor();
		
	//Modulo de Acc
	raw.acc_x = IMU.getAccelX_mss();
	raw.acc_y = IMU.getAccelY_mss();
	raw.acc_z = IMU.getAccelZ_mss();
	raw.acc_rms = sqrt(raw.acc_x*raw.acc_x + raw.acc_y*raw.acc_y + raw.acc_z*raw.acc_z);
	raw.gyr_x = IMU.getGyroX_rads();
	raw.gyr_y = IMU.getGyroY_rads();
	raw.gyr_z = IMU.getGyroZ_rads();
	raw.temp = IMU.getTemperature_C();

	return raw;
}

imuData compensateIMUValues (imuData * raw)
{
	imuData comp;
	comp.acc_x = (raw->acc_x - accCalBias[0]) * accCalFactor[0];
	comp.acc_y = (raw->acc_y - accCalBias[1]) * accCalFactor[1];
	comp.acc_z = (raw->acc_z - accCalBias[2]) * accCalFactor[2];
	comp.acc_rms = sqrt(comp.acc_x*comp.acc_x + comp.acc_y*comp.acc_y + comp.acc_z*comp.acc_z);
	comp.gyr_x = (raw->gyr_x - gyrCalBias[0]) * gyrCalFactor[0];
	comp.gyr_y = (raw->gyr_y - gyrCalBias[1]) * gyrCalFactor[1];
	comp.gyr_z = (raw->gyr_z - gyrCalBias[2]) * gyrCalFactor[2];
	comp.temp = raw->temp;

	return comp;
}

imuData filterIMUSample (imuData * comp)
{
	imuData filt;

	//Filtro los datos
	filt.acc_x = RCFilter_Update(&lpfAcc[0], comp->acc_x);
	filt.acc_y = RCFilter_Update(&lpfAcc[1], comp->acc_y);
	filt.acc_z = RCFilter_Update(&lpfAcc[2], comp->acc_z);
	filt.acc_rms = RCFilter_Update(&lpfAcc[3], comp->acc_rms);
	filt.gyr_x = RCFilter_Update(&lpfGyr[0], comp->gyr_x);
	filt.gyr_y = RCFilter_Update(&lpfGyr[1], comp->gyr_y);
	filt.gyr_z = RCFilter_Update(&lpfGyr[2], comp->gyr_z);

	return filt;
}






void configRCFilters (void)
{
	//Configuro los filtros
	for (uint8_t n = 0; n < 4; n++) {
		RCFilter_Init(&lpfAcc[n], RC_FCUOFF_ACC, (float(IMU_SAMPLE_TIME_MS)/1000) );
		RCFilter_Init(&lpfGyr[n], RC_FCUOFF_GYR, (float(IMU_SAMPLE_TIME_MS)/1000) );
	}
}

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
	
	/*
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
	*/

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
