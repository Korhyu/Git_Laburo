#include "imu.h"



/********************* VARIABLES *********************/
int stateIMU = UNCALIBRATED;
int stateCalibration = UNCALIBRATED;
int axis = 0;

extern int imuReadTimer;

imuData rawIMU;
imuData calIMU;
imuData filteredIMU;
imuData imuCalValues[IMU_CAL_SIZE*6];
imuData imuAux;

cfData  cfLastData;

RCFilter lpfAcc[4];
RCFilter lpfGyr[4];


int machineStateIMU ()
{
	int i;			//Contador auxiliar
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
			
			break;
		
		case IN_CALIBRATION:

			switch (stateCalibration)
			{
				case CAL_Zp:
					//Comienzo por calibrar el eje +Z
					Serial.println("Calibrando +Z");
					stateCalibration = ALIGINING;
					imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
					break;
					
				case CAL_Zn:
					Serial.println("Calibrando -Z");
					stateCalibration = ALIGINING;
					imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
					break;
					
				case CAL_Xp:
					Serial.println("Calibrando +X");
					stateCalibration = ALIGINING;
					imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
					break;

				case CAL_Xn:
					Serial.println("Calibrando -X");
					stateCalibration = ALIGINING;
					imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
					break;

				case CAL_Yp:
					Serial.println("Calibrando +Y");
					stateCalibration = ALIGINING;
					imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
					break;

				case CAL_Yn:
					Serial.println("Calibrando -Y");
					stateCalibration = ALIGINING;
					imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
					break;

				case ALIGINING:
					Serial.println("Poner en posicion...");
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
						stateCalibration = TAKING_SAMPLES;
						i=0;
						Serial.println("Tomando muestras");
					}
					break;

				case TAKING_SAMPLES:
						if ( i < IMU_CAL_SIZE)
						{
							//Forma nueva, guardo todos los valores para poder medir los K tambien
							imuCalValues[i+(axis*IMU_CAL_SIZE)] = getIMUSample();
							i++;

							if( (i % 50) == 0)
								Serial.printf("Muestra %d \n", i);
						
							imuReadTimer = timerLoad(IMU_READ_MSJ_DELAY);
						}
						else
						{
							stateCalibration++;
							if (stateCalibration == SAMPLES_TAKED)
							{
								//Tome todas las muestras y puedo pasar a calcular los valores

								//TODO hacer los calculos de calibracion.
							}
						
							//Paso al siguiente eje
							axis++;

						}

						break;
/* ACA TENES QUE CONTINUAR
Tengo 2 modos, uno que es almacenando muestras y el otro que es mostrandolas en terminal
La idea de estos 2 modos es que cuando estoy acomodando el sensor no guardo las muestras
Pero cuando el sensor esta en posicion y recivi la orden de almacenar dejo de mostrar y almaceno
*/
/*
if ( storeIMUData == true )
{
	//Cada TIEMPO_ENTRE_MUESTRAS_MIN min guardo un valor
	if ((cont % ((60000/IMU_SAMPLE_TIME_MS)*TIEMPO_ENTRE_MUESTRAS_MIN)) == 0)
	{
		//Cada X min guardo un valor
		cfVectorData[aux].phiHat_deg = cfLastData.phiHat_deg;
		cfVectorData[aux].thetaHat_deg = cfLastData.thetaHat_deg;
		cfVectorData[aux].yaw_deg = cfLastData.yaw_deg;

		aux++;

		if ( (aux % 10) == 0 )
		{
			//Imprimo por que muestra estoy
			Serial.println(" ********** Vector de muestras ********** ");
			Serial.println("pitch;roll;yaw");
		}

		if (aux >= CANTIDAD_MUESTRAS)
		{
			//Imprimo los datos y termino el programa
			while(1)
			{
				//Llene el buffer y lo envio
				Serial.println(" ********** Vector de muestras ********** ");
				Serial.println("pitch;roll;yaw");

				for(aux = 0 ; aux < CANTIDAD_MUESTRAS ; aux++)
				{
					plotAttitude(&cfVectorData[aux]);
				}

				
				delay(20000);
			}
		}
	}
}
else
{	
	//printAllData(&rawIMU, &filteredIMU, &cfLastData);
	//plotAttitude(&rawIMU, &filteredIMU, &lastAttitude);
	plotAttitude(&cfLastData);
}
*/


			}
			
			break;

		

		case IN_USE:
			//Tomando mediciones y usandolas
			rawIMU = getIMUSample();					//Tomo la muestra del sensor
			calIMU = compensateIMUValues(rawIMU);		//Compenso BIAS y k
			filteredIMU = filterIMUSample(calIMU);		//LPF las señales
			cfLastData = cfCalculate(filteredIMU);		//Aplico filtro complementario
			
			break;

		
		
		default:
			break;
	}

	//Corrijo segun la calibracion
	


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

imuData compensateIMUValues (imuData raw)
{
	imuData comp;
	
	comp.acc_x = (raw.acc_x - accCalBias[0]) * accCalFactor[0];
	comp.acc_y = (raw.acc_y - accCalBias[1]) * accCalFactor[1];
	comp.acc_z = (raw.acc_z - accCalBias[2]) * accCalFactor[2];
	comp.acc_rms = sqrt(comp.acc_x*comp.acc_x + comp.acc_y*comp.acc_y + comp.acc_z*comp.acc_z);
	comp.gyr_x = (raw.gyr_x - gyrCalBias[0]) * gyrCalFactor[0];
	comp.gyr_y = (raw.gyr_y - gyrCalBias[1]) * gyrCalFactor[1];
	comp.gyr_z = (raw.gyr_z - gyrCalBias[2]) * gyrCalFactor[2];
	comp.temp = raw.temp;

	return comp;
}

imuData filterIMUSample (imuData comp)
{
	imuData filt;

	//Filtro los datos
	filt.acc_x = RCFilter_Update(&lpfAcc[0], comp.acc_x);
	filt.acc_y = RCFilter_Update(&lpfAcc[1], comp.acc_y);
	filt.acc_z = RCFilter_Update(&lpfAcc[2], comp.acc_z);
	filt.acc_rms = RCFilter_Update(&lpfAcc[3], comp.acc_rms);
	filt.gyr_x = RCFilter_Update(&lpfGyr[0], comp.gyr_x);
	filt.gyr_y = RCFilter_Update(&lpfGyr[1], comp.gyr_y);
	filt.gyr_z = RCFilter_Update(&lpfGyr[2], comp.gyr_z);

	return filt;
}






void configRCFilters (void)
{
	//Configuro los filtros
	for (uint8_t n = 0; n < 4; n++) {
		RCFilter_Init(&lpfAcc[n], 5.0f, (float(IMU_SAMPLE_TIME_MS)/1000) );
		RCFilter_Init(&lpfGyr[n], 25.0f, (float(IMU_SAMPLE_TIME_MS)/1000) );
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
