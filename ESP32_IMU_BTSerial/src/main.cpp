#include <Arduino.h>
#include "BluetoothSerial.h"
#include "ICM42688.h"
#include "string.h"
#include "math.h"
#include "imu.h"
#include "complementary_filter.h"
#include "filters.h"
#include "timerFunc.h"
#include "bluetoothFunc.h"


#define CANTIDAD_MUESTRAS			480
#define TIEMPO_ENTRE_MUESTRAS_MIN	1
#define AVG_BUFFER_DEEP				(TIEMPO_ENTRE_MUESTRAS_MIN*60*1000)/IMU_SAMPLE_TIME_MS


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// an ICM42688 object with the ICM42688 sensor on I2C bus 0 with address 0x68
ICM42688 IMU(Wire,0x68);

int status;


int cont=0;
int aux=0;
int buffer_aux=0;

int imuReadTimer = 0;
int btWriteimer = 0;

imuData rawIMU;
imuData filteredIMU;
cfData  cfLastData;
cfData  cfVectorData[CANTIDAD_MUESTRAS];
cfData  cfAVGData[AVG_BUFFER_DEEP];
attitudeData lastAttitude;

RCFilter lpfAcc[4];
RCFilter lpfGyr[4];

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

void plotAttitudeBT (cfData * cfDatos){
	static int headers = 0;

	if (headers == 0)
	{
		headers = 1;
		SerialBT.println("pitch;roll;yaw");
	}
	
	//Datos filtro complementario
	SerialBT.print(cfDatos->phiHat_deg,2);
	SerialBT.print(";");
	SerialBT.print(cfDatos->thetaHat_deg,2);
	SerialBT.print(";");
	SerialBT.println(cfDatos->yaw_deg,2);
}

int btConfig ()
{
	SerialBT.begin("SmarComm_DevKit"); //Bluetooth device name
	Serial.println("The device started, now you can pair it with bluetooth!");

	return 0;
}


void setup() {
	// serial to display data
	Serial.begin(460800);
	while(!Serial) {}

	// start communication with IMU
	status = IMU.begin();
	if (status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("Status: ");
		Serial.println(status);
		while(1) {}
	}


	//Configuro los filtros
	for (uint8_t n = 0; n < 4; n++) {
		RCFilter_Init(&lpfAcc[n], 5.0f, (float(IMU_SAMPLE_TIME_MS)/1000) );
		RCFilter_Init(&lpfGyr[n], 25.0f, (float(IMU_SAMPLE_TIME_MS)/1000) );
	}

	btConfig();

	//Hago la primer carga de timers
	imuReadTimer = timerLoad(IMU_SAMPLE_TIME_MS);
	btWriteimer = timerLoad(IMU_SAMPLE_TIME_MS+10);
}

void loop() {
	//Lectura del IMU
	if( timerIsCompleted(imuReadTimer) )
	{
		// read the sensor
		IMU.readSensor();
			
		//Modulo de Acc
		rawIMU.acc_x = (IMU.getAccelX_mss() - accCalBias[0]) * accCalFactor[0];
		rawIMU.acc_y = (IMU.getAccelY_mss() - accCalBias[1]) * accCalFactor[1];
		rawIMU.acc_z = (IMU.getAccelZ_mss() - accCalBias[2]) * accCalFactor[2];
		rawIMU.acc_rms = sqrt(rawIMU.acc_x*rawIMU.acc_x + rawIMU.acc_y*rawIMU.acc_y + rawIMU.acc_z*rawIMU.acc_z);
		rawIMU.gyr_x = IMU.getGyroX_rads();
		rawIMU.gyr_y = IMU.getGyroY_rads();
		rawIMU.gyr_z = IMU.getGyroZ_rads();
		rawIMU.temp = IMU.getTemperature_C();

		//Filtro los datos
		filteredIMU.acc_x = RCFilter_Update(&lpfAcc[0], rawIMU.acc_x);
		filteredIMU.acc_y = RCFilter_Update(&lpfAcc[1], rawIMU.acc_y);
		filteredIMU.acc_z = RCFilter_Update(&lpfAcc[2], rawIMU.acc_z);
		filteredIMU.acc_rms = RCFilter_Update(&lpfAcc[3], rawIMU.acc_rms);
		filteredIMU.gyr_x = RCFilter_Update(&lpfGyr[0], rawIMU.gyr_x);
		filteredIMU.gyr_y = RCFilter_Update(&lpfGyr[1], rawIMU.gyr_y);
		filteredIMU.gyr_z = RCFilter_Update(&lpfGyr[2], rawIMU.gyr_z);

		//IMU Data
		//printIMUData(&rawIMU);
		//printIMUData(&filteredIMU);

		//Filtro Complementario
		cfLastData = cfCalculate(filteredIMU);
		cont++;

		//Cada 10 min guardo un valor
		if ((cont % ((60000/IMU_SAMPLE_TIME_MS)*TIEMPO_ENTRE_MUESTRAS_MIN)) == 0)
		{
			//Cada X min guardo un valor
			cfVectorData[aux].phiHat_deg = cfLastData.phiHat_deg;
			cfVectorData[aux].thetaHat_deg = cfLastData.thetaHat_deg;
			cfVectorData[aux].yaw_deg = cfLastData.yaw_deg;

			aux++;

			if (aux >= CANTIDAD_MUESTRAS)
			{
				//Llene el buffer y lo envio
				Serial.println(" ********** Vector de muestras ********** ");
				Serial.println("pitch;roll;yaw");

				for(aux = 0 ; aux < CANTIDAD_MUESTRAS ; aux++)
				{
					plotAttitude(&cfVectorData[aux]);
				}

				//Termino el programa
				while(1)
				{
					delay(10000);
				}
			}
		}
		
		//printAllData(&rawIMU, &filteredIMU, &cfLastData);
		//plotAttitude(&rawIMU, &filteredIMU, &lastAttitude);
		//plotAttitude(&cfLastData);

		imuReadTimer = timerLoad(IMU_SAMPLE_TIME_MS);
	}
	
	//Uso de BT como consola serie
	if( timerIsCompleted(btWriteimer) )
	{
		if (Serial.available()) {
			SerialBT.write(Serial.read());
		}
		if (SerialBT.available()) {
			Serial.write(SerialBT.read());
		}

		plotAttitudeBT(&cfLastData);

		btWriteimer = timerLoad(IMU_SAMPLE_TIME_MS);
	}
}


