#ifndef IMU_H
#define IMU_H

// Mapa de registros
#define TEMP_DATA0		0x1E	//30 decimal
#define ACCEL_DATA_X1	0x1F	//31 decimal
#define ACCEL_DATA_X0	0x20	//32 decimal
#define ACCEL_DATA_Y1	0x20	//33 decimal
#define ACCEL_DATA_Y0	0x22	//34 decimal
#define ACCEL_DATA_Z1	0x23	//35 decimal
#define ACCEL_DATA_Z0	0x24	//36 decimal
#define GYRO_DATA_X1	0x25	//37 decimal
#define GYRO_DATA_X0	0x26	//38 decimal
#define GYRO_DATA_Y1	0x27	//39 decimal
#define GYRO_DATA_Y0	0x28	//40 decimal
#define GYRO_DATA_Z1	0x29	//41 decimal
#define GYRO_DATA_Z0	0x2A	//42 decimal

/* Matrices de calibracion del acelerometro */
static float accCalBias[3] = 
	{
		-0.4561606,
		 0.7066002,
		 3.2257534
	};

static float accCalFactor[3] = 
	{
		1.00392437,
		1.02503296,
		1.00309659
	};

struct imuData{
    float acc_x;            //Acc X axis
    float acc_y;            //Acc Y axis
    float acc_z;            //Acc Z axis
    float acc_rms;          //
    float gyr_x;            //Gyr X axis
    float gyr_y;            //Gyr Y axis
    float gyr_z;            //Gyr Z axis
    float temp;             //Temp in chip
};

struct attitudeData{
	float pitch = 0;			//Rotacion Ascenso/Descenso
	float roll = 0;				//Rotacion "ladel"
	float yaw = 0;				//Rotacion Z - Orientacion
	float vertical = 0;			//Velocidad vertical
	float longitudinal = 0;		//Velocidad "adelante"
	float lateral = 0;			//Velocidad lateral
};

/*
void printIMUData (imuData * datos);
void printCFData (cfData * datos);
void printAllData (imuData * imu_raw, imuData * imu_fil, cfData * cf);
void plotAttitude (imuData * imu_raw, imuData * imu_fil, attitudeData * attitude);
void plotAttitude (cfData * cfDatos);
*/

#endif