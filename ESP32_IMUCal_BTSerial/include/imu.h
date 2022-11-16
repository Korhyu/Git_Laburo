#include "complementary_filter.h"
#include "filters.h"
#include "ICM42688.h"
#include "math.h"
#include <Arduino.h>
#include "timerFunc.h"

#ifndef IMU_H
#define IMU_H


//Estados de la maquina de estados del IMU
#define CAL_Zp			0		//Calibrando eje +Z
#define CAL_Zn			1		//Calibrando eje -Z
#define CAL_Xp			2		//Calibrando eje +X
#define CAL_Xn			3		//Calibrando eje -X
#define CAL_Yp			4		//Calibrando eje +Y
#define CAL_Yn			5		//Calibrando eje -Y
#define SAMPLES_TAKED	7		//Tome las muestras de todos los ejes

#define CALSUB_PRINT_AXIS	0		//Estado de ploteo de eje a calibrar
#define CALSUB_ALIGINING	1		//Permito la alineacion del sensor
#define CALSUB_TAKING_SAMP	3		//Estoy tomando muestras
#define CALSUB_CALCULATE	4		//Calculo los valores

#define UNCALIBRATED		-1		//IMU calibrado
#define CALIBRATED			0		//IMU calibrado

#define IN_USE				1		//Esta en uso por el sistema
#define IN_CALIBRATION		2		//IMU Calibrandose

#define IMU_ERROR			-1		//Error en el IMU

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


#define IMU_CAL_SIZE		200		//Cantidad de muestras que se usan para calibrar cada eje
#define IMU_CAL_SAMP_MS		50		//Tiempo entre muestras durante la calibracion
#define IMU_SAMPLE_TIME_MS	50		//Delay entre muestras del IMU en uso
#define IMU_READ_MSJ_DELAY	1000	//Tiempo para leer los mensajes enviados por terminal

#define GRAV_CTE      			9.807f    //Acceleracion de la gravedad

/********************* ESTRUCTURAS *********************/
struct imuData{
    float acc_x;            //Acc X axis
    float acc_y;            //Acc Y axis
    float acc_z;            //Acc Z axis
    float acc_rms;          //
    float gyr_x;            //Gyr X axis
    float gyr_y;            //Gyr Y axis
    float gyr_z;            //Gyr Z axis
    float temp;             //Temp in chip

	
	inline imuData& operator=(const imuData& newValue)
	{
        acc_x=newValue.acc_x;
        acc_y=newValue.acc_y;
        acc_z=newValue.acc_z;
        acc_rms=newValue.acc_rms;
        gyr_x=newValue.gyr_x;
        gyr_y=newValue.gyr_y;
        gyr_z=newValue.gyr_z;
        temp=newValue.temp;

        return *this;
    }
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
accCalBiasNew[0] = 5.583013, accCalBiasNew[1] = -2.435291, accCalBiasNew[2] = 1.993825
accCalFactorNew[0] = 0.597278, accCalFactorNew[1] = 2.119864, accCalFactorNew[2] = 0.726255
*/

/* Matrices de calibracion del acelerometro */
static float accCalBias[3] = 
	{
		 5.498659,
		-2.383453,	
		 1.892426
	};


/*
static float accCalBias[3] = 
	{
		-0.4561606,
		 0.7066002,	
		 3.2257534
	};
*/

static float accCalFactor[3] = 
	{
		0.606738,
		2.038399,
		0.745166
	};

/*
static float accCalFactor[3] = 
	{
		1.00392437,
		1.02503296,
		1.00309659
	};
*/

static float gyrCalBias[3] = 
	{
		-0.0,
		 0.0,
		 0.0
	};

static float gyrCalFactor[3] = 
	{
		1.0,
		1.0,
		1.0
	};




imuData getIMUSample (void);
imuData compensateIMUValues (imuData * raw);
imuData filterIMUSample (imuData * comp);
cfData cfCalculate (imuData * imu);

int IMUmachineState (void);

void printIMUData (imuData * datos);
void printCFData (cfData * datos);
void printAllData (imuData * imu_raw, imuData * imu_fil, cfData * cf);
void plotAttitude (imuData * imu_raw, imuData * imu_fil, attitudeData * attitude);
void plotAttitude (cfData * cfDatos);
void plotAttitudeBT (cfData * cfDatos);

String imuData2String (imuData * datos);			//TODO Funcion que toma los datos del IMU y los convierte a un String para imprimirlos


attitudeData simpleIntegral (imuData imu);
attitudeData simpleEulerIntegral (imuData imu);


#endif