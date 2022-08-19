#include "complementary_filter.h"
#include "filters.h"
#include "ICM42688.h"
#include "math.h"
#include "Arduino.h"
#include "timerFunc.h"

#ifndef IMU_H
#define IMU_H


//Estados de la maquina de estados del IMU
#define UNCALIBRATED	0		//Estado IDLE no hace nada
#define CAL_Zp			1		//Calibrando eje +Z
#define CAL_Zn			2		//Calibrando eje -Z
#define CAL_Xp			3		//Calibrando eje +X
#define CAL_Xn			4		//Calibrando eje -X
#define CAL_Yp			5		//Calibrando eje +Y
#define CAL_Yn			6		//Calibrando eje -Y
#define SAMPLES_TAKED	7		//Tome las muestras de todos los ejes
#define ALIGINING		8		//Permito la alineacion del sensor

#define CALIBRATED		9		//IMU calibrado
#define TAKING_SAMPLES	10		//Estoy tomando muestras
#define IN_USE			11		//Esta en uso por el sistema

#define IN_CALIBRATION	12		//IMU Calibrandose

#define IMU_ERROR		-1		//Error en el IMU

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


#define IMU_CAL_SIZE	500		//Cantidad de muestras que se usan para calibrar cada eje
#define IMU_CAL_SAMP_MS	10		//Tiempo entre muestras durante la calibracion
#define IMU_READ_MSJ_DELAY 1000	//Tiempo para leer los mensajes enviados por terminal



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
imuData compensateIMUValues (imuData raw);
imuData filterIMUSample (imuData comp);
cfData cfCalculate (imuData imu);


void printIMUData (imuData * datos);
void printCFData (cfData * datos);
void printAllData (imuData * imu_raw, imuData * imu_fil, cfData * cf);
void plotAttitude (imuData * imu_raw, imuData * imu_fil, attitudeData * attitude);
void plotAttitude (cfData * cfDatos);
void plotAttitudeBT (cfData * cfDatos);


attitudeData simpleIntegral (imuData imu);
attitudeData simpleEulerIntegral (imuData imu);


#endif