#include <Arduino.h>
#include "BluetoothSerial.h"

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


extern cfData cfLastData;
cfData  cfVectorData[CANTIDAD_MUESTRAS];
cfData  cfAVGData[AVG_BUFFER_DEEP];
attitudeData lastAttitude;

bool samplingIMU = false;
bool storeIMUData = false;




int btConfig ()
{
	SerialBT.begin("SmarComm_DevKit"); //Bluetooth device name
	Serial.println("The device started, now you can pair it with bluetooth!");

	return 0;
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


	

	btConfig();

	//Hago la primer carga de timers
	imuReadTimer = timerLoad(IMU_SAMPLE_TIME_MS);
	btWriteimer = timerLoad(IMU_SAMPLE_TIME_MS+10);
}

void loop() {
	//Lectura del IMU
	if( timerIsCompleted(imuReadTimer) )
	{

		imuReadTimer = timerLoad(IMU_SAMPLE_TIME_MS);
	}
	
	//Uso de BT como consola serie
	if( timerIsCompleted(btWriteimer) )
	{
		if (Serial.available()) {
			SerialBT.write(Serial.read());
		}
		if (SerialBT.available()) {
			//Si hay datos vacio el buffer
			while( Serial.available() > 0)
			{
				//Serial.write(SerialBT.read());
				SerialBT.read();
			}
			
		}

		plotAttitudeBT(&cfLastData);

		btWriteimer = timerLoad(IMU_SAMPLE_TIME_MS);
	}
}


