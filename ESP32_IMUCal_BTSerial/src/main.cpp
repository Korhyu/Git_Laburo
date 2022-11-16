#include <Arduino.h>
#include "BluetoothSerial.h"

#include "string.h"
#include "math.h"
#include "imu.h"
#include "complementary_filter.h"
#include "filters.h"
#include "timerFunc.h"
#include "bluetoothFunc.h"
#include "ST25DVSensor.h"
#include "ledDriver.h"
#include "UARTDriver.h"

#define TIEMPO_ENTRE_MUESTRAS_MIN	1
#define AVG_BUFFER_DEEP				(TIEMPO_ENTRE_MUESTRAS_MIN*60*1000)/IMU_SAMPLE_TIME_MS

#define EMPTY_BUFFER				"NULL"
#define PLOT_TIME_MS				250

#define OFF		0
#define ON		1


#define GPO_PIN 19
#define LPD_PIN 23
#define SDA_PIN 21
#define SCL_PIN 22

const char uri_write_message[] = "GA.MA - UALA";				// Uri message to write in the tag
const char uri_write_protocol[] = URI_ID_0x01_STRING;			// Uri protocol to write in the tag
String uri_write = String(uri_write_protocol) + String(uri_write_message);


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// an ICM42688 object with the ICM42688 sensor on I2C bus 0 with address 0x68
ICM42688 IMU(Wire,0x68);

//ST25DV st25dv;

int status;


int cont=0;
int aux=0;
int buffer_aux=0;

int imuReadTimer = 0;
int btWriteTimer = 0;
int nfcTimer = 0;
int ledTimer = 0;
int uartTimer = 0;
int pruebaTimer = 10000; 

int btStatus = OFF;
int nfcStatus = OFF;
int imuStatus = OFF;

int timeCount = 0;


extern cfData cfLastData;
attitudeData lastAttitude;

bool samplingIMU = false;
bool storeIMUData = false;

String bufferOutBT;			//Buffer de salida a BT
String bufferInBT;			//Buffer de entrada desde BT

struct btCommand btCommandAnalize(String sc);


extern RCFilter lpfAcc[4];
extern RCFilter lpfGyr[4];

int btConfig ()
{
	SerialBT.begin("SmarComm_DevKit2"); //Bluetooth device name
	Serial.println("BT OK");

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
	SerialBT.println(cfDatos->thetaHat_deg,2);
	SerialBT.print(";");
	SerialBT.println(cfDatos->yaw_deg,2);
}


void setup() {
	// serial to display data
	Serial.begin(115200);
	while(!Serial) {}

	//ledConfig();

	// configure LED PWM functionalitites
	ledcSetup(LEDRChannel, PWM_FREQ, PWM_RES);
	ledcSetup(LEDGChannel, PWM_FREQ, PWM_RES);
	ledcSetup(LEDBChannel, PWM_FREQ, PWM_RES);

	// attach the channel to the GPIO to be controlled
	ledcAttachPin(LEDR, LEDRChannel);
	ledcAttachPin(LEDG, LEDGChannel);
	ledcAttachPin(LEDB, LEDBChannel);

	ledOFF(LEDR);
	ledOFF(LEDG);
	ledOFF(LEDB);
	
	
	// start communication with IMU
	status = IMU.begin();
	if (status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("Status: ");
		Serial.println(status);
		while(1) {}
	}
	else
	{
		Serial.println("IMU OK");
		imuStatus = ON;
		ledON(LEDR);
	}
	

	//Configuracion de filtros
	for (uint8_t n = 0; n < 4; n++) {
		RCFilter_Init(&lpfAcc[n], RC_FCUOFF_ACC, (float(IMU_SAMPLE_TIME_MS)/1000) );
		RCFilter_Init(&lpfGyr[n], RC_FCUOFF_GYR, (float(IMU_SAMPLE_TIME_MS)/1000) );
	}
	//RCFilter_Config(IMU_SAMPLE_TIME_MS);

	//BT config
	btConfig();
	//ledON(LEDB);

	//NFC Config
	if( st25dv.begin(GPO_PIN, LPD_PIN, &Wire) == 0 )
	{
		Serial.println("NFC OK");
		btStatus = ON;
	}
	else
	{
		Serial.println("System Init failed!");
		while(1);
	}

	if(st25dv.writeURI(uri_write_protocol, uri_write_message, ""))
	{
		Serial.println("Write failed!");
		while(1);
	}
	else
	{
		nfcStatus = ON;
		//ledON(LEDG);
	}

	//Hago la primer carga de timers
	imuReadTimer = timerLoad(IMU_SAMPLE_TIME_MS);
	btWriteTimer = timerLoad(PLOT_TIME_MS+2);				//Ese offset extra es para que no caigan al mismo momento
	nfcTimer = timerLoad(500);
	ledTimer = timerLoad(LED_REFRESH_TIME+4);			//Ese offset extra es para que no caigan al mismo momento
	uartTimer = timerLoad(UART_REFRESH_TIME+4);			//Ese offset extra es para que no caigan al mismo momento

	ledOFF(LEDR);
	ledOFF(LEDG);
	ledOFF(LEDB);

	//ledSet(LEDG, LED_BLINK, MAX_BRIGTH);
}

void loop()
{
	//Led Control
	if( timerIsCompleted(ledTimer) )
	{
		/*
		static int ledStatus = LED_OFF;
		if ( ((nfcStatus && imuStatus) && btStatus) != OFF )
		{
			if ( ledStatus == LED_OFF )
			{
				ledcWrite(LEDGChannel, MIN_BRIGTH);
				ledStatus = LED_ON;
			}
			else
			{
				ledcWrite(LEDGChannel, MAX_BRIGTH);
				ledStatus = LED_OFF;
			}
			ledTimer = timerLoad(1000);
		}
		else
		{
			if ( ledStatus == LED_OFF )
			{
				ledcWrite(LEDRChannel, MIN_BRIGTH);
				ledStatus = LED_ON;
			}
			else
			{
				ledcWrite(LEDRChannel, MAX_BRIGTH);
				ledStatus = LED_OFF;
			}
			ledTimer = timerLoad(1000);
		}
		*/
		//ledMachineState();
		ledTimer = timerLoad(LED_REFRESH_TIME);
	}
	
	
	//Uso de BT como consola serie
	if( timerIsCompleted(btWriteTimer) )
	{
		//Transmision a BT
		if (bufferOutBT != EMPTY_BUFFER)
		{
			//Hay datos para enviar
			SerialBT.println(bufferOutBT);
			bufferOutBT = EMPTY_BUFFER;
		}

		//Recepcion BT
		if (SerialBT.available()) {
			//Si hay datos vacio el buffer
			while (SerialBT.available())
			{
				bufferInBT += (char)SerialBT.read();
			}
			Serial.print("Mensaje BT: ");
			Serial.println(bufferInBT);

			//btCommandAnalize(bufferInBT);

			bufferInBT = "";
			//TODO: Llamado a funcion que analiza la recepcion
		}

		//plotAttitude(&cfLastData);

		btWriteTimer = timerLoad(PLOT_TIME_MS);
	}

	
	//Lectura del IMU
	if( timerIsCompleted(imuReadTimer) )
	{
		//IMUmachineState();
	}
	

	//Lectura del NFC
	if( timerIsCompleted(nfcTimer) )
	{
		
	}

	//UART Rx y Tx
	if( timerIsCompleted(uartTimer) )
	{
		
	}

	//Prueba
	/*
	if( timerIsCompleted(pruebaTimer) )
	{
		int pruebastate;
		pruebaTimer = timerLoad(5000);
		
		if (pruebastate == 0)
		{
			ledSet(LEDB, LED_BLINK, MAX_BRIGTH);
			ledSet(LEDG, LED_OFF);
			ledSet(LEDR, LED_OFF);

			pruebastate = 1;
		}
		else if (pruebastate == 1)
		{
			ledSet(LEDR, LED_BLINK, MAX_BRIGTH);
			ledSet(LEDG, LED_OFF);
			ledSet(LEDB, LED_OFF);

			pruebastate = 2;
		}
		else if (pruebastate == 2)
		{
			ledSet(LEDG, LED_BLINK, MAX_BRIGTH);
			ledSet(LEDB, LED_OFF);
			ledSet(LEDR, LED_OFF);
			
			pruebastate = 0;
		}
		else
		{
			pruebastate = 0;
		}
	}
	*/
}


struct btCommand btCommandAnalize (String comm)
{
	int i=0, j=0;
	String header = "";
	String body = "";
	String parameter = "";

	struct btCommand comando;

	//El header del comando tiene 2 caracteres
	header += comm[0];
	header += comm[1];

	comando.header = header.toInt();

	//El cuerpo del comando tiene 2 caracteres
	body += comm[2];
	body += comm[3];

	comando.body = body.toInt();

	//El parametro puede tener varios caracteres asique se usa un loop de busqueda
	j = comm.indexOf("\0");
	for( i=4 ; i << j ; i++ )
	{
		parameter += comm[i];
	}

	comando.parameter = parameter.toInt();

	btPrintCommand(comando);

	return comando;
}

void btPrintCommand (struct btCommand command)
{
	Serial.println("Command: ");
	Serial.println(command.header);
	Serial.println(command.body);
	Serial.println(command.parameter);
	Serial.println("\n");
}