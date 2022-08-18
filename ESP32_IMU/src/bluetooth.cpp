#include "bluetoothFunc.h"

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

int btConfig ()
{
	#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
	#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
	#endif

	SerialBT.begin("ESP32test"); //Bluetooth device name

}

int btRWdata()
{
	if (Serial.available())
	{
		SerialBT.write(Serial.read());
	}

	if (SerialBT.available())
	{
		Serial.write(SerialBT.read());
	}
}