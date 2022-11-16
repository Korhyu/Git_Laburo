#include <Arduino.h>
#include "ledDriver.h"


int ledRstate = LED_OFF;
int ledGstate = LED_OFF;
int ledBstate = LED_OFF;

int ledRbrigth = MAX_BRIGTH;
int ledGbrigth = MAX_BRIGTH;
int ledBbrigth = MAX_BRIGTH;

int ledRcount = -1;
int ledGcount = -1;
int ledBcount = -1;

void ledConfig()
{
	// configure LED PWM functionalitites
	ledcSetup(LEDRChannel, PWM_FREQ, PWM_RES);
	ledcSetup(LEDGChannel, PWM_FREQ, PWM_RES);
	ledcSetup(LEDBChannel, PWM_FREQ, PWM_RES);

	// attach the channel to the GPIO to be controlled
	ledcAttachPin(LEDR, LEDRChannel);
	ledcAttachPin(LEDG, LEDGChannel);
	ledcAttachPin(LEDB, LEDBChannel);

	ledcWrite(LEDRChannel, MIN_BRIGTH);
	ledcWrite(LEDGChannel, MIN_BRIGTH);
	ledcWrite(LEDBChannel, MIN_BRIGTH);
}


void ledMachineState (void)
{
	if ( ledRstate == LED_ON )
		ledON (LEDR);
	else if  ( ledRstate == LED_OFF )
		ledOFF (LEDR);
	else if  ( ledRstate == LED_BLINK )
		ledBlinkingControl(LEDR);
	else
		ledSet(LEDR, LED_DIMMER, ledRbrigth);
		
	if ( ledGstate == LED_ON )
		ledON (LEDG);
	else if  ( ledGstate == LED_OFF )
		ledOFF (LEDG);
	else if  ( ledGstate == LED_BLINK )
		ledBlinkingControl(LEDG);
	else
		ledSet(LEDG, LED_DIMMER, ledGbrigth);

	if ( ledBstate == LED_ON )
		ledON (LEDB);
	else if  ( ledBstate == LED_OFF )
		ledOFF (LEDB);
	else if  ( ledBstate == LED_BLINK )
		ledBlinkingControl(LEDB);
	else
		ledSet(LEDB, LED_DIMMER, ledBbrigth);

	//ledPrintData();
}


void ledSet (int led, int state, int brigth)
{
	//Configura el LED
	if (state == LED_OFF)
		ledcWrite(led, MIN_BRIGTH);
	else if (state == LED_ON)
		ledcWrite(led, MAX_BRIGTH);
	else
	{
		ledcWrite(led, brigth);
	}

	if (led == LEDR)
		ledRstate = state;
	else if (led == LEDG)
		ledGstate = state;
	else if (led == LEDB)
		ledBstate = state;
}

void ledSet (int led, int state)
{
	//Configura el LED
	if (state == LED_OFF)
		ledcWrite(led, MIN_BRIGTH);
	else if (state == LED_ON)
		ledcWrite(led, MAX_BRIGTH);

	if (led == LEDR)
		ledRstate = state;
	else if (led == LEDG)
		ledGstate = state;
	else if (led == LEDB)
		ledBstate = state;
}

void ledON (int led)
{
	led = led2channel(led);
	ledcWrite(led, MAX_BRIGTH);
}

void ledOFF (int led)
{	
	led = led2channel(led);
	ledcWrite(led, MIN_BRIGTH);
}


int led2channel (int led)
{
	switch (led)
	{
		case LEDR:
			return LEDRChannel;

		case LEDG:
			return LEDGChannel;

		case LEDB:
			return LEDBChannel;
		
		default:
			return 0;
	}
}

void ledToggle (int led)
{
	uint32_t ledBrigth = ledcRead(led2channel(led));		// Devuelve el duty

	if( ledBrigth <= MAX_BRIGTH )
		ledOFF(led);
	else
		ledON(led);
}

void ledBlinkingControl (int led)
{
	if ( led == LEDR )
	{
		if ( ledRcount <= 0 )
		{
			ledToggle(led);
			ledRcount = LED_BLINK_COUNT;
		}
		else
			ledRcount--;
	}
	
	if ( led == LEDG )
	{
		if ( ledGcount <= 0 )
		{
			ledToggle(led);
			ledGcount = LED_BLINK_COUNT;
		}
		else
			ledGcount--;
	}

	if ( led == LEDB )
	{
		if ( ledBcount <= 0 )
		{
			ledToggle(led);
			ledBcount = LED_BLINK_COUNT;
		}
		else
			ledBcount--;
	}
}

void ledPrintData (void)
{
	Serial.println("\n");
	Serial.println("LED States:");
	Serial.println(ledRstate);
	Serial.println(ledGstate);
	Serial.println(ledBstate);

	Serial.println("\n");
	Serial.println("LED Brigth:");
	Serial.println(ledRbrigth);
	Serial.println(ledGbrigth);
	Serial.println(ledBbrigth);

	Serial.println("\n");
	Serial.println("LED Count:");
	Serial.println(ledRcount);
	Serial.println(ledGcount);
	Serial.println(ledBcount);
}
