#ifndef LEDD_H
#define LEDD_H

#define PWM_FREQ  500
#define PWM_RES   8

#define LEDRChannel   0
#define LEDGChannel   1
#define LEDBChannel   2

#define LEDR	25
#define LEDG	27
#define LEDB	26

#define	MIN_BRIGTH	255
#define	MAX_BRIGTH	240

#define	LED_OFF		0		//LED OFF
#define	LED_ON		1		//LED ON
#define	LED_BLINK	2		//LED BLINKING
#define	LED_DIMMER	3		//LED in Other brigth level

#define	LED_REFRESH_TIME	100

#define	LED_BLINK_COUNT		(500 / LED_REFRESH_TIME)		//Divido los ms que deseo que este blink por el tiempo del ciclo



void ledMachineState	(void);
void ledPrintData		(void);
void ledConfig			(void);

void ledON				(int led);
void ledOFF				(int led);
void ledToggle			(int led);
void ledBlinkingControl (int led);

void ledSet				(int led, int state, int brigth);
void ledSet				(int led, int state);

int led2channel			(int led);

#endif