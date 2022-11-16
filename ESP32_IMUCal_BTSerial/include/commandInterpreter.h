#ifndef CIN_H
#define CIN_H

#include <string>
#include <Arduino.h>



/*			Lista de parametros			*/
/*
Informacion
IMS-XXXX		Motor speed where XXXX is frecuency of the motor in Hz
IMC-XXX			Motor current here XXX is x100 the current of the motor in A
IHT-XXX			Heater temperature (NTC) where XXX is the temperature in °C
IHP-XXXX		Heater power where XXXX is the power in W
ISV-XXXX		Software version
IAL-XX			Alarm
IUI-XX			User Interface Interaction 00-99 where the diferent commands
IGI-XX			General counter of seconds working

Comandos
CMS-XXXX		Set the motor speed to XXXX Hz
CHP-XXXX		Set the heater power to XXXX W
CIO-X			Set the ION - 0 OFF / 1 ON
CDS-X			Set the speed to the UI 0, 1, 2, 3 or 4
CDP-X			Set the heater to the UI 0, 1, 2 or 3
*/

#define		IMS		0			//Motor Speed
#define		IMC		1			//Motor current
#define		IHT		2			//Heater Temperature
#define		IHP		3			//Heater Power
#define		ISV		4			//Software Version
#define		IAL		5			//Alarm
#define		IUI		6			//User Interface
#define		IGC		7			//General Counter


#define		CMS		20			//Set Motor Speed
#define		CHP		21			//Set Heater Power
#define		CIO		22			//ION ON - OFF
#define		CDS		23			//Set UI Speed
#define		CDP		24			//Set UI Heater


struct command
{
	int header;
	int parameter;
};


const std::string cmdInfo[] = {
	"IMS",		//0
	"IMC",		//1
	"IHT",		//2
	"IHP",		//3
	"ISV",		//4
	"IAL",		//5
	"IUI",		//6
	"IGC"		//7
};

#define	INFO_NUMBER		sizeof(cmdInfo)/sizeof(String)

const std::string cmdCommand[] = {
	"CMS",		//0
	"CHP",		//1
	"CIO",		//2
	"CDS",		//3
	"CDP"		//4
};

#define	COMM_NUMBER		sizeof(cmdCommand)/sizeof(string)

#endif
