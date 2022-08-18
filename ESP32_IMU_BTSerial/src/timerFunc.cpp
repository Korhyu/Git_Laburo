#include <Arduino.h>
#include "timerFunc.h"

int timerLoad (int millisCount)
{
	/* Devuelve el valor de millis al finalizar la cuenta pasada
	como parametro */

	int count = millis();
	count += millisCount;

	return count;
}



bool timerIsCompleted (int timerReference)
{
	/* Revisa si el contador pasado por parametro cumplio su tiempo
	y devuelve TRUE o FALSE dependiendo si cumplio o no */

	int actualMils = millis();

	if ( timerReference <= actualMils )
	{
		//El tiempo se cumplio
		return true;
	}
	else
	{
		return false;
	}
}