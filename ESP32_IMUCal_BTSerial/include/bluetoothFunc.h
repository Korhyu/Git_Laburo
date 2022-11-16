#ifndef BTFUNC_H
#define BTFUNC_H


#define BT_WRITE_TIME_MS 200  //Delay entre muestras del IMU

#include "string.h"

struct btCommand
{
	int	header;			//Header del comando - describe sobre que elemento va a realizarse la accion
	int body;			//Body del comando - describe la accion a efectuar sobre el elemento
	int parameter;		//Parametro del comando - indica el parametro en el cual debe quedar luego del comando - OPCIONAL
};


int btConfig();
int btRWdata();

void btPrintCommand (struct btCommand command);
//struct btCommand btCommandAnalize(String sc);				//Analiza lo recivido por el BT y realiza las acciones correspondientes

#endif