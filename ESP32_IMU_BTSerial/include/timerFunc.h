#ifndef TIMERSFUNC_H
#define TIMERSFUNC_H



/* Devuelve el valor de millis al finalizar la cuenta pasada como parametro */
int timerLoad (int millisCount);

/* Revisa si el contador pasado por parametro cumplio su tiempo y devuelve TRUE
o FALSE dependiendo si cumplio o no */
bool timerIsCompleted (int timerReference);




#endif