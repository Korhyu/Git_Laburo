#ifndef FILTERS_H
#define FILTERS_H


typedef struct {

	float coeff[2];
	float out[2];

} RCFilter;

//Filtro RC pasabajos
void RCFilter_Init(RCFilter *filt, float cutoffFreqHz, float sampleTimeS);

//Carga un nuevo dato en el filtro y calcula la salida
float RCFilter_Update(RCFilter *filt, float inp);




#endif