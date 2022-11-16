#ifndef FILTERS_H
#define FILTERS_H


#define	RC_FCUOFF_ACC	15.0f
#define	RC_FCUOFF_GYR	25.0f


typedef struct {

	float coeff[2];
	float out[2];

} RCFilter;

//Filtro RC pasabajos
void RCFilter_Init(RCFilter *filt, float cutoffFreqHz, float sampleTimeS);

//Carga un nuevo dato en el filtro y calcula la salida
float RCFilter_Update(RCFilter *filt, float inp);




#endif