#ifndef CODIGO_COMUNICACION_SOFTWARE_H
#define CODIGO_COMUNICACION_SOFTWARE_H

#include <Arduino.h>

void recibirDatosSerial(float* vector, int maxValores, int* cantidadLeida);
void enviarDatosSerial(float* vector, int cantidad);

// Funciones de alto nivel para el loop
void leerDatosDesdePC(float& Tss_ref, float& Mp_ref);
void enviarDatosALaPC(float angulo_deg, float error_deg, float pwm);

#endif
