#ifndef CODIGO_COMUNICACION_SOFTWARE_H
#define CODIGO_COMUNICACION_SOFTWARE_H

#include <Arduino.h>

void recibirDatosSerial(float* vector, int maxValores, int* cantidadLeida);
void enviarDatosSerial(float* vector, int cantidad);

// Funciones de alto nivel para el loop
bool leerDatosDesdePC(float& Tss_ref, float& Mp_ref,
                      float& kp_pc, float& ki_pc, float& kd_pc, float& n_pc,
                      float& pwm_pc, bool& toggle_pc);
void enviarDatosALaPC(float angulo_deg, float error_deg, float pwm);

#endif
