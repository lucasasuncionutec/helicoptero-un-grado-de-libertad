#ifndef CODIGO_MPU_H
#define CODIGO_MPU_H

#include <Wire.h>
#include <MPU6050.h>
#include <Arduino.h> // Incluye Arduino para tipos como `unsigned long`



// === Funciones ===
void inicializarSensor();
void configurarOffset(float nuevoOffset);
float leerAnguloBruto();
float leerSensorKalman();
float leerSensor();
#endif
