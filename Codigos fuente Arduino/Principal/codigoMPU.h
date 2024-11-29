#ifndef CODIGO_MPU_H
#define CODIGO_MPU_H

#include <Wire.h>
#include <MPU6050.h>
#include <Arduino.h> // Incluye Arduino para tipos como `unsigned long`

// Estructura para almacenar los datos de medición
struct Medida {
    unsigned long tiempo;
    float angulo;
};

// Constante para el número máximo de mediciones
const int max_medidas = 200;

// Declaración de funciones
void inicializarSensor();
void configurarOffset(float nuevoOffset);
float leerSensor();

#endif
