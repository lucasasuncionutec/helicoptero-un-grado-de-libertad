#ifndef CODIGO_MOTOR_H
#define CODIGO_MOTOR_H

#include <Arduino.h>
#include <Servo.h>

#define RELAY_PIN 4
#define SWITCH_PIN 2
#define ESC_PIN 10

void inicializarMotor();
void calibrarESC();
bool escribirVelocidadEnESC(int velocidad);
void loopCalibracion();

#endif
