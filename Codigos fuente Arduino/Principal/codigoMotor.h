#ifndef CODIGO_MOTOR_H
#define CODIGO_MOTOR_H

#include <Arduino.h>
#define SERVO_TIMER 5
#include <Servo.h>

#define RELAY_PIN 10
#define SWITCH_PIN 4
#define ESC_PIN 11

void inicializarMotor();
void calibrarESC();
bool escribirVelocidadEnESC(int velocidad);
void loopCalibracion();
bool getCalibrado();

#endif
