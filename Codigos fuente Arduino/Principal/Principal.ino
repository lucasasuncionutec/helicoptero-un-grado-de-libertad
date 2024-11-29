#include <Arduino.h>
#include "codigoMotor.h"

void setup() {
    Serial.begin(9600);
    
    inicializarMotor(); // Rutina necesaria para poder usar el motor

    Serial.println("Sistema inicializado.");
    Serial.println("Ingrese una velocidad entre 1000 y 2000 microsegundos:");
}

void loop() {
    loopCalibracion();
    
    // Pedir al usuario una velocidad por el puerto serial
    if (Serial.available() > 0) {
        int nuevaVelocidad = Serial.parseInt(); 
        if (nuevaVelocidad >= 1000 && nuevaVelocidad <= 2000) { // La velocidad ingresada debe estar entre el rango de 1000 y 2000
            if(escribirVelocidadEnESC(nuevaVelocidad)){
              Serial.print("Velocidad ajustada a: ");
            }
        } else {
            Serial.println("Por favor, ingrese un valor entre 1000 y 2000.");
        }
    }

    delay(100);
}
