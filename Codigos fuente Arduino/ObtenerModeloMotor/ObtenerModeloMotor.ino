#include "codigoMotor.h"

void setup() {
  Serial.begin(9600);
  inicializarMotor();
}

void loop() {
  loopCalibracion();
  if (Serial.available() > 0) {
    int velocidad = Serial.parseInt();  // Leer el valor en microsegundos desde la consola serial
    Serial.print(velocidad);
    if (velocidad >= 1000 && velocidad <= 2000) {
      escribirVelocidadEnESC(velocidad);
      Serial.print("Señal ajustada a: ");
      Serial.print(velocidad);
      Serial.println(" microsegundos");

    } else {
      Serial.println("Por favor ingrese un valor entre 1000 y 2000 microsegundos.");
    }
  }
}
