#include "I2Cdev.h"
#include "codigoMPU.h"
#include "Wire.h"


void setup() {
    Serial.begin(9600);  // Inicializa el puerto serial
    inicializarSensor();

}

void loop() {
  Serial.println(leerSensor());
  delay(100);
}


