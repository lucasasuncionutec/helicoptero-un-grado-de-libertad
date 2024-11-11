#include <Servo.h>

Servo esc;

int velocidad_actual = 1000;

void setup() {
  Serial.begin(9600);
  esc.attach(10);
  esc.writeMicroseconds(0);
  
  Serial.println("Ingrese un valor de microsegundos entre 1000 y 2000:");
}

void loop() {
  if (Serial.available() > 0) {
    int velocidad = Serial.parseInt();  // Leer el valor en microsegundos desde la consola serial
    Serial.print(velocidad);
    if (velocidad >= 1000 && velocidad <= 2000) {
      escribirVelocidadEnESC(velocidad);
      Serial.print("Señal ajustada a: ");
      Serial.print(velocidad_actual);
      Serial.println(" microsegundos");

    } else {
      Serial.println("Por favor ingrese un valor entre 1000 y 2000 microsegundos.");
    }
  }
}

void escribirVelocidadEnESC(int velocidad) {
  int diferencia = velocidad - velocidad_actual;
  // Crea una rampa ascendente o descendente hasta alcanzar la velocidad deseada
  for (int i = 0; i < abs(diferencia); i++) {
    if (diferencia > 0) {
      velocidad_actual ++;
    } else if (diferencia < 0) {
      velocidad_actual --;
    }
    Serial.println(velocidad_actual);
    esc.writeMicroseconds(velocidad_actual);
    //delay(0);
  }

}
