#include <Servo.h>

// Definiciones para pines
#define ESC_PIN 11  // Pin para controlar el ESC
#define RELAY_PIN 10 // Pin para el relé que controla la alimentación del ESC

Servo esc;

int velocidad_actual = 1000;

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT); 
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(0);
  Serial.println("Ingrese un valor de microsegundos entre 1000 y 2000:");
  //digitalWrite(RELAY_PIN, HIGH); // Mantiene la alimentacion del ESC encendida
  Serial.println("Calibración iniciada...");
  
  // Apagar el relé
  digitalWrite(RELAY_PIN, LOW);
  escribirVelocidadEnESC(2000); // Señal máxima
  Serial.println("Señal maxima");
  delay(5000);

  // Encender el relé
  digitalWrite(RELAY_PIN, HIGH);
  delay(1000);
  escribirVelocidadEnESC(1000); // Señal mínima
  Serial.println("Señal minima");
  delay(6000);

  Serial.println("Calibración completada.");
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
    esc.writeMicroseconds(velocidad_actual);
  }
}
