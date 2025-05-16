#include <Arduino.h>
#include "codigoMPU.h"

// === Configuración ===
const float T_muestreo = 0.01; // 10 ms
const int duracion_total_segundos = 10; 
const int cantidad_muestras = duracion_total_segundos / T_muestreo;

int16_t datos[cantidad_muestras]; // Ahora usamos enteros de 2 bytes
unsigned long tiempo_anterior = 0;
int indice = 0;

void setup() {
  Serial.begin(9600);
  inicializarSensor();
  configurarOffset(-4.38);

  Serial.println("Sistema inicializado. Comenzando adquisición de datos...");
  tiempo_anterior = millis(); // Guarda el tiempo inicial
}

void loop() {
  if (indice < cantidad_muestras) {
    if (millis() - tiempo_anterior >= T_muestreo * 1000) {
      tiempo_anterior += T_muestreo * 1000; // Actualizar para el próximo muestreo

      // Leer sensor
      float angulo_grados = leerSensor(); 

      // Guardar el ángulo en el array como entero multiplicado por 100
      datos[indice] = (int16_t)(angulo_grados * 100.0);
      indice++;
    }
  } else {
    // Terminó la adquisición, imprimir datos
    Serial.println("=== Datos adquiridos ===");
    for (int i = 0; i < cantidad_muestras; i++) {
      Serial.print(datos[i] / 100.0, 2); // Dividir y mostrar con 2 decimales
      Serial.print(",");
    }
    Serial.println("");
    Serial.println("FIN");

    // Detener el programa después de imprimir
    while (true);
  }
}
