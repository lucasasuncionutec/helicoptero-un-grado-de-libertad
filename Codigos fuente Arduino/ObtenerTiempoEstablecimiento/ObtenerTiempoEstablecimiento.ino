#include "codigoMPU.h"
#include "codigoMotor.h"

// === CONFIGURACIÓN GENERAL OPTIMIZADA ===
const int frecuenciaMuestreo = 20;             // Hz
const int periodoMuestreo = 1000 / frecuenciaMuestreo; // ms entre muestras
const int duracionTotal = 20;                  // segundos
const int cantidadMuestras = frecuenciaMuestreo * duracionTotal;  // 200
const int muestrasPrePWM = frecuenciaMuestreo * 5;                 // 100

int16_t tiempos[cantidadMuestras];        // Tiempo en ms
int16_t angulos[cantidadMuestras];        // Ángulo * 100

int16_t tiempos_pre[muestrasPrePWM];
int16_t angulos_pre[muestrasPrePWM];

int indiceMuestra = 0;
bool midiendo = false;
bool tomandoDatos = false;
unsigned long tiempoInicio = 0;
unsigned long tiempoUltimaMuestra = 0;
int pwmAplicado = 0;

void setup() {
  Serial.begin(9600);
  inicializarMotor();
  inicializarSensor();
  configurarOffset(-4.38); // Calibración personalizada

  escribirVelocidadEnESC(1000); // Motor en reposo
  Serial.println("Ingrese un valor de PWM entre 1000 y 2000:");
}

void loop() {
  loopCalibracion();  // Seguridad


  if (!tomandoDatos && Serial.available() > 0) {
    int velocidad = Serial.parseInt();
    if (velocidad >= 1000 && velocidad <= 2000) {
      pwmAplicado = velocidad;
      Serial.print("PWM recibido: ");
      Serial.println(pwmAplicado);
      Serial.println("Tomando datos del sensor durante 5 segundos en reposo...");

      for (int i = 0; i < muestrasPrePWM; i++) {
        float angulo = leerSensor();
        angulos_pre[i] = angulo * 100;
        tiempos_pre[i] = i * periodoMuestreo;
        delay(periodoMuestreo);
      }

      Serial.println("¡Listo! Aplicando PWM y comenzando medición principal...");

      escribirVelocidadEnESC(pwmAplicado);

      tiempoInicio = millis();
      tiempoUltimaMuestra = millis();
      indiceMuestra = 0;
      midiendo = true;
      tomandoDatos = true;
    } else {
      Serial.println("Valor inválido. Ingrese entre 1000 y 2000.");
    }
  }

  // === FASE DE MEDICIÓN PRINCIPAL ===
  if (midiendo && millis() - tiempoUltimaMuestra >= periodoMuestreo) {
    tiempoUltimaMuestra = millis();

    float angulo = leerSensor();
    angulos[indiceMuestra] = angulo * 100;
    tiempos[indiceMuestra] = millis() - tiempoInicio;
    indiceMuestra++;

    if (indiceMuestra >= cantidadMuestras) {
      escribirVelocidadEnESC(1000);  // apagar motor
      midiendo = false;

      Serial.println("=== TIEMPOS (s) ===");
      for (int i = 0; i < muestrasPrePWM; i++) {
        Serial.print(tiempos_pre[i] / 1000.0, 3);
        if (i < muestrasPrePWM - 1) Serial.print(", ");
      }
      for (int i = 0; i < cantidadMuestras; i++) {
        Serial.print(", ");
        Serial.print((tiempos[i] + 5000) / 1000.0, 3);  // +5s compensación
      }
      Serial.println();

      Serial.println("=== ANGULOS (°) ===");
      for (int i = 0; i < muestrasPrePWM; i++) {
        Serial.print(angulos_pre[i] / 100.0, 2);
        if (i < muestrasPrePWM - 1) Serial.print(", ");
      }
      for (int i = 0; i < cantidadMuestras; i++) {
        Serial.print(", ");
        Serial.print(angulos[i] / 100.0, 2);
      }
      Serial.println();

      Serial.println("Medición completa. Ingrese otro PWM para repetir.");
      tomandoDatos = false;
    }
  }
}
