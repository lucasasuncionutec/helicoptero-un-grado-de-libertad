#include "CodigoEncoder.h"
#include <Arduino.h>

// === Variables internas ===
volatile int encoderPos = 0;  // Contador de pulsos (modificado por interrupciones)
float offsetCal = 0.0;        // Offset para calibrar el ángulo leído

// Conversión de pulsos a grados (ej. si hay 600 pulsos/rev y modo cuadratura x4 => 2400 pulsos/rev)
const float factorConversion = 360.0 / (PULSOS_POR_REV * MODO_CUADRATURA);

// === Función de interrupción ===
// Se llama automáticamente en cada cambio de flanco del canal A
void updateEncoder() {
  int senal_a = digitalRead(A_PIN);
  int senal_b = digitalRead(B_PIN);

  // Determina el sentido de giro
  if (senal_a == senal_b) {
    encoderPos++;  // Sentido horario
  } else {
    encoderPos--;  // Sentido antihorario
  }
}

// === Inicialización de pines e interrupción ===
void inicializarEncoder() {
  pinMode(A_PIN, INPUT);
  pinMode(B_PIN, INPUT);

  // Asocia la función ISR al pin A (flancos de subida y bajada)
  attachInterrupt(digitalPinToInterrupt(A_PIN), updateEncoder, CHANGE);
}

// === Calibración del ángulo ===
// Esta función ajusta el ángulo leído para que coincida con un valor conocido
void calibrarAnguloInicial(float angulo_actual_en_grados) {
  noInterrupts(); // Evita que encoderPos cambie durante la operación
  offsetCal = angulo_actual_en_grados - encoderPos * factorConversion;
  interrupts();
}

// === Lectura del ángulo actual (en grados) ===
// Aplica la conversión y el offset para obtener el ángulo real
float leerAngulo() {
  noInterrupts();           // Protege acceso a encoderPos
  int pulsos = encoderPos;
  interrupts();
  
  // Retorna el ángulo. El signo negativo puede ser cambiado según necesidad.
  return -(pulsos * factorConversion + offsetCal);
}
