#include "registroDatos.h" 
#include "codigoMotor.h" // para usar getCalibrado()
#include <Arduino.h>

static Registro* registros = nullptr;
static int cantidad_muestras = 0;
static int indice = 0;
static bool completo = false;

static float T_muestreo = 0.01;
static unsigned long tiempo_inicio = 0;
static unsigned long tiempo_prev_muestreo = 0;
static bool temporizador_activo = false;
static bool temporizador_iniciado = false;

void inicializarRegistro(float tiempoMuestreo, int duracionSegundos) {
  cantidad_muestras = (int)(duracionSegundos / tiempoMuestreo);
  if (registros != nullptr) delete[] registros;
  registros = new Registro[cantidad_muestras];
  indice = 0;
  completo = false;

  T_muestreo = tiempoMuestreo;
  temporizador_activo = false;
  temporizador_iniciado = false;
}

void iniciarTemporizador() {
  tiempo_inicio = millis();
  tiempo_prev_muestreo = tiempo_inicio;
  temporizador_activo = true;
  temporizador_iniciado = true;
}

float obtenerTiempoActual() {
  return (millis() - tiempo_inicio) / 1000.0;
}

// función que controla y actualiza el muestreo
bool verificarMuestreo() {
  if (!temporizador_activo || completo) return false;

  unsigned long ahora = millis();
  if ((ahora - tiempo_prev_muestreo) >= T_muestreo * 1000.0) {
    tiempo_prev_muestreo += T_muestreo * 1000.0;
    return true;
  }
  return false;
}

void guardarRegistro(float tiempo, float angulo, float pwm, float error) {
  if (completo || registros == nullptr) return;

  registros[indice] = {tiempo, angulo, pwm, error};
  indice++;

  if (indice >= cantidad_muestras) {
    completo = true;
  }
}

bool registroCompleto() {
  return completo;
}

void imprimirDatos() {
  if (!completo || registros == nullptr) return;

  // Imprimir encabezados para MATLAB
  Serial.println("%% Copiar directamente en MATLAB");
  Serial.print("tiempo = [");
  for (int i = 0; i < cantidad_muestras; i++) {
    Serial.print(registros[i].tiempo, 3);
    if (i < cantidad_muestras - 1) Serial.print(", ");
  }
  Serial.println("];");

  Serial.print("angulo = [");
  for (int i = 0; i < cantidad_muestras; i++) {
    Serial.print(registros[i].angulo, 2);
    if (i < cantidad_muestras - 1) Serial.print(", ");
  }
  Serial.println("];");

  Serial.print("pwm = [");
  for (int i = 0; i < cantidad_muestras; i++) {
    Serial.print(registros[i].pwm, 1);
    if (i < cantidad_muestras - 1) Serial.print(", ");
  }
  Serial.println("];");

  Serial.print("error = [");
  for (int i = 0; i < cantidad_muestras; i++) {
    Serial.print(registros[i].error, 2);  // ← redondeo a 2 decimales
    if (i < cantidad_muestras - 1) Serial.print(", ");
  }
  Serial.println("];");
  Serial.println("%% Fin datos");


  // Liberar memoria
  delete[] registros;
  registros = nullptr;
}

