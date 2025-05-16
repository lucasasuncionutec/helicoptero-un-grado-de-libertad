#ifndef REGISTRO_DATOS_H
#define REGISTRO_DATOS_H

// === Estructura de datos para cada muestra ===
struct Registro {
  float tiempo;
  float angulo;
  float pwm;
  float error;
};

// === Inicialización ===
void inicializarRegistro(float tiempoMuestreo, int duracionSegundos);
void iniciarTemporizador();  // Inicia tiempo en cero luego de calibrar

// === Control de tiempo de muestreo ===
bool esMomentoDeMuestrear();
float obtenerTiempoActual();

// === Registro de datos ===
void guardarRegistro(float tiempo, float angulo, float pwm, float error);
bool registroCompleto();
void imprimirDatos();
bool verificarMuestreo();

#endif
