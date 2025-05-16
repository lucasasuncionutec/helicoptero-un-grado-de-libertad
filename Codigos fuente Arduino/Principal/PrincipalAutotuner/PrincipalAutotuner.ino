// Librerías necesarias para control del sistema y adquisición de datos
#include <Arduino.h>
#include "codigoMotor.h"
#include "codigoEncoder.h"
#include "registroDatos.h"

// Parámetros físicos del modelo utilizados para cálculos dinámicos
const float angulo_equilibrio = 0.0 * (PI / 180);  // Ángulo de operación para linealización (rad)
const float Lm = 0.3310;                          // Longitud desde el pivote hasta el motor (m)
const float C  = -0.1326;                         // Constante de torque del sistema (N·m)
const float I  = 0.0167;                          // Momento de inercia (kg·m²)
const float m  = 0.001586;                        // Pendiente del modelo empírico fuerza vs PWM (N/μs)
const float r  = -1.692631;                       // Intersección del modelo empírico (N)

// Variables para almacenar los valores de equilibrio del sistema
float fuerza_equilibrio = 0;                      // Fuerza necesaria para mantener el equilibrio (N)
float PWM_equilibrio = 0;                         // PWM requerido para generar esa fuerza (μs)

// Ganancias del controlador PID con filtro derivativo
const float Kp = 134.4;
const float Ki = 29.64;
const float Kd = 145.5;
const float N  = 9.704;                           // Frecuencia de corte del filtro derivativo (Hz)

// Definición de tiempos importantes del sistema
const float T = 22;                               // Tiempo de muestreo del controlador (ms)
const float T_procesamiento_total = 1;            // Tiempo total estimado de procesamiento interno (ms)

// Cálculo de constantes intermedias basadas en el controlador PIDf
float a = Kp + Kd * N;
float b = Kp * N + Ki;
float c = Ki * N;
float d = N;

// Reformulación para ecuación en diferencias
float K1 = (b * d - c) / (d * d);
float K2 = c / d;
float K3 = (a * d * d - b * d + c) / (d * d);

// Coeficientes de la ecuación en diferencias del PID discretizado
float a0 = K1 + K3;
float a1 = -K1 - K1 * exp(-d * T) + K2 * T - 2 * K3;
float a2 = K1 * exp(-d * T) - K2 * T * exp(-d * T) + K3;
float a3 = 1.0;
float a4 = -exp(-d * T) - 1.0;
float a5 = exp(-d * T);

// Ángulo deseado en radianes (referencia del sistema)
float referencia = 10.0 * (PI / 180);

// Variables de memoria para almacenar estados anteriores del controlador
float error_km1 = 0.0, error_km2 = 0.0;
float u_km1 = 0.0, u_km2 = 0.0;

// Pin utilizado para medición del tiempo de muestreo mediante osciloscopio
int pin_test_tiempo_muestreo = 8;

void setup() {
  Serial.begin(9600);
  inicializarMotor();
  inicializarEncoder();
  calibrarAnguloInicial(50.4);  // Ángulo inicial medido en grados
  pinMode(pin_test_tiempo_muestreo, OUTPUT);

  // Cálculo de la fuerza y PWM de equilibrio en función del modelo
  fuerza_equilibrio = -(C * cos(referencia)) / Lm;
  PWM_equilibrio = (fuerza_equilibrio - r) / m;

  // Configuración para registrar datos experimentales del sistema
  int duracion = 80; // Duración total en segundos
  inicializarRegistro(0.3, duracion);

  Serial.println("Sistema inicializado.");
}

void loop() {
  digitalWrite(pin_test_tiempo_muestreo, HIGH);  // Pulso de medición para osciloscopio

  loopCalibracion();
  static bool yaIniciadoTemporizador = false;
  if (getCalibrado() && !yaIniciadoTemporizador) {
    iniciarTemporizador();
    yaIniciadoTemporizador = true;
  }

  // Conversión de lectura angular de grados a radianes
  float angulo_actual_grados = leerAngulo();
  float angulo_actual = angulo_actual_grados * (PI / 180.0);

  // Cálculo del error entre referencia y salida medida
  float error = referencia - angulo_actual;

  // Aplicación de la fórmula discreta del PIDf
  float u = (1.0 / a3) * (a0 * error + a1 * error_km1 + a2 * error_km2 
                         - a4 * u_km1 - a5 * u_km2);

  // PWM corregido por equilibrio, en escala absoluta
  float PWM_aplicar = u + PWM_equilibrio;

  // Lógica de saturación del actuador (rango válido del ESC)
  if (PWM_aplicar > 2000.0f) PWM_aplicar = 2000.0f;
  if (PWM_aplicar < 1000.0f) PWM_aplicar = 1000.0f;

  // Aplicación del PWM al ESC
  escribirVelocidadEnESC((int)PWM_aplicar);

  // Actualización de variables de memoria para la próxima iteración
  error_km2 = error_km1;
  error_km1 = error;
  u_km2 = u_km1;
  u_km1 = u;

  // Guardado de registros de datos en tiempo real
  if (getCalibrado() && !registroCompleto() && verificarMuestreo()) {
    float tiempo_actual = obtenerTiempoActual();
    guardarRegistro(tiempo_actual, angulo_actual_grados, PWM_aplicar, error);
  }

  // Finalización del experimento una vez completo el registro
  if (registroCompleto()) {
    imprimirDatos();
    while (true);  // Detener ejecución
  }

  digitalWrite(pin_test_tiempo_muestreo, LOW);
  delay((T - T_procesamiento_total) / 100);  // Pausa para mantener el periodo de muestreo deseado
}
