#ifndef CODIGO_ENCODER_H
#define CODIGO_ENCODER_H

// === Definiciones de pines y constantes ===
#define A_PIN 2                 // Pin del canal A del encoder
#define B_PIN 3                 // Pin del canal B del encoder
#define PULSOS_POR_REV 600.0    // Número total de pulsos por revolución del encoder
#define MODO_CUADRATURA 4.0     // Factor según el modo de lectura (A+B y flancos => 4x)

// === Variables y funciones expuestas ===
extern volatile int encoderPos;        // Contador de pulsos del encoder (global)
void updateEncoder();                  // ISR para contar pulsos
void inicializarEncoder();            // Configura pines e interrupciones
void calibrarAnguloInicial(float);    // Ajusta el offset según ángulo conocido
float leerAngulo();                   // Devuelve el ángulo actual (en grados)

#endif
