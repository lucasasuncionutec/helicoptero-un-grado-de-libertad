#include <Wire.h>
#include "codigoMPU.h"

const unsigned long intervalo_muestreo = 20; // 50 Hz (20 ms por muestra)
unsigned long tiempo_prev = 0;
unsigned long tiempo_inicio;
unsigned long duracion_simulacion;

int indice_medidas = 0;

// Estructura para almacenar los datos de medición
struct Medida {
    unsigned long tiempo;
    float angulo;
};

//número máximo de mediciones
const int max_medidas = 200;

Medida medidas[max_medidas];

void setup() {
    Serial.begin(9600);
    Wire.begin();

    inicializarSensor();
    configurarOffset(0); 

    Serial.println("Ajuste el ángulo de la varilla al ángulo inicial deseado en los próximos 15 segundos");
    delay(5000);
    unsigned long tiempo_estabilizacion = millis();
    while (millis() - tiempo_estabilizacion < 10000) {
        Serial.print("Ángulo Y: ");
        Serial.println(leerSensor());
        delay(50);
    }

    Serial.println("Ingrese el tiempo de simulación en milisegundos:");
    while (Serial.available() == 0) {}
    duracion_simulacion = Serial.parseInt();

    Serial.println("Simulación iniciada.");
    tiempo_inicio = millis();
}

void loop() {
    unsigned long tiempo_actual = millis();
    if (tiempo_actual - tiempo_prev >= intervalo_muestreo) {
        tiempo_prev = tiempo_actual;

        float anguloY = leerSensor();
        if (indice_medidas < max_medidas) {
            medidas[indice_medidas].tiempo = tiempo_actual - tiempo_inicio;
            medidas[indice_medidas].angulo = anguloY;
            indice_medidas++;
        }
    }

    if (millis() - tiempo_inicio >= duracion_simulacion) {
        Serial.println("Simulación finalizada.");
        enviar_datos_serial();
        while (true);
    }
}

void enviar_datos_serial() {
    Serial.println("Datos obtenidos del experimento");
    Serial.println("Tiempo(ms)");
    for (int i = 0; i < indice_medidas; i++) {
        Serial.println(medidas[i].tiempo);
    }
    Serial.println("Angulo(grados)");
    for (int i = 0; i < indice_medidas; i++) {
        Serial.println(medidas[i].angulo);
    }
}
