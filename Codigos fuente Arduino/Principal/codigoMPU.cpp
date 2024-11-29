#include "codigoMPU.h"

// Instancia del sensor MPU6050
MPU6050 sensor;

// Variables internas
static float offsetCalibracion = 0.0;

void inicializarSensor() {
    sensor.initialize();
    if (!sensor.testConnection()) {
        Serial.println("Error al conectar el sensor");
        while (true);
    }
}

void configurarOffset(float nuevoOffset) {
    offsetCalibracion = nuevoOffset;
}

float leerSensor() {
    int16_t axRaw, ayRaw, azRaw;
    sensor.getAcceleration(&axRaw, &ayRaw, &azRaw);

    float ax = axRaw / 16384.0;
    float ay = ayRaw / 16384.0;
    float az = azRaw / 16384.0;

    return atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14); - offsetCalibracion;
}
