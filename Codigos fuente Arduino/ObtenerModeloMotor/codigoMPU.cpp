#include "codigoMPU.h"

// Instancia del sensor
MPU6050 sensor;

// Variables internas
float offsetCalibracion = 0.0;

// Kalman
float anguloKalman = 0.0;
float p = 0.5;
float q = 0.01;
float r = 1.5;
float k = 0.0;

// LPF sobre Kalman
float anguloKalmanSuavizado = 0.0;
float alphaSuavizado = 0.1;

void inicializarSensor() {
    Wire.begin();
    Serial.println("Inicializando sensor MPU6050...");
    sensor.initialize();
    delay(1000);

    int intentos = 0;
    const int maxIntentos = 10;
    while (!sensor.testConnection() && intentos < maxIntentos) {
        Serial.println("Error al conectar el sensor. Reintentando...");
        sensor.initialize();
        delay(500);
        intentos++;
    }

    if (sensor.testConnection()) {
        Serial.println("Sensor iniciado correctamente.");
    } else {
        Serial.println("No se pudo conectar con el MPU6050.");
    }
}

void configurarOffset(float nuevoOffset) {
    offsetCalibracion = nuevoOffset;
}

float leerAnguloBruto() {
    int16_t axRaw, ayRaw, azRaw;
    sensor.getAcceleration(&axRaw, &ayRaw, &azRaw);

    float ax = axRaw / 16384.0;
    float ay = ayRaw / 16384.0;
    float az = azRaw / 16384.0;

    return atan(-ax / sqrt(ay * ay + az * az)) * (180.0 / 3.14159265359) - offsetCalibracion;
}

float leerSensorKalman() {
    float medida = leerAnguloBruto();
    p = p + q;
    k = p / (p + r);
    anguloKalman = anguloKalman + k * (medida - anguloKalman);
    p = (1 - k) * p;
    return anguloKalman;
}

// Aplica Filtro Kalman y lo suaviza con filtro pasabajos
float leerSensor() {
    float kalman = leerSensorKalman();
    anguloKalmanSuavizado = alphaSuavizado * kalman + (1 - alphaSuavizado) * anguloKalmanSuavizado;
    return anguloKalmanSuavizado;
}
