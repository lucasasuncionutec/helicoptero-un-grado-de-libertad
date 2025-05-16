#include "codigoMPU.h"

// Instancia del sensor
MPU6050 sensor;

// Variables internas
float offsetCalibracion = 0.0;

// Filtro pasabajo (LPF de primer orden discreto)
float anguloFiltrado = 0.0;
float alphaSuavizado = 0.05; // Ajusta este valor según cuánta suavidad quieras

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
    if (!sensor.testConnection()) {
        Serial.println("Error: No se pudo leer el sensor MPU6050");
        return NAN; // o algún valor inválido especial
    }

    int16_t axRaw, ayRaw, azRaw;
    sensor.getAcceleration(&axRaw, &ayRaw, &azRaw);

    float ax = axRaw / 16384.0;
    float ay = ayRaw / 16384.0;
    float az = azRaw / 16384.0;

    if (isnan(ax) || isnan(ay) || isnan(az)) {
        Serial.println("Lectura inválida del sensor");
        return NAN;
    }

    return atan(-ax / sqrt(ay * ay + az * az)) * (180.0 / 3.14159265359) - offsetCalibracion;
}


// Solo aplica el filtro pasabajo
float leerSensor() {
    float medida = leerAnguloBruto();
    anguloFiltrado = alphaSuavizado * medida + (1.0 - alphaSuavizado) * anguloFiltrado;
    return anguloFiltrado;
}
