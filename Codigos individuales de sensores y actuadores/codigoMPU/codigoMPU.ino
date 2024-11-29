#include "MPU6050.h"

MPU6050 sensor;
int ax, ay, az;

float angulo;
float calibracion_angulo = -3.9;

void setup() {
    Serial.begin(9600);  // Iniciar puerto serial
    Wire.begin();        // Iniciar comunicación I2C

    sensor.initialize();  // Inicializar el sensor

    while (!sensor.testConnection()) {
        Serial.println("Intentando iniciar el sensor");
        delay(1000);  // Esperar 1 segundo antes de intentar nuevamente
    }
    Serial.println("Sensor iniciado correctamente");
}

void loop() {
    leerSensor(); // Leer la medida del sensor MPU y calcular el ángulo
    
    Serial.print("Ángulo (grados): ");
    Serial.println(angulo);
    
    delay(100);  
}

void leerSensor() {
  sensor.getAcceleration(&ax, &ay, &az);
  
  // Calcular ángulo con acelerómetro 
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0/3.14);

  angulo = accel_ang_y - calibracion_angulo;
}
