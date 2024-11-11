// Librerias I2C para controlar el MPU6050
// La libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h>

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implícito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerómetro y giroscopio en los ejes x, y, z
int ax, ay, az; 
int gx, gy, gz; 

long tiempo_prev;
float dt;
float ang_y;  // Ahora en radianes
float ang_y_prev;
float ang_real;  // También en radianes

float calibration = -0.257;  // Calibración en radianes (-14.75° = -0.257 rad)

int velocidad_actual = 1000;
Servo esc;

// Variables para el controlador
float x, x_1, x_2;
float y, y_1, y_2;
int T = 100;  //Periodo de muestreo en ms (T=0.1 s)
unsigned long t_pasado;
unsigned long t_actual;

void setup() {
    Serial.begin(9600);  // Iniciando puerto serial
    Wire.begin();        // Iniciando I2C  
    
    sensor.initialize();  // Inicializar el sensor

    while (!sensor.testConnection()) {
        Serial.println("Intentando iniciar el sensor");
        delay(1000);  // Esperar un segundo antes de intentar nuevamente
    }
    Serial.println("Sensor iniciado correctamente");

    esc.attach(2);
    esc.writeMicroseconds(0);

    tiempo_prev = millis();  // Inicializar tiempo_prev

    // Inicializar variables del controlador
    x = 0.0;
    y = 0.0;
    x_1 = 0.0;
    x_2 = 0.0;
    y_1 = 0.0;
    y_2 = 0.0;
}

void loop() {
    t_actual = millis();   // Almacena el tiempo que el programa ha estado corriendo
    int delta_tiempo = t_actual - t_pasado;  // Controla el tiempo al que se actualiza la acción de control
    
    if (delta_tiempo >= T) {
        leerSensor();  // Obtener el ángulo del sensor MPU6050
        
        // Actualización del valor de entrada para el controlador
        x = ang_real;  // Utilizamos el ángulo en radianes como entrada de control
        
        // Controlador discreto
        y = 0.0842 * x + 0.0842 * x_1 + 1.61 * y_1 - 0.7997 * y_2;  // Acción de control

        // Actualización de variables para la siguiente iteración
        x_2 = x_1;
        x_1 = x;
        y_2 = y_1;
        y_1 = y;

        // Convertir la salida del controlador a un PWM válido para el ESC
        escribirVelocidadEnESC(map(y, -1.57, 1.57, 1000, 2000));  // Mapeo de radianes a PWM (entre -π/2 y π/2 radianes)
        
        Serial.print("Ángulo (radianes): ");
        Serial.println(ang_real);
        Serial.print("Controlador (y): ");
        Serial.println(y);

        t_pasado = t_actual;
    }

    if (Serial.available() > 0) {
        int velocidad = Serial.parseInt();  // Leer el valor en microsegundos desde la consola serial
        if (velocidad >= 1000 && velocidad <= 2000) {
            escribirVelocidadEnESC(velocidad);
            Serial.print(velocidad_actual);
        } else {
            Serial.println("Por favor ingrese un valor entre 1000 y 2000 microsegundos.");
        }
    }
}

void leerSensor() {
    // Leer las aceleraciones y velocidades angulares
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);
    
    dt = (millis() - tiempo_prev) / 1000.0;
    tiempo_prev = millis();
    
    // Calcular los ángulos con acelerómetro
    float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)));  // Ahora devuelve radianes
    
    // Calcular ángulo de rotación con giroscopio y filtro complemento (en radianes)
    ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt) + 0.02 * accel_ang_y;
    
    ang_y_prev = ang_y;
    ang_real = ang_y - calibration;  // Aplicar la calibración en radianes
}

void escribirVelocidadEnESC(int velocidad) {
    int diferencia = velocidad - velocidad_actual;
    for (int i = 0; i < abs(diferencia); i++) {
        if (diferencia > 0) {
            velocidad_actual++;
        } else if (diferencia < 0) {
            velocidad_actual--;
        }
        velocidad_actual = constrain(velocidad_actual, 1000, 2000);
        esc.writeMicroseconds(velocidad_actual);
        delay(1);  // Retardo para una transición suave
    }
}
