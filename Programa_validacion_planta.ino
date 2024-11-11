#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;

// Variables para el sensor
int ax, ay, az; 
int gx, gy, gz; 
float ang_y = 0;
float ang_y_prev = 0;
float ang_real = 0;
float calibration = -14.75;

unsigned long tiempo_prev;
unsigned long tiempo_actual;
unsigned long tiempo_inicio_muestras;

// Estructura para las medidas
struct Medida {
  unsigned long tiempo;
  float angulo;
};

const int frecuencia_muestreo = 50;  // 20 ms cada muestra
const int max_medidas = 200;
Medida medidas[max_medidas];
int indice_medidas = 0;
volatile bool tomar_muestra = false;
unsigned long duracion_simulacion;
unsigned long tiempo_inicio;

void setup() {
  Serial.begin(9600);  // Inicializa el puerto serial
  Wire.begin();        // Inicia la comunicación I2C

  sensor.initialize();  // Inicializa el MPU6050

  // Verifica si el sensor se ha conectado correctamente
  while (!sensor.testConnection()) {
    Serial.println("Intentando iniciar el sensor");
    delay(1000);  // Espera un segundo antes de intentar nuevamente
  }
  Serial.println("Sensor iniciado correctamente");

  // Fase de estabilización: lee el ángulo durante 20 segundos y lo muestra por consola
  Serial.println("Esperando 20 segundos para estabilizar el sensor...");
  tiempo_inicio_muestras = millis();
  while (millis() - tiempo_inicio_muestras < 20000) {
    leerSensor();  // Lee el sensor
    Serial.print("Ángulo Y: ");
    Serial.println(ang_real);  // Muestra el ángulo en la consola
    delay(50);  // Pausa de 50 ms para evitar sobrecargar
  }
  Serial.println("Estabilización completada.");

  // Configura el Timer 1 para generar interrupciones a la frecuencia definida
  configurar_timer1();  
  tiempo_inicio = millis();  // Guarda el tiempo de inicio

  // Leer el tiempo de simulación desde serial
  Serial.println("Ingrese el tiempo de simulación en milisegundos:");
  while (Serial.available() == 0) {}
  duracion_simulacion = Serial.parseInt();

  Serial.println("Simulación iniciada.");
}

void loop() {
  if (tomar_muestra && indice_medidas < max_medidas) {
    tomar_muestra = false;

    // Lee el sensor MPU6050 para obtener el ángulo real
    leerSensor();

    // Guarda la medición
    medidas[indice_medidas].tiempo = millis() - tiempo_inicio;
    medidas[indice_medidas].angulo = ang_real;  // Usa el ángulo real en lugar de un valor aleatorio
    indice_medidas++;
  }

  if (millis() - tiempo_inicio >= duracion_simulacion) {
    Serial.println("Simulación finalizada.");
    enviar_datos_serial();
    while (true);  // Detener la ejecución
  }
}

void configurar_timer1() {
  noInterrupts();  // Deshabilitar interrupciones durante la configuración

  TCCR1A = 0;  // Limpiar registros
  TCCR1B = 0;
  TCNT1 = 0;   // Reiniciar el contador

  // Modo CTC (Clear Timer on Compare Match)
  TCCR1B |= (1 << WGM12);

  // Configura el prescaler a 64
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // Calcula el valor de OCR1A para la frecuencia deseada
  OCR1A = (16000000 / (64 * frecuencia_muestreo)) - 1;

  // Habilita la interrupción del comparador del timer
  TIMSK1 |= (1 << OCIE1A);  

  interrupts();  // Habilitar interrupciones
}

ISR(TIMER1_COMPA_vect) {
  tomar_muestra = true;
}

void enviar_datos_serial() {
  Serial.println("Tiempo(ms), Angulo(grados)");
  for (int i = 0; i < indice_medidas; i++) {
    Serial.print(medidas[i].tiempo);
    Serial.print(", ");
    Serial.println(medidas[i].angulo);
  }
}

void leerSensor() {
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  
  float dt = (micros() - tiempo_prev) / 1000000.0;  // Usar micros() para mayor precisión
  tiempo_prev = micros();
  
  // Calcular los ángulos con el acelerómetro
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
  
  // Calcular el ángulo de rotación con el giroscopio y aplicar el filtro complemento
  ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt) + 0.02 * accel_ang_y;
  
  ang_y_prev = ang_y;
  ang_real = ang_y - calibration;
  ang_real = - ang_real;
}
