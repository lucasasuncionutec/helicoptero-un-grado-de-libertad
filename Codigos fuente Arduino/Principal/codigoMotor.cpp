#include "codigoMotor.h"

Servo motorESC;

// Variables internas
static int ultimoEstadoSwitch = HIGH; // Estado inicial del switch
static bool yaCalibrado = false;   // Indica si el ESC ya fue calibrado
int velocidadActual = 1000;       // Velocidad actual del ESC

// Variables auxiliares
unsigned long ultimoTiempoRebote = 0; // Tiempo del último cambio de estado
unsigned long retrasoRebote = 100;    // Tiempo para evitar rebotes

void inicializarMotor() {
    motorESC.attach(ESC_PIN);
    motorESC.writeMicroseconds(1000); // Señal mínima
    
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);  // Relé apagado
}

// Detectar si se presiona el switch con rebote
bool leerSwitchRebote() {
    int estadoActualSwitch = digitalRead(SWITCH_PIN);
    unsigned long tiempoActual = millis();
    if ((tiempoActual - ultimoTiempoRebote) > retrasoRebote) {
        if (estadoActualSwitch == LOW && ultimoEstadoSwitch == HIGH) { 
            ultimoEstadoSwitch = estadoActualSwitch;
            ultimoTiempoRebote = tiempoActual;
            return true; 
        }
    }
    ultimoEstadoSwitch = estadoActualSwitch;
    return false; 
}

// Loop que inicia la calibración si se presiona el switch dos veces, incluye sistemas de seguridad
void loopCalibracion() {
    static unsigned long tiempoPrimerToque = 0; // Tiempo del primer toque
    static bool esperandoSegundoToque = false; // Indica si está esperando un segundo toque

    if (leerSwitchRebote()) {
        unsigned long tiempoActual = millis();

        if (!esperandoSegundoToque) {
            // Primer toque detectado
            esperandoSegundoToque = true;
            tiempoPrimerToque = tiempoActual;
            motorESC.writeMicroseconds(1000);
            if (!yaCalibrado) {
                Serial.println("Presione el botón otra vez solo si no escucha tres sonidos consecutivos.");
                digitalWrite(RELAY_PIN, LOW); // Apagar relé
                delay(1000);
                digitalWrite(RELAY_PIN, HIGH); // Encender relé
            } else {
                Serial.println("Reinicie el Arduino para calibrar.");
            }
        } else {
            // Segundo toque detectado
            if (tiempoActual - tiempoPrimerToque <= 3000) { // Si ocurre dentro de 3 segundos
                if (!yaCalibrado) {
                    calibrarESC();
                }
                esperandoSegundoToque = false; // Resetea el estado
            }
        }
    }

    // Solo un toque detectado luego del tiempo de espera
    if (esperandoSegundoToque && (millis() - tiempoPrimerToque > 3000)) {
        yaCalibrado = true;
        esperandoSegundoToque = false; // Resetea el estado
    }
}

void calibrarESC() {
    if (!yaCalibrado) {
        Serial.println("Calibración iniciada...");
 
        // Apagar el relé
        digitalWrite(RELAY_PIN, LOW);
        motorESC.writeMicroseconds(2000); // Señal máxima
        delay(1000);
  
        // Encender el relé
        digitalWrite(RELAY_PIN, HIGH);
        motorESC.writeMicroseconds(1000); // Señal mínima
        delay(6000);
  
        Serial.println("Calibración completada.");
        yaCalibrado = true;
    }
}

bool escribirVelocidadEnESC(int velocidad) {
    if (yaCalibrado) {
        int diferencia = velocidad - velocidadActual;
        // Crear una rampa de velocidad para evitar cambios abruptos
        for (int i = 0; i < abs(diferencia); i++) {
            if (diferencia > 0) {
                velocidadActual++;
            } else if (diferencia < 0) {
                velocidadActual--;
            }
            Serial.println(velocidadActual);
            motorESC.writeMicroseconds(velocidadActual);
            delay(1); 
        }
        return true;
    } else {
        Serial.println("Calibre el ESC antes de intentar usar el motor.");
    }
    return false;
}
