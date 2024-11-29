#define RELAY_PIN 4 // Pin para el relé
#define SWITCH_PIN 2 // Pin para el switch

bool lastSwitchState = HIGH; // Estado previo del switch

void setup() {
    Serial.begin(9600);          // Iniciar puerto serial
    pinMode(RELAY_PIN, OUTPUT);  // Configuración del pin del relé
    pinMode(SWITCH_PIN, INPUT_PULLUP); // Switch con resistencia pull-up interna
}

void loop() {
    bool currentSwitchState = digitalRead(SWITCH_PIN); // Leer estado del switch
    
    // Detectar flanco de bajada
    if ((currentSwitchState == LOW && lastSwitchState == HIGH)) {
        calibrarESC(); // Llamar a la función de calibración del ESC
    }
    lastSwitchState = currentSwitchState; // Actualizar el estado previo del switch
}


// Función para calibrar el ESC
void calibrarESC() {
    Serial.println("Iniciando calibración del ESC...");

    // Apagar el ESC
    digitalWrite(RELAY_PIN, LOW);
    delay(2000);  // Esperar 2 segundos para garantizar el apagado completo

    // Encender el ESC en modo calibración
    digitalWrite(RELAY_PIN, HIGH);
    delay(2000);

    Serial.println("Calibración completada");
}
