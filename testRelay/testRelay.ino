// Definición de pines
#define RELAY_PIN 10   // Pin para el relé
#define SWITCH_PIN 3  // Pin para el switch

// Variable para el estado anterior del switch
bool lastSwitchState = HIGH; // Estado previo del switch

void setup() {
    // Configurar pines
    pinMode(RELAY_PIN, OUTPUT);       // Configurar relé como salida
    pinMode(SWITCH_PIN, INPUT_PULLUP); // Configurar switch como entrada con resistencia pull-up interna

    // Inicializar puerto serie
    Serial.begin(9600);
    Serial.println("Prueba de switch iniciada.");
}

void loop() {
    // Leer el estado actual del switch
    bool currentSwitchState = digitalRead(SWITCH_PIN);
    Serial.println(currentSwitchState);
    // Detectar flanco descendente (de HIGH a LOW)
    if (currentSwitchState == LOW && lastSwitchState == HIGH) {
      digitalWrite(RELAY_PIN, HIGH);
    }else if(currentSwitchState == HIGH && lastSwitchState == LOW){
      digitalWrite(RELAY_PIN, LOW);
    }

    // Actualizar el estado previo del switch
    lastSwitchState = currentSwitchState;

    // Pequeño retraso para evitar saturar el monitor serie (opcional)
    delay(50); // Ajusta si es necesario
}
