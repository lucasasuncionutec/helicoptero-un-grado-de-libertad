#include "CodigoComunicacionSoftware.h"

extern bool pid_recibido_desde_pc;
extern bool tss_mp_recibidos_pc;
extern bool pwm_recibido_pc;
extern bool nuevo_pwm_pc;

// === Función para recibir un vector de datos separados por coma ===
void recibirDatosSerial(float* vector, int maxValores, int* cantidadLeida) {
    char mensaje[80];  // Asumimos que 80 caracteres son suficientes
    int leidos = Serial.readBytesUntil('\n', mensaje, sizeof(mensaje) - 1);
    mensaje[leidos] = '\0';

    *cantidadLeida = 0;
    char* token = strtok(mensaje, ",");

    while (token != NULL && *cantidadLeida < maxValores) {
        vector[(*cantidadLeida)++] = atof(token);
        token = strtok(NULL, ",");
    }
}


// === Función para enviar un vector como CSV ===
void enviarDatosSerial(float* vector, int cantidad) {
    char mensaje[80] = "";  // Ajustar tamaño si hay más valores
    char temp[20];

    for (int i = 0; i < cantidad; i++) {
        dtostrf(vector[i], 0, 4, temp);  // convierte float en string con 4 decimales
        strcat(mensaje, temp);
        if (i < cantidad - 1) strcat(mensaje, ",");
    }

    Serial.println(mensaje);  // Solo un println al final
}


// === Función para comunicación de datos específicos ===
#include <math.h>  // Para isnan()

bool leerDatosDesdePC(float& Tss_ref, float& Mp_ref,
                      float& kp_pc, float& ki_pc,
                      float& kd_pc, float& n_pc,
                      float& pwm_pc, bool& toggle_pc)
{
    static char buf[80];
    static uint8_t idx = 0;

    // Si no hay ni un byte, salgo rápido
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            buf[idx] = '\0';
            idx = 0;
            // acá tenés una línea completa en buf[]
            float datos[10];
            int leidos = 0;
            char *tok = strtok(buf, ",");
            while (tok && leidos < 10) {
                datos[leidos++] = atof(tok);
                tok = strtok(nullptr, ",");
            }

            // -- procesá datos[0..leidos-1] igual que antes --
            bool recibioAlgoValido = false;
            // 1) Tss, Mp…
            if (leidos >= 2) {
                if (!isnan(datos[0]) && datos[0]>=0) {
                    Tss_ref = datos[0];
                    tss_mp_recibidos_pc = true;
                    recibioAlgoValido = true;
                }
                if (!isnan(datos[1]) && datos[1]>=0) {
                    Mp_ref = datos[1];
                    tss_mp_recibidos_pc = true;
                    recibioAlgoValido = true;
                }
            }
            // 2) PID completo…
            if (leidos >= 6) {
                if (!isnan(datos[2]) && !isnan(datos[3]) &&
                    !isnan(datos[4]) && !isnan(datos[5])) {
                    kp_pc = datos[2];
                    ki_pc = datos[3];
                    kd_pc = datos[4];
                    n_pc  = datos[5];
                    pid_recibido_desde_pc = true;
                    recibioAlgoValido = true;
                }
            }
            // 3) PWM…
            if (leidos >= 7 && !isnan(datos[6]) &&
                datos[6]>=1000 && datos[6]<=2000) {
                pwm_pc = datos[6];
                nuevo_pwm_pc = true;
                pwm_recibido_pc = true;
                recibioAlgoValido = true;
            }
            // 4) toggle…
            if (leidos >= 8) {
                toggle_pc = datos[7] > 0.5f;
            }

            return recibioAlgoValido;
        } else {
            // Mientras no sea \n, voy acumulando (si no desbordás)
            if (idx < sizeof(buf)-1) buf[idx++] = c;
        }
    }

    // Si no había datos para leer:
    return false;
}


void enviarDatosALaPC(float angulo_deg, float error_deg, float pwm) {
    Serial.print("#");  // ← Marca de inicio de trama válida
    Serial.print(angulo_deg, 4); Serial.print(",");
    Serial.print(error_deg, 4);  Serial.print(",");
    Serial.println(pwm, 4);
}

