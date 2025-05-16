#include "CodigoComunicacionSoftware.h"

// === Función para recibir un vector de datos separados por coma ===
void recibirDatosSerial(float* vector, int maxValores, int* cantidadLeida) {
    static String buffer = "";
    *cantidadLeida = 0;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
            int idx = 0;
            char mensaje[64];
            buffer.toCharArray(mensaje, sizeof(mensaje));
            char* token = strtok(mensaje, ",");

            while (token != NULL && idx < maxValores) {
                vector[idx++] = atof(token);
                token = strtok(NULL, ",");
            }

            *cantidadLeida = idx;
            buffer = "";
            break;
        } else {
            buffer += c;
        }
    }
}

// === Función para enviar un vector como CSV ===
void enviarDatosSerial(float* vector, int cantidad) {
    for (int i = 0; i < cantidad; i++) {
        Serial.print(vector[i], 4);
        if (i < cantidad - 1) Serial.print(",");
    }
    Serial.println();
}


// === Función para comunicación de datos específicos ===


void leerDatosDesdePC(float& Tss_ref, float& Mp_ref) {
    float datosRecibidos[10];
    int cantidadRecibida = 0;
    recibirDatosSerial(datosRecibidos, 10, &cantidadRecibida);

    if (cantidadRecibida >= 2) {
        Tss_ref = datosRecibidos[0];
        Mp_ref  = datosRecibidos[1];
    }
}

void enviarDatosALaPC(float angulo_deg, float error_deg, float pwm) {
    float datosAEnviar[3] = {angulo_deg, error_deg, pwm};
    enviarDatosSerial(datosAEnviar, 3);
}
