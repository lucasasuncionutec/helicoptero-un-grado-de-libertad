#include "CodigoComunicacionSoftware.h"

extern bool pid_recibido_desde_pc;
extern bool tss_mp_recibidos_pc;
extern bool pwm_recibido_pc;


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


bool leerDatosDesdePC(float& Tss_ref, float& Mp_ref,
                      float& kp_pc, float& ki_pc, float& kd_pc, float& n_pc,
                      float& pwm_pc, bool& toggle_pc)
{
    float datosRecibidos[10];
    int cantidadRecibida = 0;
    recibirDatosSerial(datosRecibidos, 10, &cantidadRecibida);

    if (cantidadRecibida >= 2) {
        Tss_ref = datosRecibidos[0];
        Mp_ref  = datosRecibidos[1];
        tss_mp_recibidos_pc = true;
    }

    if (cantidadRecibida >= 6) {
        kp_pc = datosRecibidos[2];
        ki_pc = datosRecibidos[3];
        kd_pc = datosRecibidos[4];
        n_pc  = datosRecibidos[5];
        pid_recibido_desde_pc = true;
    }

    if (cantidadRecibida >= 7) {
        pwm_pc = datosRecibidos[6];
        pwm_recibido_pc = true;
    }

    if (cantidadRecibida >= 8) {
        toggle_pc = datosRecibidos[7] > 0.5f;
    }

    return (cantidadRecibida >= 6);  // true si al menos vino PID completo
}



void enviarDatosALaPC(float angulo_deg, float error_deg, float pwm) {
    float datosAEnviar[3] = {angulo_deg, error_deg, pwm};
    enviarDatosSerial(datosAEnviar, 3);
}
