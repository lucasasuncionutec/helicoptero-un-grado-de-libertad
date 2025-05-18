// === Principal.ino ====================================================
// Librerías necesarias para control del sistema y adquisición de datos
#include <Arduino.h>
#include "codigoMotor.h"
#include "codigoEncoder.h"
#include "registroDatos.h"
#include "codigoIO.h"
#include "CodigoComunicacionSoftware.h"


// === Parámetros físicos ===
const float Lm = 0.3310;
const float C  = -0.1326;
const float I  = 0.0167;
const float m  = 0.001586;
const float r  = -1.692631; // El original es -1.692631

// === Tiempos ===
const float T = 22;                        // [ms] período de muestreo
const float T_procesamiento_total = 1;     // [ms] tiempo de cómputo estimado

// === Variables globales del PID ===
float Kp, Ki, Kd, N;
float a0, a1, a2, a3 = 1.0f, a4, a5;

// === Estados del PID ===
float error_km1 = 0.0, error_km2 = 0.0;
float u_km1 = 0.0, u_km2 = 0.0;

// === Variables compartidas con codigoIO ===
float Tss = 12.0;
float Mp  = 0.20;
bool  controlActivo = false;

float pwmAplicado       = 0.0;
float anguloActual_deg  = 0.0;
float anguloActual_rad  = 0.0;
float errorActual_deg   = 0.0;

float angulo_equilibrio_rad = 0.0;   // [rad]
float PWM_equilibrio        = 0.0;
float fuerza_equilibrio     = 0.0;
float anguloReferencia_rad  = 0.0;   // [rad]

int pin_test_tiempo_muestreo = 8;

bool pid_recibido_desde_pc = false;
bool tss_mp_recibidos_pc   = false;
bool pwm_recibido_pc       = false;
float pwm_pc = 1000.0f;     // PWM recibido desde la PC (si viene)
bool  toggle_pc = false;    // Estado del control (ON/OFF) recibido desde la PC



void calcularPIDfCompleto(float angulo_eq_rad, float Mp, float Tss)
{
    angulo_equilibrio_rad = angulo_eq_rad;

    float A   = Lm / I;
    float B   = C * sin(angulo_equilibrio_rad) / I;
    float T_s = T / 1000.0f;

    // Cálculo de parámetros deseados
    float lnMp  = log(Mp);  // Mp en [0,1]
    float zeta  = -lnMp / sqrt(PI * PI + lnMp * lnMp);

    float omega_n  = 4.0f / (zeta * Tss);

    float p1 = 10.0f * zeta * omega_n;
    float p2 = 12.0f * zeta * omega_n;

    float alpha1 = p1 + p2 + 2.0f * zeta * omega_n;
    float alpha2 = p1 * p2 + 2.0f * zeta * omega_n * (p1 + p2) + omega_n * omega_n;
    float alpha3 = 2.0f * zeta * omega_n * p1 * p2 + omega_n * omega_n * (p1 + p2);
    float alpha4 = omega_n * omega_n * p1 * p2;

    // Cálculo de ganancias del PIDf
    Kp = (alpha3 - alpha1 * B - (alpha4 / alpha1)) / (alpha1 * m * A);
    Ki = alpha4 / (alpha1 * m * A);
    Kd = (alpha2 - B - Kp * m * A) / (alpha1 * m * A);
    N  = alpha1;

    // Coeficientes del controlador discreto
    float a = Kp + Kd * N;
    float b = Kp * N + Ki;
    float c = Ki * N;
    float d = N;

    float K1 = (b * d - c) / (d * d);
    float K2 = c / d;
    float K3 = (a * d * d - b * d + c) / (d * d);

    a0 = K1 + K3;
    a1 = -K1 - K1 * exp(-d * T_s) + K2 * T_s - 2.0f * K3;
    a2 = K1 * exp(-d * T_s) - K2 * T_s * exp(-d * T_s) + K3;
    a3 = 1.0f;
    a4 = -exp(-d * T_s) - 1.0f;
    a5 = exp(-d * T_s);

    // Cálculo de la fuerza de equilibrio
    fuerza_equilibrio = -(C * cos(angulo_equilibrio_rad)) / Lm;
    PWM_equilibrio    = (fuerza_equilibrio - r) / m;

    // ----------------- Impresión en consola -----------------
    Serial.println("\n========== PID Recalculado ==========");
    Serial.print("Tss: "); Serial.print(Tss, 3); Serial.print(" s\t");
    Serial.print("Mp: ");  Serial.print(Mp, 2); Serial.println(" %");

    Serial.print("Zeta (ζ): "); Serial.println(zeta, 5);
    Serial.print("Omega_n (ωₙ): "); Serial.print(omega_n, 5); Serial.println(" rad/s");

    Serial.print("Kp: "); Serial.print(Kp, 5);
    Serial.print("\tKi: "); Serial.print(Ki, 5);
    Serial.print("\tKd: "); Serial.print(Kd, 5);
    Serial.print("\tN: ");  Serial.println(N, 5);

    Serial.print("Ángulo de equilibrio: ");
    Serial.print(angulo_equilibrio_rad * 180 / PI, 3); Serial.println("°");

    Serial.println("=====================================\n");
}



// ----------------------------------------------------------------------
void setup()
{
    Serial.begin(9600);
    pinMode(pin_test_tiempo_muestreo, OUTPUT);

    inicializarMotor();
    inicializarEncoder();
    calibrarAnguloInicial(50.4); // <-- asegúrate de que el offset esté en grados si este es su formato

    lcd.init();
    lcd.backlight();
    showMenu();

    calcularPIDfCompleto(angulo_equilibrio_rad, Mp, Tss); // Cálculo inicial
    inicializarRegistro(0.3f, 60);   // T_muestreo = 0.3 s, duración = 80 s
}

// ----------------------------------------------------------------------
void loop()
{
    static unsigned long t_anterior = 0;
    const unsigned long Ts_ms = T - T_procesamiento_total;

    // Variables de estado persistentes entre iteraciones
    static bool  yaIniciadoTemporizador = false;
    static float ultimoAngulo_rad       = -999.0f;
    static float ultimoTss              = -1.0f;
    static float ultimoMp               = -1.0f;

    if (millis() - t_anterior >= Ts_ms) {
        t_anterior = millis();

        digitalWrite(pin_test_tiempo_muestreo, HIGH);

        loopCalibracion();  // rutina de calibración mecánica

        // Detecta si vienen valores del software
        float kp_manual, ki_manual, kd_manual, n_manual;
        bool pid_manual_recibido = leerDatosDesdePC(Tss, Mp, kp_manual, ki_manual, kd_manual, n_manual, pwm_pc, toggle_pc);

        if (pid_manual_recibido) {
            Kp = kp_manual;
            Ki = ki_manual;
            Kd = kd_manual;
            N  = n_manual;

            pid_recibido_desde_pc = true;
            Serial.println("[Arduino] PID recibido desde PC → valores sobrescritos");
        }

        if (Tss != ultimoTss || Mp != ultimoMp) {
            tss_mp_recibidos_pc = true;
        }

        if (pwm_pc != 1000.0f) {
            pwm_recibido_pc = true;
        }

        if (pid_manual_recibido || pwm_recibido_pc || tss_mp_recibidos_pc) {
            controlActivo = toggle_pc;
        }

        // Medición de estado actual
        anguloActual_deg = leerAngulo();
        anguloActual_rad = anguloActual_deg * PI / 180.0f;
        float error = anguloReferencia_rad - anguloActual_rad;
        float u = 0.0f;

        // Control PIDf discreto
        float PWM_aplicar;
        if (pwm_recibido_pc) {
            PWM_aplicar = constrain(pwm_pc, 1000.0f, 2000.0f);
        } else {
            float u_local = (1.0f / a3) *
                            (a0 * error + a1 * error_km1 + a2 * error_km2
                            - a4 * u_km1  - a5 * u_km2);
            PWM_aplicar = constrain(u_local + PWM_equilibrio, 1000.0f, 2000.0f);
            u = u_local;
        }

        errorActual_deg = error * 180.0f / PI;
        pwmAplicado     = controlActivo ? PWM_aplicar : 1000.0f;

        if (controlActivo && Tss >= 12.0f) {
            escribirVelocidadEnESC(pwmAplicado);
        } else {
            escribirVelocidadEnESC(1000);
        }

        // Actualización de estados del PID
        error_km2 = error_km1; error_km1 = error;
        u_km2     = u_km1;     u_km1    = u;

        // Recalcular PID si cambian condiciones
        if (controlActivo && Tss >= 8.0f &&
            (angulo_equilibrio_rad != ultimoAngulo_rad || Tss != ultimoTss)) {
            
            calcularPIDfCompleto(angulo_equilibrio_rad, Mp, Tss);
            ultimoAngulo_rad = angulo_equilibrio_rad;
            ultimoTss        = Tss;
            ultimoMp         = Mp;

            // Restaurar control interno del Arduino
            pid_recibido_desde_pc = false;
            tss_mp_recibidos_pc   = false;
            pwm_recibido_pc       = false;

            if (!yaIniciadoTemporizador) {
                iniciarTemporizador();
                yaIniciadoTemporizador = true;
            }
        }

        // Registro de datos
        static bool datosYaImpresos = false;
        if (controlActivo && getCalibrado() && !registroCompleto() && verificarMuestreo()) {
            guardarRegistro(obtenerTiempoActual(), anguloActual_deg, PWM_aplicar, errorActual_deg);
        }
        if (controlActivo && registroCompleto() && !datosYaImpresos) {
            imprimirDatos();
            datosYaImpresos = true;
        }

        enviarDatosALaPC(anguloActual_deg, errorActual_deg, PWM_aplicar);
        navegarMenu();

        digitalWrite(pin_test_tiempo_muestreo, LOW);
    }
}
