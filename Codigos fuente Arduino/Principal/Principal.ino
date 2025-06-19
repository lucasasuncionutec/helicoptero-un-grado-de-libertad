// === Principal.ino ====================================================
// Librerías necesarias para control del sistema y adquisición de datos
#include <Arduino.h>
#include "codigoMotor.h"
#include "codigoEncoder.h"
#include "registroDatos.h"
#include "codigoIO.h"
#include "CodigoComunicacionSoftware.h"

// === Parámetros físicos ===
const float Lm = 0.3310f;
const float C  = -0.1326f;
const float I  = 0.0167f;
const float m  = 0.001586f;
const float r  = -1.692631f;

// === Tiempos ===
const float T = 22.0f;                     // [ms] período de muestreo
const float T_procesamiento_total = 1.0f;  // [ms] tiempo de cómputo estimado

// === Variables globales del PID ===
float Kp, Ki, Kd, N;
float a0, a1, a2, a3 = 1.0f, a4, a5;

// === Estados del PID ===
float error_km1 = 0.0f, error_km2 = 0.0f;
float u_km1     = 0.0f, u_km2     = 0.0f;

// === Variables de control por PC ===
float pwm_pc    = 1000.0f;  // último PWM recibido de PC
bool  usePwmPc  = false;    // si true, usar siempre pwm_pc

// === Variables compartidas con codigoIO ===
float Tss = 12.0f;
float Mp  = 0.20f;
bool  controlActivo = false;

// === Variables para display y registro ===
float pwmAplicado       = 0.0f;
float anguloActual_deg  = 0.0f;
float anguloActual_rad  = 0.0f;
float errorActual_deg   = 0.0f;

float angulo_equilibrio_rad = 0.0f;
float PWM_equilibrio        = 0.0f;
float fuerza_equilibrio     = 0.0f;
float anguloReferencia_rad  = 0.0f;

int pin_test_tiempo_muestreo = 8;

// Flags de recepción desde PC
bool pid_recibido_desde_pc = false;
bool tss_mp_recibidos_pc   = false;
bool pwm_recibido_pc       = false;
bool nuevo_pwm_pc    = false;

// ----------------------------------------------------------------------
void calcularPIDfCompleto(float ang_eq, float Mp_val, float Tss_val) {
    angulo_equilibrio_rad = ang_eq;
    float A = Lm / I;
    float B = C * sin(ang_eq) / I;
    float Ts = T / 1000.0f;

    // diseño sobrepoles
    float lnMp   = log(Mp_val);
    float zeta   = -lnMp / sqrt(PI*PI + lnMp*lnMp);
    float wn     = 4.0f / (zeta * Tss_val);

    // polos deseados p1, p2
    float p1     = 10.0f * zeta * wn;
    float p2     = 12.0f * zeta * wn;
    float alpha1 = p1 + p2 + 2.0f * zeta * wn;
    float alpha2 = p1*p2 + 2.0f*zeta*wn*(p1+p2) + wn*wn;
    float alpha3 = 2.0f*zeta*wn*p1*p2 + wn*wn*(p1+p2);
    float alpha4 = wn*wn*p1*p2;

    // ganancias PIDf
    Kp = (alpha3 - alpha1*B - (alpha4/alpha1)) / (alpha1 * m * A);
    Ki =  alpha4 / (alpha1 * m * A);
    Kd = (alpha2 - B - Kp*m*A) / (alpha1 * m * A);
    N  =  alpha1;

    // discretización
    float a = Kp + Kd * N;
    float b = Kp * N + Ki;
    float c = Ki * N;
    float d = N;

    float K1 = (b*d - c) / (d*d);
    float K2 = c / d;
    float K3 = (a*d*d - b*d + c) / (d*d);

    a0 = K1 + K3;
    a1 = -K1 - K1*exp(-d*Ts) + K2*Ts - 2.0f*K3;
    a2 =  K1*exp(-d*Ts) - K2*Ts*exp(-d*Ts) + K3;
    a3 = 1.0f;
    a4 = -exp(-d*Ts) - 1.0f;
    a5 =  exp(-d*Ts);

    fuerza_equilibrio = -(C * cos(ang_eq)) / Lm;
    PWM_equilibrio    = (fuerza_equilibrio - r) / m;
}

// ----------------------------------------------------------------------
void setup() {
    Serial.begin(9600);
    pinMode(pin_test_tiempo_muestreo, OUTPUT);

    inicializarMotor();
    inicializarEncoder();
    calibrarAnguloInicial(50.4f);

    lcd.init();
    lcd.backlight();
    showMenu();

    calcularPIDfCompleto(angulo_equilibrio_rad, Mp, Tss);
    inicializarRegistro(0.3f, 60);
}

// ----------------------------------------------------------------------
void loop() {
    static unsigned long tPrev      = 0;
    const unsigned long Ts_ms       = T - T_procesamiento_total;
    static bool           timerInit = false;
    static float          lastEq    = -999, lastTssVal = -1;

    if (millis() - tPrev < Ts_ms) return;
    tPrev = millis();
    digitalWrite(pin_test_tiempo_muestreo, HIGH);

    // 1) Calibración mecánica y UI
    loopCalibracion();
    navegarMenu();

    // 2) Leer y aplicar datos de PC
    float kp_rx, ki_rx, kd_rx, n_rx, Tss_rx, Mp_rx, pwm_rx;
    bool  toggle_rx,
          got = leerDatosDesdePC(
            Tss_rx, Mp_rx,
            kp_rx, ki_rx, kd_rx, n_rx,
            pwm_rx, toggle_rx
          );

    if (got) {
        if (!isnan(kp_rx)&&!isnan(ki_rx)&&!isnan(kd_rx)&&!isnan(n_rx)) {
            Kp=kp_rx; Ki=ki_rx; Kd=kd_rx; N=n_rx;
            pid_recibido_desde_pc = true;
        }
        if (!isnan(Tss_rx)&&!isnan(Mp_rx)) {
            Tss=Tss_rx; Mp=Mp_rx;
            tss_mp_recibidos_pc = true;
        }
        if (!isnan(pwm_rx)) {
            pwm_pc = pwm_rx;
            usePwmPc = true;
            pwm_recibido_pc = true;
        }
        controlActivo = toggle_rx;
    }

    // 3) Leer sensor
    anguloActual_deg = leerAngulo();
    anguloActual_rad = anguloActual_deg * PI/180.0f;
    float error = anguloReferencia_rad - anguloActual_rad;

    // 4) Calcular PWM
    float PWM_out;
    if (usePwmPc) {
        PWM_out = constrain(pwm_pc,1000,2000);
    } else {
        float u = (1.0f/a3)*(a0*error + a1*error_km1 + a2*error_km2 - a4*u_km1 - a5*u_km2);
        PWM_out = constrain(u+PWM_equilibrio,1000,2000);
        u_km2 = u_km1;  u_km1 = u;
    }

    // 5) Aplicar y mostrar
    pwmAplicado    = controlActivo ? PWM_out : 1000;
    errorActual_deg= error * 180.0f/PI;
    if (controlActivo && Tss>=12) escribirVelocidadEnESC(pwmAplicado);
    else                     escribirVelocidadEnESC(1000);

    error_km2 = error_km1;
    error_km1 = error;

    // 6) Recalcular PID si cambió
    if (controlActivo && Tss>=8 && (angulo_equilibrio_rad!=lastEq||Tss!=lastTssVal)) {
        calcularPIDfCompleto(angulo_equilibrio_rad, Mp, Tss);
        
        lastEq      = angulo_equilibrio_rad;
        lastTssVal  = Tss;
        pid_recibido_desde_pc = false;
        tss_mp_recibidos_pc   = false;

        // reiniciar estados para evitar salto ---
        error_km1 = error_km2 = 0.0f;
        u_km1     = u_km2     = 0.0f;
        if (!timerInit) { iniciarTemporizador(); timerInit = true; }
    }

    // 7) Registrar en SD
    static bool dumped = false;
    if (controlActivo && getCalibrado() && !registroCompleto() && verificarMuestreo()) {
        guardarRegistro(obtenerTiempoActual(),
                        anguloActual_deg,
                        PWM_out,
                        errorActual_deg);
    }
    if (controlActivo && registroCompleto() && !dumped) {
        imprimirDatos();
        dumped = true;
    }

    // 8) Enviar datos a PC
    enviarDatosALaPC(anguloActual_deg, errorActual_deg, pwmAplicado);

    digitalWrite(pin_test_tiempo_muestreo, LOW);
}