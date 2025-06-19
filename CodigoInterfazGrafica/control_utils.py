import numpy as np
from control import TransferFunction, tf, feedback

class ControlSystem:
    def __init__(self):
        # === Parámetros físicos ===
        self.I  = 0.0167
        self.C  = -0.1326
        self.Lm = 0.3310
        self.m  = 0.001586
        self.r  = -1.692631

        # === Estado ===
        self.theta_eq_rad = 0.0
        self.anguloReferencia_rad = 0.0

        # === PIDf (parámetros y coeficientes) ===
        self.Kp = 0; self.Ki = 0; self.Kd = 0; self.N = 1
        self.Ts = 0.022  # [s] período de muestreo
        self.a0 = self.a1 = self.a2 = self.a3 = self.a4 = self.a5 = 0

        # === Estados anteriores del PIDf ===
        self.error_km1 = 0.0
        self.error_km2 = 0.0
        self.u_km1 = 0.0
        self.u_km2 = 0.0

        # === PWM equilibrio ===
        self.PWM_eq = self.pwm_equilibrio()

        # === Modelos dinámicos ===
        self.motor_models = {
            'Modelo estático 1': TransferFunction([self.m], [1])
        }
        self.mech_models = {
            'Modelo sin fricción': self._make_mech_tf(self.theta_eq_rad)
        }

    # === PWM equilibrio ===
    def pwm_equilibrio(self):
        fuerza_eq = -(self.C * np.cos(self.theta_eq_rad)) / self.Lm
        return (fuerza_eq - self.r) / self.m

    # === Generar G(s) mecánico ===
    def _make_mech_tf(self, theta_eq_rad):
        A = self.Lm / self.I
        B = (self.C * np.sin(theta_eq_rad)) / self.I
        self.last_A = A
        self.last_B = B
        return TransferFunction([A], [1, 0, B])

    def set_equilibrium_angle_deg(self, angle_deg):
        self.theta_eq_rad = np.radians(angle_deg)
        self.mech_models['Mecánica 1'] = self._make_mech_tf(self.theta_eq_rad)
        self.PWM_eq = self.pwm_equilibrio()

    def get_motor_tf(self, name='Modelo estático 1'):
        return self.motor_models[name]

    def get_mech_tf(self, name='Modelo sin fricción'):
        return self.mech_models[name]

    def get_real_plant_tf(self):
        return self.get_motor_tf() * self.get_mech_tf()

    # === PIDf continuo como función de transferencia ===
    def pidf_tf(self, Kp, Ki, Kd, N):
        s = tf([1, 0], [0, 1])
        return Kp + Ki/s + (Kd * N * s) / (s + N)

    # === Generación del PIDf discreto desde Tss y Mp ===
    def assignment_tf(self, Tss, Mp):
        lnMp = np.log(Mp)
        zeta = -lnMp / np.sqrt(np.pi**2 + lnMp**2)
        wn = 4.0 / (zeta * Tss)
        tf_2nd_order = TransferFunction([wn**2], [1, 2*zeta*wn, wn**2])
        return tf_2nd_order, zeta, wn

    # === Establecer parámetros del PID y calcular coeficientes ===
    def set_pid_params(self, Kp, Ki, Kd, N):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.N  = N

        T = self.Ts
        a = Kp + Kd * N
        b = Kp * N + Ki
        c = Ki * N
        d = N

        K1 = (b * d - c) / (d * d)
        K2 = c / d
        K3 = (a * d * d - b * d + c) / (d * d)

        self.a0 = K1 + K3
        self.a1 = -K1 - K1 * np.exp(-d * T) + K2 * T - 2.0 * K3
        self.a2 = K1 * np.exp(-d * T) - K2 * T * np.exp(-d * T) + K3
        self.a3 = 1.0
        self.a4 = -np.exp(-d * T) - 1.0
        self.a5 = np.exp(-d * T)

        print(f"[CONTROL] PIDf discreto actualizado. a0={self.a0:.3f}, ..., a5={self.a5:.3f}")

    # === Calcular PWM desde el ángulo actual (en grados) ===
    def calcular_pwm(self, ang_actual_deg):
        theta_rad = np.radians(ang_actual_deg)
        error = self.anguloReferencia_rad - theta_rad

        u = (1.0 / self.a3) * (
            self.a0 * error + self.a1 * self.error_km1 + self.a2 * self.error_km2
            - self.a4 * self.u_km1 - self.a5 * self.u_km2
        )
        pwm = np.clip(u + self.PWM_eq, 1000.0, 2000.0)

        # Actualizar estados
        self.error_km2 = self.error_km1
        self.error_km1 = error
        self.u_km2 = self.u_km1
        self.u_km1 = u

        return pwm

    def set_pidf_coefs(self, kp, ki, kd, n, Ts):
        # Coeficientes del PIDf discreto usando el mismo cálculo que Arduino
        a = kp + kd * n
        b = kp * n + ki
        c = ki * n
        d = n

        K1 = (b * d - c) / (d * d)
        K2 = c / d
        K3 = (a * d * d - b * d + c) / (d * d)

        self.a0 = K1 + K3
        self.a1 = -K1 - K1 * np.exp(-d * Ts) + K2 * Ts - 2.0 * K3
        self.a2 = K1 * np.exp(-d * Ts) - K2 * Ts * np.exp(-d * Ts) + K3
        self.a3 = 1.0
        self.a4 = -np.exp(-d * Ts) - 1.0
        self.a5 = np.exp(-d * Ts)

        # Guardar estados iniciales
        self.e_km1 = 0.0
        self.e_km2 = 0.0
        self.u_km1 = 0.0
        self.u_km2 = 0.0
