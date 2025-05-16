import numpy as np
from control import TransferFunction, tf, feedback

class ControlSystem:
    def __init__(self):
        # Parámetros físicos
        self.I  = 0.0167       # Momento de inercia
        self.C  = -0.1326      # Constante de torque
        self.Lm = 0.3310       # Longitud desde el eje hasta el motor
        self.m  = 0.001586     # Pendiente de fuerza vs PWM
        self.r  = -1.692631    # Ordenada al origen

        # Ángulo de equilibrio (por defecto)
        self.theta_eq_rad = 0.0

        # Modelos únicos
        self.motor_models = {
            'Modelo estático 1': TransferFunction([self.m], [1])  # modelo eléctrico: G(s) = m
        }

        self.mech_models = {
            'Modelo sin fricción': self._make_mech_tf(self.theta_eq_rad)  # se actualiza con el ángulo
        }

    # --- Utilidad interna para crear el modelo mecánico dado un ángulo ---
    def _make_mech_tf(self, theta_eq_rad):
        A = self.Lm / self.I
        B = (self.C * np.sin(theta_eq_rad)) / self.I
        self.last_A = A
        self.last_B = B
        return TransferFunction([A], [1, 0, B])


    # --- Permite al usuario actualizar el ángulo desde GUI ---
    def set_equilibrium_angle_deg(self, angle_deg):
        self.theta_eq_rad = np.radians(angle_deg)
        self.mech_models['Mecánica 1'] = self._make_mech_tf(self.theta_eq_rad)

    # --- Accesores de modelos ---
    def get_motor_tf(self, name='Motor A'):
        return self.motor_models[name]

    def get_mech_tf(self, name='Mecánica 1'):
        return self.mech_models[name]

    def get_real_plant_tf(self):
        Gm = self.get_mech_tf()
        Ge = self.get_motor_tf()
        return Ge * Gm

    # --- Controladores ---
    def pidf_tf(self, Kp, Ki, Kd, N):
        s = tf([1, 0], [0, 1])
        return Kp + Ki/s + (Kd * N * s) / (s + N)

    def assignment_tf(self, Tss, Mp):
        zeta = -np.log(Mp) / np.sqrt(np.pi**2 + (np.log(Mp))**2)
        wn = 4.0 / (zeta * Tss)
        return TransferFunction([wn**2], [1, 2*zeta*wn, wn**2])

    def closed_loop(self, C):
        G = self.get_real_plant_tf()
        return feedback(C * G, 1)

    def pwm_equilibrio(self):
        theta = self.theta_eq_rad
        fuerza_eq = -(self.C * np.cos(theta)) / self.Lm
        return (fuerza_eq - self.r) / self.m
