# io_utils.py
import serial
import threading
import queue
import time
import json

class SerialComm:
    def __init__(self, port='COM5', baud=9600, timeout=1, simulate=False):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.simulate = simulate
        self.serial = None
        self.queue = queue.Queue()
        self.running = False

    def start(self):
        if self.running:
            print("[Advertencia] El hilo de adquisición ya está corriendo.")
            return  # evita lanzar otro hilo

        self.running = True
        target = self._simulate_data if self.simulate else self._read_real
        threading.Thread(target=target, daemon=True).start()


    def _read_real(self):
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2)
            while self.running:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.queue.put(line)
                else:
                    time.sleep(0.001)
        except Exception as e:
            self.queue.put(f"ERROR: {e}")

    def _simulate_data(self):
        import math, random

        t0 = time.time()
        ref = 20.0  # grados
        angle = -50.0  # valor inicial
        dt = 0.01
        y = angle
        dy = 0

        # Parámetros del sistema de segundo orden (subamortiguado)
        wn = 2.0  # frecuencia natural
        zeta = 0.7  # factor de amortiguamiento

        while self.running:
            t = time.time() - t0

            # Simulación del sistema de segundo orden discretizado
            err = ref - y
            ddy = wn**2 * err - 2 * zeta * wn * dy
            dy += ddy * dt
            y += dy * dt

            # PWM simulado como proporcional al error (con saturación)
            pwm_eq = 1500
            pwm = pwm_eq + 10 * err + random.uniform(-2, 2)
            pwm = max(1000, min(2000, pwm))

            # Ruido leve agregado al ángulo y error
            y_noisy = y + random.uniform(-0.5, 0.5)
            err_noisy = err + random.uniform(-0.3, 0.3)

            self.queue.put((t, y_noisy, err_noisy, pwm))
            time.sleep(dt)


    def stop(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()

def save_config(path, cfg):
    with open(path, 'w') as f:
        json.dump(cfg, f, indent=2)

def load_config(path):
    with open(path) as f:
        return json.load(f)
