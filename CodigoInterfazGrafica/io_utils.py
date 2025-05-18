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
        self.esc_warning_active = False  # NUEVO

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
                    if not line:
                        continue

                    # --- Detectar advertencia ESC ---
                    if "calibre" in line.lower() and "esc" in line.lower():  # NUEVO
                        self.esc_warning_active = True                      # NUEVO
                        self.queue.put(("ESC_WARNING", line))               # NUEVO
                        continue                                            # NUEVO

                    # --- Datos de ángulo,error,pwm ---
                    partes = line.split(",")
                    if len(partes) == 3 and all(self._es_float(p) for p in partes):
                        self.queue.put(tuple(map(float, partes)))
                    else:
                        print(f"[SERIAL] Línea ignorada (esperaba 3 valores): {line}")
                else:
                    time.sleep(0.001)
        except Exception as e:
            self.queue.put(f"ERROR: {e}")

    @staticmethod
    def _es_float(s):
        try:
            float(s)
            return True
        except ValueError:
            return False

    def _simulate_data(self):
        import math, random

        t0 = time.time()
        ref = 20.0
        angle = -50.0
        dt = 0.01
        y = angle
        dy = 0

        wn = 2.0
        zeta = 0.7

        while self.running:
            t = time.time() - t0
            err = ref - y
            ddy = wn**2 * err - 2 * zeta * wn * dy
            dy += ddy * dt
            y += dy * dt

            pwm_eq = 1500
            pwm = pwm_eq + 10 * err + random.uniform(-2, 2)
            pwm = max(1000, min(2000, pwm))

            y_noisy = y + random.uniform(-0.5, 0.5)
            err_noisy = err + random.uniform(-0.3, 0.3)

            self.queue.put((y_noisy, err_noisy, pwm))
            time.sleep(dt)

    def send_command(self, msg):
        if self.serial and self.serial.is_open:
            try:
                self.serial.write((msg.strip() + "\n").encode())
                print(f"[Serial] Enviado: {msg.strip()}")
            except Exception as e:
                print(f"[Serial] Error al enviar: {e}")

    def stop(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.serial = None

def save_config(path, cfg):
    with open(path, 'w') as f:
        json.dump(cfg, f, indent=2)

def load_config(path):
    with open(path) as f:
        return json.load(f)
