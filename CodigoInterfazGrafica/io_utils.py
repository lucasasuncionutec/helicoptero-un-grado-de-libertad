# io_utils.py  – versión “libre-de-bloqueos” usando pyserial.threaded
# --------------------------------------------------------------------
# Requiere:  pip install pyserial   (>= 3.5)
# --------------------------------------------------------------------
import serial
import serial.threaded          # <— motor de hilos de pyserial
import threading
import queue
import json
import time
import numpy as np


# ════════════════════════════════════════════════════════════════════
# 1)  PROTOCOLO ────────────── decodifica líneas y las manda a la cola
# ════════════════════════════════════════════════════════════════════
class _LineProtocol(serial.threaded.LineReader):
    """
    Lee bytes → arma líneas → parsea tramas válidas y las entrega a la
    cola pasada en el constructor.
    """
    TERMINATOR = b'\n'

    def __init__(self, rx_queue, pwm_eq_ref_lambda):
        super().__init__()
        self.rx_queue = rx_queue
        self._calc_pwm_sw = pwm_eq_ref_lambda  # función para PWM soft

    # ---------- llamada automática por LineReader -------------------
    def handle_line(self, line: str) -> None:
        """
        Se ejecuta en *el hilo interno* de ReaderThread cada vez que
        llega una línea terminada en '\n'.
        Formatos:
          • #angulo,err,pwm
          • “calibre ESC …” (aviso)
        """
        line = line.strip()
        if not line:
            return

        if "calibre" in line.lower() and "esc" in line.lower():
            self.rx_queue.put(("ESC_WARNING", line))
            return

        if not line.startswith('#'):
            # ignoramos líneas que no son nuestras tramas
            return

        payload = line[1:]
        partes = [p.strip() for p in payload.split(',')]
        if len(partes) != 3:
            return

        try:
            ang_deg, err_deg, pwm_hw = map(float, partes)
        except ValueError:
            return

        # PWM software (lo calcula el objeto contenedor mediante lambda)
        pwm_sw = self._calc_pwm_sw(ang_deg)
        self.rx_queue.put((ang_deg, err_deg, pwm_hw, pwm_sw))


# ════════════════════════════════════════════════════════════════════
# 2)  SerialComm – interfaz de alto nivel para tu aplicación
# ════════════════════════════════════════════════════════════════════
class SerialComm:
    """
    Comunicación serie no-bloqueante con ReaderThread + LineReader.

    • Publica tuplas:
        (ang_deg, err_deg, pwm_hw, pwm_sw)   ← datos de telemetría
      y mensajes especiales:
        ("ESC_WARNING", txt)                 ← banner de calibración ESC
    • Mantiene un pequeño PIDf en SW para producir pwm_sw idéntico
      al del Arduino (útil para graficar / debug).
    """

    # ---------------------------------------------------------------
    def __init__(self, port='COM5', baud=9600, simulate=False):
        self.port = port
        self.baud = baud
        self.simulate = simulate                  # TRUE = genera datos fake
        self.rx_queue: queue.Queue = queue.Queue()

        self._reader_thread = None                # ReaderThread de pyserial
        self._protocol = None                     # instancia _LineProtocol
        self._simulate_th = None                  # hilo de simulación
        self.running = False

        # ------- coeficientes PIDf SW ---------------
        self._init_pidf_state()

    # ---------------------------------------------------------------
    #              ==  API esperado por tu GUI  ==
    # ---------------------------------------------------------------
    # .queue → alias retro-compatibilidad
    @property
    def queue(self):
        return self.rx_queue

    # ---------------  control del hilo RX --------------------------
    def start(self):
        if self.running:
            print("[WARN] SerialComm ya fue iniciado.")
            return

        self.running = True

        if self.simulate:
            self._simulate_th = threading.Thread(
                target=self._simulate_data, daemon=True)
            self._simulate_th.start()
            return

        # →  puerto real
        try:
            ser = serial.Serial(self.port, self.baud,
                                timeout=0.1,   # no bloquea GUI
                                write_timeout=0.2)
        except Exception as e:
            print(f"[SERIAL] Error abriendo {self.port}: {e}")
            self.rx_queue.put(("ERROR", str(e)))
            self.running = False
            return

        # ReaderThread administra su propio hilo; le pasamos nuestro protocolo
        self._reader_thread = serial.threaded.ReaderThread(
            ser,
            lambda: _LineProtocol(self.rx_queue,
                                  self._calcular_pwm_soft)
        )
        self._reader_thread.start()  # arranca hilo interno
        self._protocol = self._reader_thread.connect()[1]
        #  connect() → (serial_instance, protocolo)

    def stop(self):
        self.running = False

        if self._reader_thread is not None:
            self._reader_thread.close()
            self._reader_thread = None

        if self._simulate_th and self._simulate_th.is_alive():
            self._simulate_th.join(timeout=0.1)

    # ---------------  TX genérico ----------------------------------
    def send_command(self, msg: str):
        """
        Envía una línea cr+lf al Arduino.
        Si estamos en modo simulación se ignora.
        """
        if self.simulate or self._protocol is None:
            return
        try:
            self._protocol.transport.write((msg.strip() + '\n').encode())
        except Exception as e:
            print(f"[SERIAL] Error TX: {e}")

    # atajo específico de tu sketch
    def send_pidf_data(self, Tss, Mp, kp, ki, kd, n, pwm, toggle):
        datos = [Tss, Mp, kp, ki, kd, n, pwm, 1 if toggle else 0]
        partes = [
            "nan" if (isinstance(x, float) and np.isnan(x)) else f"{x:.4f}"
            if isinstance(x, (float, int)) else str(x)
            for x in datos
        ]
        self.send_command(",".join(partes))

    # ════════════════════════════════════════════════════════════════
    #                PIDf software  (idéntico a Arduino)
    # ══════════════════════════════════════════════════════­═══════
    def _init_pidf_state(self):
        self.a0 = self.a1 = self.a2 = 0.0
        self.a3 = 1.0
        self.a4 = self.a5 = 0.0
        self.e_km1 = self.e_km2 = 0.0
        self.u_km1 = self.u_km2 = 0.0
        self.pwm_eq = 1500.0
        self.ref = 0.0

    def set_pidf(self, kp, ki, kd, n, Ts, pwm_eq=1500.0):
        a = kp + kd * n
        b = kp * n + ki
        c = ki * n
        d = n

        K1 = (b * d - c) / d**2
        K2 = c / d
        K3 = (a * d**2 - b * d + c) / d**2

        self.a0 = K1 + K3
        self.a1 = -K1 - K1*np.exp(-d*Ts) + K2*Ts - 2*K3
        self.a2 = K1*np.exp(-d*Ts) - K2*Ts*np.exp(-d*Ts) + K3
        self.a3 = 1.0
        self.a4 = -np.exp(-d*Ts) - 1.0
        self.a5 = np.exp(-d*Ts)
        self.pwm_eq = pwm_eq

    def set_referencia(self, ref_deg):
        self.ref = np.radians(ref_deg)

    def _calcular_pwm_soft(self, ang_deg):
        ang_rad = np.radians(ang_deg)
        error   = self.ref - ang_rad

        u = (1.0/self.a3) * (
            self.a0*error + self.a1*self.e_km1 + self.a2*self.e_km2
            - self.a4*self.u_km1 - self.a5*self.u_km2
        )
        pwm = np.clip(u + self.pwm_eq, 1000, 2000)

        # roll de estados
        self.e_km2, self.e_km1 = self.e_km1, error
        self.u_km2, self.u_km1 = self.u_km1, u
        return pwm

    # ════════════════════════════════════════════════════════════════
    #               GENERADOR SIMULADO (opcional)
    # ══════════════════════════════════════════════════════­═══════
    def _simulate_data(self):
        import random
        dt    = 0.02
        y     = -45.0
        dy    = 0.0
        ref   = 15.0
        wn, zeta = 2.0, 0.6

        while self.running:
            err = ref - y
            ddy = wn**2*err - 2*zeta*wn*dy
            dy += ddy*dt
            y  += dy*dt

            pwm_hw = self.pwm_eq + 12*err + random.uniform(-3, 3)
            pwm_hw = np.clip(pwm_hw, 1000, 2000)

            self.rx_queue.put((y, err, pwm_hw, pwm_hw))
            time.sleep(dt)


# ===================================================================
#        helpers para salvar / cargar configuraciones en JSON
# ===================================================================
def save_config(path: str, cfg: dict):
    with open(path, 'w') as f:
        json.dump(cfg, f, indent=2)


def load_config(path: str) -> dict:
    with open(path) as f:
        return json.load(f)
