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
        import random
        while self.running:
            ts  = time.time()
            ang = random.uniform(-30, 30)
            err = random.uniform(-5, 5)
            pwm = random.uniform(1000, 2000)
            self.queue.put((ts, ang, err, pwm))
            time.sleep(0.001)

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
