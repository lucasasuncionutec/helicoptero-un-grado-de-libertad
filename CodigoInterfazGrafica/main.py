import sys, numpy as np
from functools import partial
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QComboBox, QLabel, QFileDialog, QDoubleSpinBox,
    QFormLayout, QGridLayout, QGroupBox, QCheckBox, QTabWidget
)
from PySide6.QtCore import QTimer
from PySide6.QtGui import QPixmap

from io_utils import SerialComm, save_config, load_config
from control_utils import ControlSystem
from control import TransferFunction, feedback
from pz_charts_matplotlib import PZChartMatplotlib as PZChart
import pyqtgraph as pg
from control import step_response
from control import forced_response

class BlockDiagramCanvas(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(380, 128)
        self.setScaledContents(True)
        self.update_image("ModeloMotor1", "ModeloMecanico1", "PIDf")

    def update_image(self, motor, mecanica, controlador):
        nombre = f"{motor}_{mecanica}_{controlador}".replace(" ", "_")
        ruta = f"diagramas/{nombre}.png"
        pixmap = QPixmap(ruta)
        if pixmap.isNull():
            self.setText(f"[Imagen no encontrada]\n{ruta}")
            self.setStyleSheet("color: red; font-size: 10pt;")
        else:
            self.setPixmap(pixmap)



class ControlApp(QMainWindow):   
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Helicóptero 1‑DOF")
        self.resize(1200, 800)

        self.comm = SerialComm(simulate=True)
        self.ctrlsys = ControlSystem()
        self.C_tf = self.ctrlsys.pidf_tf(1, 0, 0, 10)

        central = QWidget()
        self.setCentralWidget(central)
        hmain = QHBoxLayout(central)

        # === Panel izquierdo con pestañas ===
        tabs = QTabWidget()
        tabs.setMinimumWidth(310)

        self.tab_modelo = QWidget()
        self.tab_simulacion = QWidget()
        tabs.addTab(self.tab_modelo, "Modelo")
        tabs.addTab(self.tab_simulacion, "Simulación")

        self.setup_tab_modelo()
        self.setup_tab_simulacion()


        hmain.addWidget(tabs)

        # === Panel derecho ===
        gpanel = QWidget()
        gv = QVBoxLayout(gpanel)
        self.gv_layout = gv


        self.plot_angle = pg.PlotWidget(title="Ángulo [°]")
        self.plot_error = pg.PlotWidget(title="Error [°]")
        self.plot_pwm   = pg.PlotWidget(title="PWM aplicado")

        # Ángulo
        self.plot_angle.setLabel("bottom", "Tiempo [s]")
        self.plot_angle.setLabel("left", "Ángulo [°]")
        self.curve_angle_real = self.plot_angle.plot(pen='y', name='Ángulo (real)')
        self.curve_angle_sim  = self.plot_angle.plot(pen='c', name='Ángulo (sim)')

        # Error
        self.plot_error.setLabel("bottom", "Tiempo [s]")
        self.plot_error.setLabel("left", "Error [°]")
        self.curve_error_real = self.plot_error.plot(pen='r', name='Error (real)')
        self.curve_error_sim  = self.plot_error.plot(pen='m', name='Error (sim)')

        # PWM
        self.plot_pwm.setLabel("bottom", "Tiempo [s]")
        self.plot_pwm.setLabel("left", "PWM aplicado")
        self.curve_pwm_real = self.plot_pwm.plot(pen='g', name='PWM (real)')
        self.curve_pwm_sim  = self.plot_pwm.plot(pen='b', name='PWM (sim)')

        gv.addWidget(self.plot_angle)
        gv.addWidget(self.plot_error)
        gv.addWidget(self.plot_pwm)
        self.display_options_sim = {
            "Ángulo (sim)": True,
            "Error (sim)": True,
            "PWM aplicado (sim)": True,
        }


        self.display_flags = {
            "angle_real": True, "angle_sim": True,
            "error_real": True, "error_sim": True,
            "pwm_real": True,   "pwm_sim": True,
        }
        self.add_toggle_buttons("Ángulo", "angle_real", "angle_sim")
        self.add_toggle_buttons("Error", "error_real", "error_sim")
        self.add_toggle_buttons("PWM", "pwm_real", "pwm_sim")

        # === Fila inferior con imagen + PZChart ===
        bottom_widget = QWidget()
        hbottom = QHBoxLayout(bottom_widget)
        hbottom.setContentsMargins(0, 0, 0, 0)

        self.diagram_canvas = BlockDiagramCanvas()
        self.diagram_canvas.setFixedSize(360*1.2, 128*1.2)
        hbottom.addWidget(self.diagram_canvas, stretch=2)

        self.pz_chart = PZChart()
        self.pz_chart.sigChanged.connect(self._pz_moved)
        hbottom.addWidget(self.pz_chart, stretch=1)

        bottom_widget.setFixedHeight(280)
        gv.addWidget(bottom_widget)
        hmain.addWidget(gpanel, stretch=1)

        # === Final setup ===
        self.pz_chart.set_poles_zeros(self.C_tf.poles(), self.C_tf.zeros())
        self.timer = QTimer(); self.timer.setInterval(1)
        self.timer.timeout.connect(self._update)
        self._buff = []

        self.cb_m.currentIndexChanged.connect(lambda: self._update_diagram_labels())
        self.cb_e.currentIndexChanged.connect(lambda: self._update_diagram_labels())
        self.cb_c.currentIndexChanged.connect(lambda: self._update_diagram_labels())

        self.nombre_motor = {"Modelo estático 1": "ModeloMotor1"}
        self.nombre_mecanica = {"Modelo sin fricción": "ModeloMecanico1"}
        self.nombre_control = {"PIDf manual": "PIDf", "Asign polos": "ModeloAsignacionPolos1"}

        self.anguloReferencia = np.radians(self.ref_spin.value())
        self._actualizar_constantes_modelo()

        self.sim_data = {"t": [], "y": [], "error": [], "pwm": []}
        self.real_data = {"t_ang": ([],), "angle": [], "error": [], "pwm": []}

    def add_toggle_buttons(self, label, key_real, key_sim):
        row = QHBoxLayout()
        btn_real = QPushButton(f"{label} (real)")
        btn_sim  = QPushButton(f"{label} (sim)")
        btn_real.setCheckable(True); btn_sim.setCheckable(True)
        btn_real.setChecked(True);   btn_sim.setChecked(True)

        def toggle_real(): 
            self.display_flags[key_real] = btn_real.isChecked()
            self._update_plot_visibility()

        def toggle_sim():  
            self.display_flags[key_sim] = btn_sim.isChecked()
            self._update_plot_visibility()

        btn_real.clicked.connect(toggle_real)
        btn_sim.clicked.connect(toggle_sim)

        row.addWidget(btn_real); row.addWidget(btn_sim)
        self.gv_layout.addLayout(row)



    def setup_tab_modelo(self):
        layout = QVBoxLayout(self.tab_modelo)

        # === Selección de modelos ===
        grid_modelos = QGridLayout()
        grid_modelos.addWidget(QLabel("Motor:"),     0, 0)
        self.cb_m = QComboBox()
        self.cb_m.addItems(self.ctrlsys.motor_models.keys())
        grid_modelos.addWidget(self.cb_m, 0, 1)

        grid_modelos.addWidget(QLabel("Mecánica:"),  1, 0)
        self.cb_e = QComboBox()
        self.cb_e.addItems(self.ctrlsys.mech_models.keys())
        grid_modelos.addWidget(self.cb_e, 1, 1)

        grid_modelos.addWidget(QLabel("Control:"),   2, 0)
        self.cb_c = QComboBox()
        self.cb_c.addItems(["PIDf manual", "Asign polos"])
        grid_modelos.addWidget(self.cb_c, 2, 1)

        layout.addLayout(grid_modelos)

        # === Parámetros del sistema ===
        layout.addSpacing(10)
        layout.addWidget(QLabel("Ángulo de equilibrio (°):"))
        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(-90.0, 90.0)
        self.angle_spin.setSingleStep(1.0)
        self.angle_spin.setValue(0.0)
        self.angle_spin.valueChanged.connect(self._on_angle_changed)
        layout.addWidget(self.angle_spin)

        layout.addWidget(QLabel("Ángulo de referencia (°):"))
        self.ref_spin = QDoubleSpinBox()
        self.ref_spin.setRange(-90.0, 90.0)
        self.ref_spin.setSingleStep(1.0)
        self.ref_spin.setValue(0.0)
        self.ref_spin.valueChanged.connect(self._on_reference_changed)
        layout.addWidget(self.ref_spin)

        self.cb_same_as_eq = QCheckBox("Usar mismo ángulo que equilibrio")
        self.cb_same_as_eq.setChecked(False)
        self.cb_same_as_eq.toggled.connect(self._on_checkbox_toggled)
        layout.addWidget(self.cb_same_as_eq)

        layout.addWidget(QLabel("Ángulo inicial (°):"))
        self.init_spin = QDoubleSpinBox()
        self.init_spin.setRange(-90.0, 90.0)
        self.init_spin.setSingleStep(1.0)
        self.init_spin.setValue(-50.0)  # valor por defecto
        layout.addWidget(self.init_spin)


        # === Controlador PIDf ===
        layout.addSpacing(10)
        layout.addWidget(QLabel("Controlador PIDf:"))
        self.spin_kp = QDoubleSpinBox(); self.spin_kp.setDecimals(4); self.spin_kp.setMaximum(10000); self.spin_kp.setValue(5.9067)
        self.spin_ki = QDoubleSpinBox(); self.spin_ki.setDecimals(4); self.spin_ki.setMaximum(10000); self.spin_ki.setValue(0.2765)
        self.spin_kd = QDoubleSpinBox(); self.spin_kd.setDecimals(4); self.spin_kd.setMaximum(10000); self.spin_kd.setValue(30.9819)
        self.spin_n  = QDoubleSpinBox(); self.spin_n.setDecimals(2);  self.spin_n.setMaximum(1000);  self.spin_n.setValue(5.4539)

        form_pid = QFormLayout()
        form_pid.addRow("Kp:", self.spin_kp)
        form_pid.addRow("Ki:", self.spin_ki)
        form_pid.addRow("Kd:", self.spin_kd)
        form_pid.addRow("N:",  self.spin_n)
        layout.addLayout(form_pid)

        btn_apply_pid = QPushButton("Aplicar PID")
        btn_apply_pid.clicked.connect(self._actualizar_pid_manual)
        layout.addWidget(btn_apply_pid)

        btn_poles = QPushButton("Configurar Tss & Mp")
        btn_poles.clicked.connect(self._edit_poles)
        layout.addWidget(btn_poles)

        layout.addStretch()

    def setup_tab_simulacion(self):
        layout = QVBoxLayout(self.tab_simulacion)

        layout.addWidget(QLabel("Tiempo de simulación (s):"))
        self.step_time_spin = QDoubleSpinBox()
        self.step_time_spin.setRange(1.0, 120.0)
        self.step_time_spin.setSingleStep(0.5)
        self.step_time_spin.setValue(5.0)
        layout.addWidget(self.step_time_spin)

        btn_step = QPushButton("Recalcular respuesta")
        btn_step.clicked.connect(self._recalcular_step)
        layout.addWidget(btn_step)

        layout.addSpacing(10)

        hrun = QHBoxLayout()

        # Botón Iniciar
        btn_start = QPushButton("⏵")
        btn_start.setStyleSheet("background-color: #4CAF50; font-size: 14pt; font-weight: bold;")
        btn_start.clicked.connect(partial(self._run, True))
        hrun.addWidget(btn_start)

        # Botón Reset
        btn_reset = QPushButton("🔄")
        btn_reset.setStyleSheet("background-color: #2196F3; font-size: 14pt; font-weight: bold;")
        btn_reset.clicked.connect(self._reset_real_data)
        hrun.addWidget(btn_reset)

        # Botón Parar
        btn_stop = QPushButton("⏹")
        btn_stop.setStyleSheet("background-color: #F44336; font-size: 14pt; font-weight: bold;")
        btn_stop.clicked.connect(partial(self._run, False))
        hrun.addWidget(btn_stop)

        layout.addLayout(hrun)


        layout.addSpacing(10)

        bsave = QPushButton("Guardar configuración")
        bsave.clicked.connect(self._save)
        layout.addWidget(bsave)

        bload = QPushButton("Cargar configuración")
        bload.clicked.connect(self._load)
        layout.addWidget(bload)

        layout.addSpacing(10)

        # === Modelo actual + PIDf ===
        self.lbl_A  = QLabel("—")
        self.lbl_B  = QLabel("—")
        self.lbl_m  = QLabel("—")
        self.lbl_kp = QLabel("—")
        self.lbl_ki = QLabel("—")
        self.lbl_kd = QLabel("—")
        self.lbl_n  = QLabel("—")

        grid = QGridLayout()
        grid.addWidget(QLabel("A:"), 0, 0); grid.addWidget(self.lbl_A, 0, 1)
        grid.addWidget(QLabel("B:"), 1, 0); grid.addWidget(self.lbl_B, 1, 1)
        grid.addWidget(QLabel("m:"), 2, 0); grid.addWidget(self.lbl_m, 2, 1)
        grid.addWidget(QLabel("Kp:"), 0, 2); grid.addWidget(self.lbl_kp, 0, 3)
        grid.addWidget(QLabel("Ki:"), 1, 2); grid.addWidget(self.lbl_ki, 1, 3)
        grid.addWidget(QLabel("Kd:"), 2, 2); grid.addWidget(self.lbl_kd, 2, 3)
        grid.addWidget(QLabel("N:"),  3, 2); grid.addWidget(self.lbl_n,  3, 3)

        box = QGroupBox("Modelo actual y PIDf")
        box.setLayout(grid)
        layout.addWidget(box)

        layout.addStretch()

        
    def _actualizar_constantes_modelo(self):
        A = self.ctrlsys.last_A
        B = self.ctrlsys.last_B
        m = self.ctrlsys.m
        self.lbl_A.setText(f"{A:.5f}")
        self.lbl_B.setText(f"{B:.5f}")
        self.lbl_m.setText(f"{m:.6f}")
        self.lbl_kp.setText(f"{self.spin_kp.value():.4f}")
        self.lbl_ki.setText(f"{self.spin_ki.value():.4f}")
        self.lbl_kd.setText(f"{self.spin_kd.value():.4f}")
        self.lbl_n.setText(f"{self.spin_n.value():.2f}")



    def _update_diagram_labels(self):
        motor_txt = self.cb_m.currentText()
        meca_txt  = self.cb_e.currentText()
        ctrl_txt  = self.cb_c.currentText()

        motor      = self.nombre_motor.get(motor_txt, motor_txt)
        mecanica   = self.nombre_mecanica.get(meca_txt, meca_txt)
        controlador = self.nombre_control.get(ctrl_txt, ctrl_txt)

        self.diagram_canvas.update_image(motor, mecanica, controlador)
        self._actualizar_constantes_modelo()

    def _update_sim_plot_visibility(self):
        if self.display_flags["angle_sim"]:
            self.curve_angle_sim.setData(self.sim_data["t"], np.degrees(self.sim_data["y"]))
        else:
            self.curve_angle_sim.clear()

        if self.display_flags["error_sim"]:
            self.curve_error_sim.setData(self.sim_data["t"], np.degrees(self.sim_data["error"]))
        else:
            self.curve_error_sim.clear()

        if self.display_flags["pwm_sim"]:
            self.curve_pwm_sim.setData(self.sim_data["t"], self.sim_data["pwm"])
        else:
            self.curve_pwm_sim.clear()


    def _run(self, start: bool):
        if start:  self.comm.start();  self.timer.start()
        else:      self.timer.stop();  self.comm.stop()

    def _pause(self): self.timer.setEnabled(not self.timer.isActive())

    def _reset_real_data(self):
        self._buff.clear()
        self.real_data = {
            "t_ang": ([],),
            "angle": [],
            "error": [],
            "pwm": [],
        }
        self._update_plot_visibility()
        print("[Datos reales] Reiniciados")

    def _edit_pid(self):
        self.C_tf = self.ctrlsys.pidf_tf(2.0, 1.0, 0.5, 10.0)
        self.pz_chart.set_poles_zeros(self.C_tf.poles(), self.C_tf.zeros())

    def _edit_poles(self):
        Tss = 2.0
        Mp  = 0.2
        self.C_tf = self.ctrlsys.assignment_tf(Tss, Mp)
        self.pz_chart.set_poles_zeros(self.C_tf.poles(), self.C_tf.zeros())
        self._update_metrics(Tss, Mp)

    def _on_angle_changed(self, value_deg):
        self.ctrlsys.set_equilibrium_angle_deg(value_deg)
        self._update_pz()
        self._actualizar_constantes_modelo()

        # Si el checkbox está activado, sincronizar con la referencia
        if self.cb_same_as_eq.isChecked():
            self.ref_spin.blockSignals(True)  # Evita que dispare _on_reference_changed
            self.ref_spin.setValue(value_deg)
            self.ref_spin.blockSignals(False)
            self.anguloReferencia = np.radians(value_deg)
            print(f"[Referencia] Actualizada automáticamente a {value_deg}° por cambio en equilibrio")

    def _update_metrics(self, Tss, Mp):
        zeta = -np.log(Mp) / np.sqrt(np.pi**2 + (np.log(Mp))**2)
        wn   = 4.0 / (zeta * Tss)
        self.lbl_mp.setText(f"{Mp:.3f}")
        self.lbl_tss.setText(f"{Tss:.3f} s")
        self.lbl_zeta.setText(f"{zeta:.3f}")
        self.lbl_wn.setText(f"{wn:.3f} rad/s")

    def _save(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Guardar configuración", "", "JSON (*.json)")
        if not fn: return
        cfg = {
            "motor": self.cb_m.currentText(),
            "mech":  self.cb_e.currentText(),
            "ctrl":  self.cb_c.currentText(),
            "C_num": self.C_tf.num[0][0].tolist(),
            "C_den": self.C_tf.den[0][0].tolist(),
        }
        save_config(fn, cfg)

    def _load(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Cargar configuración", "", "JSON (*.json)")
        if not fn: return
        cfg = load_config(fn)
        self.cb_m.setCurrentText(cfg["motor"])
        self.cb_e.setCurrentText(cfg["mech"])
        self.cb_c.setCurrentText(cfg["ctrl"])
        self.C_tf = TransferFunction(cfg["C_num"], cfg["C_den"])
        self.pz_chart.set_poles_zeros(self.C_tf.poles(), self.C_tf.zeros())

    def _update_plot_visibility(self):
        # Real
        if self.display_flags["angle_real"]:
            self.curve_angle_real.setData(*self.real_data["t_ang"], self.real_data["angle"])
        else:
            self.curve_angle_real.clear()

        if self.display_flags["error_real"]:
            self.curve_error_real.setData(*self.real_data["t_ang"], self.real_data["error"])
        else:
            self.curve_error_real.clear()

        if self.display_flags["pwm_real"]:
            self.curve_pwm_real.setData(*self.real_data["t_ang"], self.real_data["pwm"])
        else:
            self.curve_pwm_real.clear()

        # Simulado
        if self.display_flags["angle_sim"]:
            self.curve_angle_sim.setData(self.sim_data["t"], np.degrees(self.sim_data["y"]))
        else:
            self.curve_angle_sim.clear()

        if self.display_flags["error_sim"]:
            self.curve_error_sim.setData(self.sim_data["t"], np.degrees(self.sim_data["error"]))
        else:
            self.curve_error_sim.clear()

        if self.display_flags["pwm_sim"]:
            self.curve_pwm_sim.setData(self.sim_data["t"], self.sim_data["pwm"])
        else:
            self.curve_pwm_sim.clear()


    def _update(self, force=False):
        while not self.comm.queue.empty():
            item = self.comm.queue.get()
            if isinstance(item, str):
                try:
                    item = list(map(float, item.split(',')))
                except:
                    continue
            if isinstance(item, tuple) or isinstance(item, list):
                if len(item) == 4:
                    self._buff.append(tuple(item))

        if not self._buff and not force:
            return

        try:
            ts, ang, er, pwm = zip(*self._buff[-1000:])
            self.real_data = {
                "t_ang": (ts,),  # la coma convierte ts en tupla (requerida por setData)
                "angle": ang,
                "error": er,
                "pwm": pwm,
            }
            self._update_plot_visibility()

        except ValueError:
            return  # evita crasheo si hay datos corruptos o vacíos




    def _update_pz(self):
        self.pz_chart.set_poles_zeros(self.C_tf.poles(), self.C_tf.zeros())

    def _pz_moved(self, poles, zeros):
        num = np.real_if_close(np.poly(zeros), tol=1e-9)
        den = np.real_if_close(np.poly(poles), tol=1e-9)
        self.C_tf = TransferFunction(num.tolist(), den.tolist())
        self._update_pz()

    def closeEvent(self, ev):
        self.comm.stop()
        super().closeEvent(ev)

    def _recalcular_step(self):
        try:
            motor_name = self.cb_m.currentText()
            mech_name  = self.cb_e.currentText()
            control_mode = self.cb_c.currentText()
            t_final = self.step_time_spin.value()

            print(f"\n[Simulación paso] Modo: {control_mode}")
            print(f"  - Modelo motor:    {motor_name}")
            print(f"  - Modelo mecánica: {mech_name}")
            print(f"  - Tiempo simulado: {t_final} s")

            # === ACTUALIZAR CONTROLADOR DESDE SPINBOX ===
            if control_mode == "PIDf manual":
                kp = self.spin_kp.value()
                ki = self.spin_ki.value()
                kd = self.spin_kd.value()
                n  = self.spin_n.value()
                self.C_tf = self.ctrlsys.pidf_tf(kp, ki, kd, n)
                print(f"  - Controlador PIDf actualizado: Kp={kp}, Ki={ki}, Kd={kd}, N={n}")

            # === Simular sistema cerrado ===
            Gm = self.ctrlsys.get_mech_tf(mech_name)
            Gp = self.ctrlsys.get_motor_tf(motor_name)
            C  = self.C_tf
            T  = feedback(C * Gp * Gm, 1)


            t_pre  = 60.0                         # duración de la etapa inicial
            t_real = self.step_time_spin.value()  # duración que el usuario pidió
            t_total = t_pre + t_real

            # Tiempo total de simulación
            t = np.linspace(0, t_total, 2000)

            # Primera parte: mantener el sistema en el ángulo inicial
            ref1 = np.radians(self.init_spin.value())
            ref2 = self.anguloReferencia

            u = np.piecewise(t, [t < t_pre, t >= t_pre], [ref1, ref2])

            # Simular todo
            t_sim, y = forced_response(T, T=t, U=u)

            # === Recorte de la etapa visible ===
            mask = t >= t_pre
            t_visible = t[mask] - t_pre
            y_visible = y[mask]

            # === Cálculo de error y PWM aplicado ===
            error_visible = ref2 - y_visible
            t_pwm, delta_pwm = forced_response(self.C_tf, T=t_visible, U=error_visible)


            pwm_eq = self.ctrlsys.pwm_equilibrio()
            pwm_visible = delta_pwm + pwm_eq
            pwm_visible = np.clip(pwm_visible, 1000, 2000)  # Saturación


            self.sim_data = {
                "t": t_visible,
                "y": y_visible,
                "error": error_visible,
                "pwm": pwm_visible,
            }

            self._update_sim_plot_visibility()


        except Exception as e:
            print("[ERROR simulación paso]:", e)

            
    def _actualizar_pid_manual(self):
        kp = self.spin_kp.value()
        ki = self.spin_ki.value()
        kd = self.spin_kd.value()
        n  = self.spin_n.value()

        print(f"[PID manual] Aplicando Kp={kp}, Ki={ki}, Kd={kd}, N={n}")
        self.C_tf = self.ctrlsys.pidf_tf(kp, ki, kd, n)
        self.pz_chart.set_poles_zeros(self.C_tf.poles(), self.C_tf.zeros())
        self._actualizar_constantes_modelo()

    def _on_reference_changed(self, value_deg):
        self.anguloReferencia = np.radians(value_deg)
        print(f"[Referencia] Ángulo de referencia actualizado a {value_deg}°")

    def _on_checkbox_toggled(self, checked):
        if checked:
            self.ref_spin.setEnabled(False)
            valor_eq = self.angle_spin.value()
            self.ref_spin.setValue(valor_eq)
            self.anguloReferencia = np.radians(valor_eq)
            print(f"[Referencia] Sincronizado con ángulo de equilibrio: {valor_eq}°")
        else:
            self.ref_spin.setEnabled(True)
            self._on_reference_changed(self.ref_spin.value())



if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ControlApp(); win.show()
    sys.exit(app.exec())
