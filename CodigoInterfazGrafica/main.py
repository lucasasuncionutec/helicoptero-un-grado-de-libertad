import sys, numpy as np
from functools import partial
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QComboBox, QLabel, QFileDialog, QDoubleSpinBox,
    QFormLayout, QGridLayout, QGroupBox, QFrame, QCheckBox
)
from PySide6.QtCore import Qt
from PySide6.QtCore import QTimer
from PySide6.QtGui import QPixmap

from io_utils import SerialComm, save_config, load_config
from control_utils import ControlSystem
from control import TransferFunction, feedback
from pz_charts_matplotlib import PZChartMatplotlib as PZChart
import pyqtgraph as pg
from control import step_response


class BlockDiagramCanvas(QLabel):
    """Etiqueta que muestra una imagen del diagrama de bloques según los modelos seleccionados."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(380, 128)  # escala proporcional a 1142x386 reducida a ~66%
        self.setScaledContents(True)
        self.update_image("ModeloMotor1", "ModeloMecanico1", "PIDf")

    def update_image(self, motor, mecanica, controlador):
        nombre = f"{motor}_{mecanica}_{controlador}".replace(" ", "_")
        ruta = f"diagramas/{nombre}.png"
        try:
            pixmap = QPixmap(ruta)
            if pixmap.isNull():
                raise FileNotFoundError
            self.setPixmap(pixmap)
        except:
            self.setText(f"[Imagen no encontrada]\n{ruta}")
            self.setStyleSheet("color: red; font-size: 10pt;")


class ControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Helicóptero 1‑DOF")
        self.resize(1200, 800)

        self.comm    = SerialComm(simulate=True)
        self.ctrlsys = ControlSystem()
        self.C_tf    = self.ctrlsys.pidf_tf(1, 0, 0, 10)

        central = QWidget(); self.setCentralWidget(central)
        hmain = QHBoxLayout(central)

        # === Panel izquierdo ===
        pnl = QWidget(); pnl.setFixedWidth(280)
        v = QVBoxLayout(pnl)

        v.addWidget(QLabel("Modelo Motor:"))
        self.cb_m = QComboBox(); self.cb_m.addItems(self.ctrlsys.motor_models.keys()); v.addWidget(self.cb_m)

        v.addWidget(QLabel("Modelo Mecánica:"))
        self.cb_e = QComboBox(); self.cb_e.addItems(self.ctrlsys.mech_models.keys()); v.addWidget(self.cb_e)

        v.addWidget(QLabel("Modo de Control:"))
        self.cb_c = QComboBox(); self.cb_c.addItems(["PIDf manual", "Asign polos"]); v.addWidget(self.cb_c)

        v.addWidget(QLabel("Ángulo de equilibrio (°):"))
        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(-90.0, 90.0)
        self.angle_spin.setSingleStep(1.0)
        self.angle_spin.setValue(0.0)
        self.angle_spin.valueChanged.connect(self._on_angle_changed)
        v.addWidget(self.angle_spin)

        # === Campo para ángulo de referencia ===
        ref_layout = QHBoxLayout()
        self.ref_label = QLabel("Ángulo de referencia (°):")
        self.ref_spin = QDoubleSpinBox()
        self.ref_spin.setRange(-90.0, 90.0)
        self.ref_spin.setSingleStep(1.0)
        self.ref_spin.setValue(0.0)
        self.ref_spin.valueChanged.connect(self._on_reference_changed)

        self.cb_same_as_eq = QCheckBox("Usar mismo ángulo")
        self.cb_same_as_eq.setChecked(False)
        self.cb_same_as_eq.toggled.connect(self._on_checkbox_toggled)

        ref_layout.addWidget(self.ref_label)
        ref_layout.addWidget(self.ref_spin)

        v.addLayout(ref_layout)
        v.addWidget(self.cb_same_as_eq)


        # Intervalo de simulación (respuesta al escalón)
        v.addWidget(QLabel("Tiempo de simulación (s):"))
        self.step_time_spin = QDoubleSpinBox()
        self.step_time_spin.setRange(1.0, 20.0)
        self.step_time_spin.setSingleStep(0.5)
        self.step_time_spin.setValue(5.0)
        v.addWidget(self.step_time_spin)

        btn_step = QPushButton("Recalcular respuesta")
        btn_step.clicked.connect(self._recalcular_step)
        v.addWidget(btn_step)


        # Panel de edición PIDf
        v.addWidget(QLabel("Controlador PIDf:"))

        self.spin_kp = QDoubleSpinBox()
        self.spin_kp.setDecimals(4)
        self.spin_kp.setMaximum(10000.0)
        self.spin_kp.setValue(5.9067)

        self.spin_ki = QDoubleSpinBox()
        self.spin_ki.setDecimals(4)
        self.spin_ki.setMaximum(10000.0)
        self.spin_ki.setValue(0.2765)

        self.spin_kd = QDoubleSpinBox()
        self.spin_kd.setDecimals(4)
        self.spin_kd.setMaximum(10000.0)
        self.spin_kd.setValue(30.9819)

        self.spin_n = QDoubleSpinBox()
        self.spin_n.setDecimals(2)
        self.spin_n.setMaximum(1000.0)
        self.spin_n.setValue(5.4539)


        form_pid = QFormLayout()
        form_pid.addRow("Kp:", self.spin_kp)
        form_pid.addRow("Ki:", self.spin_ki)
        form_pid.addRow("Kd:", self.spin_kd)
        form_pid.addRow("N:",  self.spin_n)
        v.addLayout(form_pid)

        btn_apply_pid = QPushButton("Aplicar PID")
        btn_apply_pid.clicked.connect(self._actualizar_pid_manual)
        v.addWidget(btn_apply_pid)

        btn_poles = QPushButton("Configurar Tss & Mp"); btn_poles.clicked.connect(self._edit_poles); v.addWidget(btn_poles)

        hrun = QHBoxLayout()
        for lab, slot in [("Start", partial(self._run, True)), ("Pause", self._pause), ("Stop", partial(self._run, False))]:
            b = QPushButton(lab); b.clicked.connect(slot); hrun.addWidget(b)
        v.addLayout(hrun)

        bsave = QPushButton("Guardar configuración"); bsave.clicked.connect(self._save); v.addWidget(bsave)
        bload = QPushButton("Cargar configuración");  bload.clicked.connect(self._load); v.addWidget(bload)

        from PySide6.QtCore import Qt  # Asegurate de tener esta línea importada arriba

        # === Constantes del modelo + PID actual en 2 columnas ===
        self.lbl_A  = QLabel("—")
        self.lbl_B  = QLabel("—")
        self.lbl_m  = QLabel("—")
        self.lbl_kp = QLabel("—")
        self.lbl_ki = QLabel("—")
        self.lbl_kd = QLabel("—")
        self.lbl_n  = QLabel("—")

        grid_abm_kpid = QGridLayout()

        # Títulos de cada columna
        titulo_modelo = QLabel("Constantes")
        titulo_pid    = QLabel("PIDf actual")

        titulo_modelo.setAlignment(Qt.AlignCenter)
        titulo_pid.setAlignment(Qt.AlignCenter)

        # Primera fila: títulos
        grid_abm_kpid.addWidget(titulo_modelo, 0, 0, 1, 2)
        grid_abm_kpid.addWidget(titulo_pid,    0, 3, 1, 2)

        # Segunda a quinta fila: datos
        grid_abm_kpid.addWidget(QLabel("A:"),  1, 0)
        grid_abm_kpid.addWidget(self.lbl_A,    1, 1)
        grid_abm_kpid.addWidget(QLabel("B:"),  2, 0)
        grid_abm_kpid.addWidget(self.lbl_B,    2, 1)
        grid_abm_kpid.addWidget(QLabel("m:"),  3, 0)
        grid_abm_kpid.addWidget(self.lbl_m,    3, 1)

        # Separador vertical
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)
        grid_abm_kpid.addWidget(line, 1, 2, 4, 1)

        # Columna derecha: PID actual
        grid_abm_kpid.addWidget(QLabel("Kp:"), 1, 3)
        grid_abm_kpid.addWidget(self.lbl_kp,   1, 4)
        grid_abm_kpid.addWidget(QLabel("Ki:"), 2, 3)
        grid_abm_kpid.addWidget(self.lbl_ki,   2, 4)
        grid_abm_kpid.addWidget(QLabel("Kd:"), 3, 3)
        grid_abm_kpid.addWidget(self.lbl_kd,   3, 4)
        grid_abm_kpid.addWidget(QLabel("N:"),  4, 3)
        grid_abm_kpid.addWidget(self.lbl_n,    4, 4)

        group_const = QGroupBox()
        group_const.setTitle("")
        group_const.setLayout(grid_abm_kpid)
        v.addWidget(group_const)



        v.addStretch()
        hmain.addWidget(pnl)

        # === Panel derecho ===
        gpanel = QWidget(); gv = QVBoxLayout(gpanel)

        self.sig_plot = pg.PlotWidget(title="Señales (Ángulo, Error, PWM)")
        self.sig_plot.addLegend()
        self.curve_ang = self.sig_plot.plot(pen='y', name='Ángulo')
        self.curve_err = self.sig_plot.plot(pen='r', name='Error')
        self.curve_pwm = self.sig_plot.plot(pen='g', name='PWM')
        gv.addWidget(self.sig_plot)

        # === Gráfica de respuesta al escalón ===
        self.step_plot = pg.PlotWidget(title="Respuesta al escalón (lazo cerrado)")
        self.step_plot.setLabel("bottom", "Tiempo [s]")
        self.step_plot.setLabel("left", "Salida")
        self.step_plot_curve = self.step_plot.plot(pen='c')
        gv.addWidget(self.step_plot)


        # === Botones para selección de señales ===
        self.display_options = {
            "Ángulo": True,
            "Error": True,
            "PWM": True,
        }
        self.display_buttons = {}

        signal_selector = QHBoxLayout()
        for label in self.display_options:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setChecked(True)
            btn.clicked.connect(self._update_plot_visibility)
            self.display_buttons[label] = btn
            signal_selector.addWidget(btn)
        gv.addLayout(signal_selector)



        # === Fila inferior con imagen + PZChart ===
        bottom_widget = QWidget()
        hbottom = QHBoxLayout(bottom_widget)
        hbottom.setContentsMargins(0, 0, 0, 0)

        self.diagram_canvas = BlockDiagramCanvas()
        self.diagram_canvas.setFixedSize(360*1.2, 128*1.2)  # Forzar tamaño imagen
        hbottom.addWidget(self.diagram_canvas, stretch=2)

        self.pz_chart = PZChart()
        self.pz_chart.sigChanged.connect(self._pz_moved)
        hbottom.addWidget(self.pz_chart, stretch=1)

        bottom_widget.setFixedHeight(320)  # Forzar altura total de la fila
        gv.addWidget(bottom_widget)

        hmain.addWidget(gpanel, stretch=1)

        self.pz_chart.set_poles_zeros(self.C_tf.poles(), self.C_tf.zeros())

        self.cb_m.currentIndexChanged.connect(lambda: self._update_diagram_labels())
        self.cb_e.currentIndexChanged.connect(lambda: self._update_diagram_labels())
        self.cb_c.currentIndexChanged.connect(lambda: self._update_diagram_labels())

        self.nombre_motor = {
            "Modelo estático 1": "ModeloMotor1"
        }
        self.nombre_mecanica = {
            "Modelo sin fricción": "ModeloMecanico1"
        }

        self.nombre_control = {
            "PIDf manual": "PIDf",
            "Asign polos": "ModeloAsignacionPolos1"
        }


        self.timer = QTimer(); self.timer.setInterval(1)
        self.timer.timeout.connect(self._update)
        self._buff = []
        self._actualizar_constantes_modelo()

        self.anguloReferencia = np.radians(self.ref_spin.value())


        
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


    def _run(self, start: bool):
        if start:  self.comm.start();  self.timer.start()
        else:      self.timer.stop();  self.comm.stop()

    def _pause(self): self.timer.setEnabled(not self.timer.isActive())

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
        for label, btn in self.display_buttons.items():
            self.display_options[label] = btn.isChecked()
        self._update(force=True)

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
        except ValueError:
            return  # evita crasheo si hay datos corruptos o vacíos

        if self.display_options.get("Ángulo", False):
            self.curve_ang.setData(ts, ang)
        else:
            self.curve_ang.clear()

        if self.display_options.get("Error", False):
            self.curve_err.setData(ts, er)
        else:
            self.curve_err.clear()

        if self.display_options.get("PWM", False):
            self.curve_pwm.setData(ts, pwm)
        else:
            self.curve_pwm.clear()



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

            t = np.linspace(0, t_final, 1000)
            t, y = step_response(T, T=t)
            self.step_plot_curve.setData(t, y)

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
