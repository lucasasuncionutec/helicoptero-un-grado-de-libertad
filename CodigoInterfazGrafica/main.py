import sys, numpy as np
from functools import partial
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QComboBox, QLabel, QFileDialog, QDoubleSpinBox,
    QFormLayout, QGridLayout, QGroupBox, QCheckBox, QTabWidget, QStyle
)
from PySide6.QtCore import QTimer
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import QColorDialog
from pyqtgraph import mkColor
from PySide6.QtCore import Qt
import time
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

        self.comm = SerialComm(simulate=False)
        self.comm.start()

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
        self.tab_estilo = QWidget()  

        tabs.addTab(self.tab_modelo, "Modelo")
        tabs.addTab(self.tab_simulacion, "Simulación")
        tabs.addTab(self.tab_estilo, "Estilo")  

        self.setup_tab_modelo()
        self.anguloReferencia = np.radians(self.ref_spin.value())
        self.setup_tab_simulacion()
        self.setup_tab_estilo() 


        hmain.addWidget(tabs)

        # === Panel derecho ===
        gpanel = QWidget()
        gv = QVBoxLayout(gpanel)
        self.gv_layout = gv

        # ---------- Banner de advertencias que llegan por el puerto serie ----------
        self.lbl_serial_warning = QLabel("")
        self.lbl_serial_warning.setStyleSheet("""
            QLabel {
                background-color: #FFF3CD;
                color: #856404;
                border: 1px solid #FFE8A1;
                padding: 6px;
                border-radius: 4px;
                font-weight: bold;
            }
        """)
        self.lbl_serial_warning.setAlignment(Qt.AlignCenter)
        self.lbl_serial_warning.hide()           # comienza oculto
        gv.addWidget(self.lbl_serial_warning)    # lo ubicamos arriba de los plots


      # === Crear gráficas sin título inicialmente ===
        self.plot_angle = pg.PlotWidget()
        self.plot_error = pg.PlotWidget()
        self.plot_pwm   = pg.PlotWidget()

        # === Diccionario de títulos por gráfica ===
        self.titles = {
            self.plot_angle: "Ángulo [°]",
            self.plot_error: "Error [°]",
            self.plot_pwm:   "PWM aplicado",
        }

        # === Asignar título a cada gráfica ===
        for plot, title in self.titles.items():
            plot.setTitle(title)



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
        self._aplicar_estilo_curvas()

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


        # === Fila inferior con imagen + PZChart ===
        bottom_widget = QWidget()
        hbottom = QHBoxLayout(bottom_widget)
        hbottom.setContentsMargins(0, 0, 0, 0)

        self.diagram_canvas = BlockDiagramCanvas()
        self.diagram_canvas.setFixedSize(360*1.2, 128*1.35)
        hbottom.addWidget(self.diagram_canvas, stretch=2)

        self.pz_chart = PZChart()
        self.pz_chart.sigChanged.connect(self._pz_moved)
        hbottom.addWidget(self.pz_chart, stretch=1)

        bottom_widget.setFixedHeight(280)
        gv.addWidget(bottom_widget)
        hmain.addWidget(gpanel, stretch=1)

        # === Final setup ===
        self.pz_chart.set_poles_zeros([], [])


        self.timer = QTimer(); self.timer.setInterval(1)
        self.timer.timeout.connect(self._update)
        self._buff = []

        self.cb_m.currentIndexChanged.connect(lambda: self._update_diagram_labels())
        self.cb_e.currentIndexChanged.connect(lambda: self._update_diagram_labels())
        self.cb_c.currentIndexChanged.connect(lambda: self._update_diagram_labels())

        self.nombre_motor = {"Modelo estático 1": "ModeloMotor1"}
        self.nombre_mecanica = {"Modelo sin fricción": "ModeloMecanico1"}
        self.nombre_control = {"PIDf manual": "PIDf", "Asign polos": "ModeloAsignacionPolos1"}

        
        self._actualizar_constantes_modelo()

        self.sim_data = {"t": [], "y": [], "error": [], "pwm": []}
        self.real_data = {"t_ang": ([],), "angle": [], "error": [], "pwm": []}

    def add_toggle_buttons(self, label, key_real, key_sim, layout_destino):
        row = QHBoxLayout()
        btn_real = QPushButton()
        btn_sim  = QPushButton()
        btn_real.setCheckable(True); btn_sim.setCheckable(True)
        btn_real.setChecked(True);   btn_sim.setChecked(True)

        # === Colores por defecto para cada curva ===
        color_map = {
            "angle_real": (80, 180, 255),
            "angle_sim":  (80, 255, 180),
            "error_real": (255, 80, 120),
            "error_sim":  (255, 160, 80),
            "pwm_real":   (160, 120, 255),
            "pwm_sim":    (100, 200, 100),
        }

        def apply_style(button, rgb, checked, text_base):
            r, g, b = rgb
            icon = "👁️" if checked else "🚫"
            button.setText(f"{icon} {text_base}")
            button.setStyleSheet(
                f"""
                QPushButton {{
                    background-color: rgb({r},{g},{b});
                    border: 1px solid black;
                    font-weight: bold;
                }}
                QPushButton:checked {{
                    background-color: rgb({r},{g},{b});
                }}
                QPushButton:hover {{
                    background-color: rgb({r},{g},{b});
                }}
                """
            )

        def toggle_real():
            checked = btn_real.isChecked()
            self.display_flags[key_real] = checked
            apply_style(btn_real, color_map[key_real], checked, f"{label} (real)")
            self._update_plot_visibility()

        def toggle_sim():
            checked = btn_sim.isChecked()
            self.display_flags[key_sim] = checked
            apply_style(btn_sim, color_map[key_sim], checked, f"{label} (sim)")
            self._update_plot_visibility()

        # Aplicar estilos iniciales
        apply_style(btn_real, color_map[key_real], True, f"{label} (real)")
        apply_style(btn_sim,  color_map[key_sim],  True, f"{label} (sim)")

        # Conectar eventos
        btn_real.clicked.connect(toggle_real)
        btn_sim.clicked.connect(toggle_sim)

        row.addWidget(btn_real)
        row.addWidget(btn_sim)
        layout_destino.addLayout(row)




    def _aplicar_estilo_curvas(self):
        for nombre in self.color_curvas:
            color = self.color_curvas[nombre]
            width = self.ancho_curvas[nombre].value()
            pen = pg.mkPen(color=color, width=width)
            getattr(self, nombre).setPen(pen)
        print("[Estilo] Estilo de curvas actualizado")

    def setup_tab_estilo(self):
        layout = QVBoxLayout(self.tab_estilo)

        # === 1. Grupo: Fondo ===
        group_fondo = QGroupBox("Fondo del gráfico")
        fondo_layout = QVBoxLayout()

        # --- Botón para elegir color de fondo ---
        self.btn_bg_color = QPushButton("Seleccionar color de fondo")
        self.btn_bg_color.clicked.connect(self._elegir_color_fondo)
        fondo_layout.addWidget(self.btn_bg_color)

        # --- Botón para elegir color de ejes y texto ---
        self.color_texto = (255, 255, 255)  # valor por defecto: blanco
        self.btn_text_color = QPushButton("Seleccionar color de texto y ejes")
        self.btn_text_color.clicked.connect(self._elegir_color_texto)
        fondo_layout.addWidget(self.btn_text_color)



        group_fondo.setLayout(fondo_layout)
        layout.addWidget(group_fondo)

        # === 2. Grupo: Estilo de curvas ===
        group_estilo = QGroupBox("Estilo de curvas (color y grosor)")
        estilo_layout = QVBoxLayout()

        curvas = [
            ("Ángulo (real)", "curve_angle_real"),
            ("Ángulo (sim)", "curve_angle_sim"),
            ("Error (real)", "curve_error_real"),
            ("Error (sim)", "curve_error_sim"),
            ("PWM (real)", "curve_pwm_real"),
            ("PWM (sim)", "curve_pwm_sim"),
        ]

        self.color_curvas = {}
        self.ancho_curvas = {}
        self.btn_color_curvas = {}

        default_colors = {
            "curve_angle_real": (80, 180, 255),
            "curve_angle_sim":  (80, 255, 180),
            "curve_error_real": (255, 80, 120),
            "curve_error_sim":  (255, 160, 80),
            "curve_pwm_real":   (160, 120, 255),
            "curve_pwm_sim":    (100, 200, 100),
        }

        for label, attr in curvas:
            color_defecto = default_colors.get(attr, (255, 255, 255))
            self.color_curvas[attr] = color_defecto

            lbl = QLabel(label)
            btn_color = QPushButton("Color")
            btn_color.clicked.connect(lambda _, key=attr: self._elegir_color_curva(key))
            btn_color.setFixedWidth(70)

            spin_width = QDoubleSpinBox()
            spin_width.setRange(0.1, 5.0)
            spin_width.setSingleStep(0.1)
            spin_width.setValue(1.0)
            spin_width.setSuffix(" px")
            spin_width.setFixedWidth(70)

            self.ancho_curvas[attr] = spin_width
            self.btn_color_curvas[attr] = btn_color
            btn_color.setStyleSheet(f"background-color: rgb{color_defecto};")

            row = QHBoxLayout()
            row.addWidget(lbl, stretch=1)
            row.addWidget(btn_color)
            row.addWidget(spin_width)
            estilo_layout.addLayout(row)

        btn_aplicar_curvas = QPushButton("Aplicar estilo de curvas")
        btn_aplicar_curvas.clicked.connect(self._aplicar_estilo_curvas)
        estilo_layout.addWidget(btn_aplicar_curvas)

        group_estilo.setLayout(estilo_layout)
        layout.addWidget(group_estilo)

        # === 3. Grupo: Visibilidad de curvas ===
        group_visibility = QGroupBox("Curvas visibles")
        layout_visibility = QVBoxLayout()

        self.add_toggle_buttons("Ángulo", "angle_real", "angle_sim", layout_visibility)
        self.add_toggle_buttons("Error", "error_real", "error_sim", layout_visibility)
        self.add_toggle_buttons("PWM", "pwm_real", "pwm_sim", layout_visibility)

        group_visibility.setLayout(layout_visibility)
        layout.addWidget(group_visibility)

        layout.addStretch()


    def _elegir_color_fondo(self):
        color = QColorDialog.getColor()
        if color.isValid():
            self.color_fondo = (color.red(), color.green(), color.blue())
            self.btn_bg_color.setStyleSheet(f"background-color: rgb{self.color_fondo};")
            self._aplicar_estilo_fondo()


    def _elegir_color_curva(self, nombre):
        color = QColorDialog.getColor()
        if color.isValid():
            rgb = (color.red(), color.green(), color.blue())
            self.color_curvas[nombre] = rgb
            self.btn_color_curvas[nombre].setStyleSheet(f"background-color: rgb{rgb};")

            # 🔁 Si es una curva con botón de visibilidad, también actualizar su botón
            curva_to_key = {
                "curve_angle_real": ("angle_real", "Ángulo (real)"),
                "curve_angle_sim":  ("angle_sim",  "Ángulo (sim)"),
                "curve_error_real": ("error_real", "Error (real)"),
                "curve_error_sim":  ("error_sim",  "Error (sim)"),
                "curve_pwm_real":   ("pwm_real",   "PWM (real)"),
                "curve_pwm_sim":    ("pwm_sim",    "PWM (sim)"),
            }

            # Buscar si hay botón asociado
            if nombre in curva_to_key:
                key, label = curva_to_key[nombre]
                checked = self.display_flags.get(key, True)

                # Buscar el botón por referencia (opcional, si querés guardar los QPushButton)
                for layout in self.tab_estilo.findChildren(QHBoxLayout):
                    for i in range(layout.count()):
                        widget = layout.itemAt(i).widget()
                        if isinstance(widget, QPushButton) and label in widget.text():
                            # Actualizar estilo del botón
                            icon = "👁️" if checked else "🚫"
                            widget.setText(f"{icon} {label}")
                            widget.setStyleSheet(
                                f"""
                                QPushButton {{
                                    background-color: rgb{rgb};
                                    border: 1px solid black;
                                    font-weight: bold;
                                }}
                                QPushButton:checked {{
                                    background-color: rgb{rgb};
                                }}
                                QPushButton:hover {{
                                    background-color: rgb{rgb};
                                }}
                                """
                            )
                            return


    def _elegir_color_texto(self):
        color = QColorDialog.getColor()
        if color.isValid():
            self.color_texto = (color.red(), color.green(), color.blue())
            self.btn_text_color.setStyleSheet(f"background-color: rgb{self.color_texto};")
            self._aplicar_estilo_fondo()



    def _aplicar_estilo_fondo(self):
        r, g, b = self.color_fondo
        bg_color = mkColor(r, g, b)

        r_txt, g_txt, b_txt = self.color_texto
        text_color = (r_txt, g_txt, b_txt)
        text_qcolor = mkColor(*text_color)

        for plot in [self.plot_angle, self.plot_error, self.plot_pwm]:
            plot.setBackground(bg_color)

            # Cambiar el color del texto de los ejes
            for ax in ['bottom', 'left']:
                axis = plot.getAxis(ax)
                axis.setTextPen(text_qcolor)
                axis.setPen(text_qcolor)

            # Cambiar el título con color
            if plot in self.titles:
                plot.setTitle(self.titles[plot], color=text_color)


        print(f"[Estilo] Fondo: RGB({r},{g},{b}) | Texto: RGB{text_color}")



    def setup_tab_modelo(self):
        layout_principal = QVBoxLayout(self.tab_modelo)

        # === Grupo: Selección de modelos ===
        group_modelos = QGroupBox("Selección de modelos")
        grid_modelos = QGridLayout(group_modelos)

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

        layout_principal.addWidget(group_modelos)

        self.cb_m.currentIndexChanged.connect(self._actualizar_modelo_dinamico)
        self.cb_e.currentIndexChanged.connect(self._actualizar_modelo_dinamico)
        self.cb_c.currentIndexChanged.connect(self._actualizar_modelo_dinamico)

        # === Grupo: Parámetros del sistema ===
        group_param = QGroupBox("Parámetros del sistema")
        param_layout = QVBoxLayout(group_param)

        self.angle_spin = QDoubleSpinBox(); self.angle_spin.setRange(-90, 90); self.angle_spin.setSingleStep(1.0); self.angle_spin.setValue(0.0)
        self.angle_spin.valueChanged.connect(self._on_angle_changed)
        param_layout.addWidget(QLabel("Ángulo de equilibrio (°):"))
        param_layout.addWidget(self.angle_spin)

        self.ref_spin = QDoubleSpinBox(); self.ref_spin.setRange(-90, 90); self.ref_spin.setSingleStep(1.0); self.ref_spin.setValue(0.0)
        self.ref_spin.valueChanged.connect(self._on_reference_changed)
        param_layout.addWidget(QLabel("Ángulo de referencia (°):"))
        param_layout.addWidget(self.ref_spin)

        self.cb_same_as_eq = QCheckBox("Usar mismo ángulo que equilibrio")
        self.cb_same_as_eq.setChecked(False)
        self.cb_same_as_eq.toggled.connect(self._on_checkbox_toggled)
        param_layout.addWidget(self.cb_same_as_eq)

        self.init_spin = QDoubleSpinBox(); self.init_spin.setRange(-90, 90); self.init_spin.setSingleStep(1.0); self.init_spin.setValue(-50.0)
        param_layout.addWidget(QLabel("Ángulo inicial (°):"))
        param_layout.addWidget(self.init_spin)

        layout_principal.addWidget(group_param)

        # === Contenedor de widgets dinámicos ===
        self.model_dynamics_container = QVBoxLayout()
        layout_principal.addLayout(self.model_dynamics_container)

        # === Grupo: Controlador PIDf ===
        group_pid = QGroupBox("Controlador PIDf")
        pid_layout = QFormLayout(group_pid)

        self.spin_kp = QDoubleSpinBox(); self.spin_kp.setDecimals(4); self.spin_kp.setMaximum(10000); self.spin_kp.setValue(5.9067)
        self.spin_ki = QDoubleSpinBox(); self.spin_ki.setDecimals(4); self.spin_ki.setMaximum(10000); self.spin_ki.setValue(0.2765)
        self.spin_kd = QDoubleSpinBox(); self.spin_kd.setDecimals(4); self.spin_kd.setMaximum(10000); self.spin_kd.setValue(30.9819)
        self.spin_n  = QDoubleSpinBox(); self.spin_n.setDecimals(2);  self.spin_n.setMaximum(1000);  self.spin_n.setValue(5.4539)

        pid_layout.addRow("Kp:", self.spin_kp)
        pid_layout.addRow("Ki:", self.spin_ki)
        pid_layout.addRow("Kd:", self.spin_kd)
        pid_layout.addRow("N:",  self.spin_n)

        btn_apply_pid = QPushButton("Aplicar PID")
        btn_apply_pid.clicked.connect(self._actualizar_pid_manual)
        pid_layout.addRow(btn_apply_pid)

        btn_enviar = QPushButton("Enviar datos al Arduino")
        btn_enviar.clicked.connect(self._enviar_pid_a_arduino)
        pid_layout.addRow(btn_enviar)


        layout_principal.addWidget(group_pid)

        layout_principal.addStretch()

        # === Sección inferior: Guardar / Cargar configuración ===
        btns_bottom = QVBoxLayout()
        btns_bottom.setAlignment(Qt.AlignHCenter)

        icon_guardar = self.style().standardIcon(QStyle.SP_DialogSaveButton)
        icon_cargar  = self.style().standardIcon(QStyle.SP_DialogOpenButton)

        btn_save = QPushButton("Guardar configuración")
        btn_save.setIcon(icon_guardar)
        btn_save.setFixedWidth(220)
        btn_save.clicked.connect(self._save)
        btns_bottom.addWidget(btn_save)

        btn_load = QPushButton("Cargar configuración")
        btn_load.setIcon(icon_cargar)
        btn_load.setFixedWidth(220)
        btn_load.clicked.connect(self._load)
        btns_bottom.addWidget(btn_load)

        layout_principal.addLayout(btns_bottom)

        self._actualizar_modelo_dinamico()

    def _enviar_pid_a_arduino(self):
        kp = self.spin_kp.value()
        ki = self.spin_ki.value()
        kd = self.spin_kd.value()
        n  = self.spin_n.value()

        # Solo usar Tss y Mp si existen (solo en modo "Asign polos")
        tss = self.tss_spin.value() if hasattr(self, "tss_spin") else 0.0
        mp  = self.mp_spin.value()  if hasattr(self, "mp_spin") else 0.0

        # Solo enviar si hay al menos Kp, Ki, Kd, N
        if any([kp, ki, kd, n]):
            msg = f"{tss:.5f},{mp:.5f},{kp:.5f},{ki:.5f},{kd:.5f},{n:.5f}"
            self.comm.send_command(msg)
            print(f"[PC → Arduino] {msg}")
        else:
            print("[Advertencia] No se enviaron datos porque los parámetros están vacíos.")


    def _actualizar_modelo_dinamico(self):
        # Limpiar contenedor dinámico
        while self.model_dynamics_container.count():
            item = self.model_dynamics_container.takeAt(0)
            widget = item.widget()
            if widget: widget.setParent(None)

        tipo_control = self.cb_c.currentText()
        tipo_motor   = self.cb_m.currentText()
        tipo_meca    = self.cb_e.currentText()

        # === Ejemplo de lógica: mostrar botón solo con Asignación de Polos ===
        if tipo_control == "Asign polos":
            # Campos de entrada para Tss y Mp
            form_asignacion = QFormLayout()
            
            self.tss_spin = QDoubleSpinBox()
            self.tss_spin.setRange(0, 30.0)
            self.tss_spin.setSingleStep(0.1)
            self.tss_spin.setValue(2.0)

            self.mp_spin = QDoubleSpinBox()
            self.mp_spin.setRange(0.01, 2)
            self.mp_spin.setSingleStep(0.01)
            self.mp_spin.setValue(0.2)

            form_asignacion.addRow("Tss (s):", self.tss_spin)
            form_asignacion.addRow("Mp:", self.mp_spin)

            self.model_dynamics_container.addLayout(form_asignacion)

            # Mostrar zeta y omegan
            self.lbl_zeta = QLabel("ζ = —")
            self.lbl_wn   = QLabel("ωₙ = —")
            self.lbl_eqn  = QLabel("Fórmulas: ζ = -ln(Mp)/√(π² + ln²(Mp)) ; ωₙ = 4/(ζ·Tss)")
            self.lbl_eqn.setWordWrap(True)

            self.model_dynamics_container.addWidget(self.lbl_zeta)
            self.model_dynamics_container.addWidget(self.lbl_wn)
            self.model_dynamics_container.addWidget(self.lbl_eqn)

            # Botón para aplicar cálculo
            btn_calc = QPushButton("Calcular polos")
            btn_calc.clicked.connect(self._aplicar_asignacion_polos)
            self.model_dynamics_container.addWidget(btn_calc)


        # Aquí podrías agregar sliders, QLineEdit, más botones según los modelos elegidos
        # if tipo_motor == "XYZ" and tipo_meca == "ABC": ...

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

        self.lbl_poles = QLabel("Polos:\n—")
        self.lbl_zeros = QLabel("Ceros:\n—")        

        # Asegurar alineación monoespaciada
        self.lbl_poles.setStyleSheet("font-family: monospace;")
        self.lbl_zeros.setStyleSheet("font-family: monospace;")

        group_pz = QGroupBox("Polos y Ceros (Rectangular + Polar)")
        layout_pz = QVBoxLayout(group_pz)
        layout_pz.addWidget(self.lbl_poles)
        layout_pz.addWidget(self.lbl_zeros)

        layout.addWidget(group_pz)



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


    def _aplicar_asignacion_polos(self):
        try:
            Tss = self.tss_spin.value()
            Mp  = self.mp_spin.value()

            # Obtener función de transferencia y métricas
            C_tf, zeta, wn = self.ctrlsys.assignment_tf(Tss, Mp)

            # Asignar a atributos
            self.C_tf = C_tf

            # Actualizar las etiquetas
            self.lbl_zeta.setText(f"ζ = {zeta:.3f}")
            self.lbl_wn.setText(f"ωₙ = {wn:.3f} rad/s")

            # (Opcional) Imprimir en consola
            print(f"[Asignación de polos] Tss={Tss}, Mp={Mp}")
            print(f"  ζ={zeta:.3f}, ωₙ={wn:.3f}")
            print(f"  C(s) = {C_tf}")

        except Exception as e:
            print("[ERROR asignación de polos]:", e)

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
        if start:
            self.timer.start()  # solo empieza a graficar
            QTimer.singleShot(200, lambda: self._update(force=True))  # opcional, para forzar un primer update
        else:
            self.timer.stop()  # pausa la graficación, pero sigue conectado



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

    def _edit_poles(self):
        Tss = 2.0
        Mp  = 0.2
        self.C_tf = self.ctrlsys.assignment_tf(Tss, Mp)
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


    def _update(self, force: bool = False):
        """
        - Vacía la cola que llega desde `SerialComm`
        - Muestra/oculta la advertencia ESC
        - Convierte cada trama (ang, err, pwm) en puntos de 20 ms y los grafica
        """
        # ------------ 1. Procesar mensajes de la cola -------------
        while not self.comm.queue.empty():
            item = self.comm.queue.get()

            # ► Advertencia que viene como tupla ('ESC_WARNING', texto)
            if isinstance(item, tuple) and item[0] == "ESC_WARNING":
                self.lbl_serial_warning.setText(f"⚠️ {item[1]}")
                self.lbl_serial_warning.show()
                self._esc_last_received = time.time()
                continue

            # ► Trama numérica: (ángulo, error, pwm)
            if isinstance(item, (tuple, list)) and len(item) == 3:
                self._buff.append(tuple(item))
            # (opcional) silenciar cualquier otra cosa:
            # else:
            #     print(f"[SERIAL] Ignorado: {item!r}")

        # ------------ 2. Ocultar banner si expiró -----------------
        if hasattr(self, "_esc_last_received"):
            if time.time() - self._esc_last_received > 1.0:
                self.lbl_serial_warning.hide()

        # ------------ 3. Si no hay datos, salir -------------------
        if not self._buff and not force:
            return

        # ------------ 4. Actualizar curvas ------------------------
        try:
            ang, err, pwm = zip(*self._buff)         # desempaca
            ts = np.arange(len(ang)) * 0.02          # 20 ms → 50 Hz

            self.real_data = {
                "t_ang": (ts,),   # ← mantiene interfaz con _update_plot_visibility
                "angle": ang,
                "error": err,
                "pwm":   pwm,
            }
            self._update_plot_visibility()

        except Exception as e:
            print(f"[SERIAL] Error al graficar: {e}")


    def _pz_moved(self, poles, zeros):
        num = np.real_if_close(np.poly(zeros), tol=1e-9)
        den = np.real_if_close(np.poly(poles), tol=1e-9)
        self.C_tf = TransferFunction(num.tolist(), den.tolist())

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

            # Mostrar polos y ceros en consola
            poles = T.poles()
            zeros = T.zeros()

            print("Polos del sistema:", poles)
            print("Ceros del sistema:", zeros)

            # Mostrar en interfaz con ambas notaciones
            def format_complex(c):
                r = abs(c)
                a = np.angle(c)
                return f"{c.real:>7.3f} + {c.imag:>7.3f}j   | {r:>6.3f} ∠ {np.degrees(a):>6.2f}°"

            p_txt = "\n".join([f"• {format_complex(p)}" for p in poles]) or "—"
            z_txt = "\n".join([f"• {format_complex(z)}" for z in zeros]) or "—"

            self.lbl_poles.setText(f"Polos:\n{p_txt}")
            self.lbl_zeros.setText(f"Ceros:\n{z_txt}")



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
            self.pz_chart.set_poles_zeros(T.poles(), T.zeros())


        except Exception as e:
            print("[ERROR simulación paso]:", e)

            
    def _actualizar_pid_manual(self):
        kp = self.spin_kp.value()
        ki = self.spin_ki.value()
        kd = self.spin_kd.value()
        n  = self.spin_n.value()

        print(f"[PID manual] Aplicando Kp={kp}, Ki={ki}, Kd={kd}, N={n}")
        self.C_tf = self.ctrlsys.pidf_tf(kp, ki, kd, n)
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
