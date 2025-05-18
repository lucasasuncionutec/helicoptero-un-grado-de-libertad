from PySide6.QtWidgets import QWidget, QVBoxLayout
from PySide6.QtCore import Signal
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

class PZChartMatplotlib(QWidget):
    """Mapa P‑Z interactivo con zoom y pan."""
    sigChanged = Signal(list, list)  # (poles, zeros)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.fig = Figure(figsize=(4, 4))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Real")
        self.ax.set_ylabel("Imaginario")
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.axhline(0, color='k', lw=0.5)
        self.ax.axvline(0, color='k', lw=0.5)

        self.canvas = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.canvas, self)

        layout = QVBoxLayout(self)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)

        # Datos internos
        self.poles, self.zeros = [], []
        self.sel = None  # ('p' | 'z', idx)

        # Artistas
        self.line_p, = self.ax.plot([], [], 'x', color='crimson',
                                    markersize=10, markeredgewidth=2, picker=5, label='Polos')
        self.line_z, = self.ax.plot([], [], 'o', mfc='none', mec='royalblue',
                                    markersize=9, markeredgewidth=1.8, picker=5, label='Ceros')
        self.ax.legend(loc='upper right', fontsize=9)

        # Conexiones
        self.canvas.mpl_connect('pick_event', self._pick)
        self.canvas.mpl_connect('motion_notify_event', self._drag)
        self.canvas.mpl_connect('button_release_event', self._release)

    def set_poles_zeros(self, poles, zeros):
        self.poles = list(poles)
        self.zeros = list(zeros)
        self._draw()

    def _draw(self):
        self.line_p.set_data([p.real for p in self.poles],
                             [p.imag for p in self.poles])
        self.line_z.set_data([z.real for z in self.zeros],
                             [z.imag for z in self.zeros])
        self.canvas.draw_idle()

    def _pick(self, ev):
        if ev.artist is self.line_p:
            self.sel = ('p', ev.ind[0])
        elif ev.artist is self.line_z:
            self.sel = ('z', ev.ind[0])

    def _drag(self, ev):
        if self.sel and ev.inaxes is self.ax:
            kind, idx = self.sel
            if kind == 'p':
                self.poles[idx] = complex(ev.xdata, ev.ydata)
            else:
                self.zeros[idx] = complex(ev.xdata, ev.ydata)
            self._draw()

    def _release(self, _ev):
        if self.sel:
            self.sel = None
            self.sigChanged.emit(self.poles[:], self.zeros[:])
