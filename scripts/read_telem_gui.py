import sys
import serial
import struct
import numpy as np
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg

# Configuration
PORT = "COM6"
BAUD = 921600
SYNC0 = 0x5A
SYNC1 = 0xA5

class SerialWorker(QtCore.QThread):
    data_received = QtCore.pyqtSignal(tuple)
    status_msg = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.running = True

    def run(self):
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
            self.ser.reset_input_buffer()
            while self.running:
                b = self.ser.read(1)
                if not b or b[0] != SYNC0: continue
                b2 = self.ser.read(1)
                if not b2 or b2[0] != SYNC1: continue

                payload = self.ser.read(12)
                if len(payload) == 12:
                    values = struct.unpack_from("<5h", payload, 2)
                    self.data_received.emit(values)
        except Exception as e:
            self.status_msg.emit(f"Error: {e}")
        finally:
            if self.ser: self.ser.close()

    def send_command(self, text):
        if self.ser and self.ser.is_open:
            try:
                cmd = f"{text}\n"
                self.ser.write(cmd.encode('ascii'))
                self.status_msg.emit(f"Sent: {text}")
            except Exception as e:
                self.status_msg.emit(f"Write Error: {e}")

    def stop(self):
        self.running = False
        self.wait()

class LivePlotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control Center")
        
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)
        top_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(top_layout)

        # Plot Area
        self.plot_widget = pg.PlotWidget(title="Live Data Feedback")
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.enableAutoRange(axis='y', enable=True)
        top_layout.addWidget(self.plot_widget, stretch=4)

        # Sidebar
        controls = QtWidgets.QVBoxLayout()
        top_layout.addLayout(controls, stretch=1)

        # --- Motion Controls ---
        controls.addWidget(QtWidgets.QLabel("<b>Motion Control:</b>"))
        
        # Enable Switch
        self.enable_cb = QtWidgets.QCheckBox("Motor Enabled")
        self.enable_cb.toggled.connect(lambda chk: self.worker.send_command(f"en {1 if chk else 0}"))
        controls.addWidget(self.enable_cb)

        # Speed Control
        controls.addWidget(QtWidgets.QLabel("Target Speed (RPM):"))
        self.speed_input = QtWidgets.QSpinBox()
        self.speed_input.setRange(-32768, 32767)
        self.speed_input.setValue(1000)
        controls.addWidget(self.speed_input)
        
        self.speed_btn = QtWidgets.QPushButton("Set Speed")
        self.speed_btn.clicked.connect(lambda: self.worker.send_command(f"spd {self.speed_input.value()}"))
        controls.addWidget(self.speed_btn)

        # Current Control
        controls.addWidget(QtWidgets.QLabel("Max Current (mA):"))
        self.curr_input = QtWidgets.QSpinBox()
        self.curr_input.setRange(0, 5000)
        self.curr_input.setValue(600)
        controls.addWidget(self.curr_input)
        
        self.curr_btn = QtWidgets.QPushButton("Set Imax")
        self.curr_btn.clicked.connect(lambda: self.worker.send_command(f"imax {self.curr_input.value()}"))
        controls.addWidget(self.curr_btn)

        controls.addSpacing(20)
        controls.addWidget(QtWidgets.QLabel("<b>Plot Settings:</b>"))
        
        # X-Axis Time & Freq (Simplified UI)
        self.time_span_input = QtWidgets.QSpinBox()
        self.time_span_input.setRange(1, 60)
        self.time_span_input.setValue(5)
        self.time_span_input.setPrefix("Window: ")
        self.time_span_input.setSuffix("s")
        self.time_span_input.valueChanged.connect(self.update_buffer_size)
        controls.addWidget(self.time_span_input)

        # Signal Toggles
        self.colors = ['#FF5555', '#55FF55', '#5555FF', '#FFFF55', '#FF55FF']
        self.curves = []
        self.data_buffers = []
        for i in range(5):
            curve = self.plot_widget.plot(pen=pg.mkPen(self.colors[i], width=2), name=f"V{i+1}")
            self.curves.append(curve)
            cb = QtWidgets.QCheckBox(f"Signal {i+1}")
            cb.setChecked(True)
            cb.stateChanged.connect(lambda state, idx=i: self.curves[idx].setVisible(state == 2))
            controls.addWidget(cb)

        controls.addStretch()

        # Status Bar / Console
        self.console_label = QtWidgets.QLabel("Status: Ready")
        self.console_label.setStyleSheet("border-top: 1px solid #ccc; padding-top: 5px; color: #555;")
        main_layout.addWidget(self.console_label)

        self.update_buffer_size()

        # Start Serial
        self.worker = SerialWorker()
        self.worker.data_received.connect(self.update_plot)
        self.worker.status_msg.connect(self.console_label.setText)
        self.worker.start()

    def update_buffer_size(self):
        # Using fixed 100Hz for time axis calculation (adjustable)
        freq = 100 
        seconds = self.time_span_input.value()
        self.max_points = int(freq * seconds)
        self.data_buffers = [np.zeros(self.max_points) for _ in range(5)]
        self.x_axis_values = np.linspace(-seconds, 0, self.max_points)

    def update_plot(self, values):
        for i in range(5):
            self.data_buffers[i] = np.roll(self.data_buffers[i], -1)
            self.data_buffers[i][-1] = values[i]
            if self.curves[i].isVisible():
                self.curves[i].setData(self.x_axis_values, self.data_buffers[i])

    def closeEvent(self, event):
        self.worker.stop()
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = LivePlotter()
    window.resize(1100, 650)
    window.show()
    sys.exit(app.exec())
