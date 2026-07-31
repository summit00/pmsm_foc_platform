
import collections
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np

# ── Protocol ──────────────────────────────────────────────────────────────────
RX_MAGIC    = 0xABCD
TX_MAGIC    = 0xDCBA
SAMPLE_BYTES = 3
PAYLOAD_N   = 10

TX_SLOTS = [
    ("Enable",           1.0,   "",       "0"),
    ("Mode",             1.0,   "",       "1"),
    ("Target Speed",     100.0, "rpm",    "1000"),
    ("Accel",            100.0, "rpm/s",  "500"),
    ("Current Limit",    10.0,  "mA",     "2000"),
    ("Sine Amplitude",   1.0,   "mV",    "1000"),
    ("Reserved[6]",       1.0,   "",       "0"),
    ("Reserved[7]",       1.0,   "",       "0"),
    ("Reserved[8]",       1.0,   "",       "0"),
    ("Reserved[9]",       1.0,   "",       "0"),
]

RX_SLOTS = [
    ("Udc",     100.0, "V"),
    ("demandSpeed", 1.0, "rpm"),
    ("feedbackSpeed", 1.0, "rpm"),
    ("encoderSpeed", 1.0, "rpm"),
    ("observerSpeed", 1.0, "rpm"),
    ("Id",      1000.0,"A"),
    ("Iq",      1000.0,"A"),
    ("encoderAngle", 100.0, "°"),
    ("observerAngle", 100.0, "°"),
    ("angleError", 100.0, "°"),
]

def build_frame(payload_ints: list[int], seq: int) -> bytes:
    return struct.pack("<HH" + "i" * PAYLOAD_N, RX_MAGIC, seq & 0xFFFF, *payload_ints)

# ── Serial Worker Thread ──────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    def __init__(self, port, baud, on_error_cb):
        super().__init__(daemon=True)
        self._port     = port
        self._baud     = baud
        self._on_error = on_error_cb
        self._ser      = None
        self._stop_evt = threading.Event()
        self._tx_queue = queue.SimpleQueue()
        self.error     = None
        
        self._lock = threading.Lock()
        self._samples_list = []
        self._frame_count = 0
        self._new_data_flag = False

    def run(self):
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.01)
            buf = b""
            magic_bytes = struct.pack("<H", TX_MAGIC)
            current_state = [0] * 10
            seen_ids = set()
            
            while not self._stop_evt.is_set():
                while True:
                    try:
                        self._ser.write(self._tx_queue.get(block=False))
                    except queue.Empty:
                        break

                chunk = self._ser.read(4096)
                if not chunk:
                    continue
                
                buf += chunk
                while len(buf) >= 6:
                    idx = buf.find(magic_bytes)
                    if idx == -1:
                        buf = buf[-1:] if buf else b""
                        break
                    if idx > 0:
                        buf = buf[idx:]
                        if len(buf) < 6:
                            break
                    
                    magic, seq, sample_count = struct.unpack_from("<HHH", buf, 0)
                    if not (1 <= sample_count <= 240):
                        buf = buf[2:]
                        continue
                    
                    expected_len = 6 + sample_count * SAMPLE_BYTES
                    if len(buf) < expected_len:
                        break
                    
                    batch_samples = []
                    for i in range(sample_count):
                        offset = 6 + i * SAMPLE_BYTES
                        sid, val = struct.unpack_from("<Bh", buf, offset)
                        if 1 <= sid <= 10:
                            if sid in seen_ids:
                                batch_samples.append(list(current_state))
                                seen_ids.clear()
                            current_state[sid - 1] = val
                            seen_ids.add(sid)
                    
                    buf = buf[expected_len:]
                    
                    if batch_samples:
                        with self._lock:
                            self._samples_list.extend(batch_samples)
                            self._frame_count += len(batch_samples)
                            self._new_data_flag = True
                            
        except Exception as e:
            self.error = str(e)
            self._on_error()

    def get_new_samples(self):
        with self._lock:
            flag = self._new_data_flag
            self._new_data_flag = False
            samples = self._samples_list
            self._samples_list = []
            return flag, samples, self._frame_count

    def send(self, data: bytes):
        self._tx_queue.put(data)

    def stop(self):
        self._stop_evt.set()
        if self._ser:
            try:
                self._ser.close()
            except:
                pass

# ── GUI App ───────────────────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PMSM FOC — Motor Control (Dynamic Blitted GUI)")
        self.resizable(False, False)
        self._reader = None
        self._seq    = 0
        
        # Consistent pixel depth horizontal density buffer
        self.PLOT_WINDOW_POINTS = 1000
        self._history_data = np.zeros((self.PLOT_WINDOW_POINTS, PAYLOAD_N), dtype=np.float32)
        self._history_head = 0
        self._total_samples_received = 0
        self._downsample_counter = 0
        
        # Throttled Auto-scaling metrics 
        self._y_scale_ticks = 0
        self._bg_cache = None
        self._plot_active = True
        
        self._build_ui()
        self._refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        
        self.after(500, self._rebuild_axes_and_cache)
        self._start_loops()

    def _build_ui(self):
        pad = {"padx": 5, "pady": 3}

        conn = ttk.LabelFrame(self, text="Connection")
        conn.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=6)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, **pad)
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(conn, textvariable=self._port_var, width=12, state="readonly")
        self._port_cb.grid(row=0, column=1, **pad)

        ttk.Label(conn, text="Baud:").grid(row=0, column=2, **pad)
        self._baud_var = tk.StringVar(value="115200")
        ttk.Entry(conn, textvariable=self._baud_var, width=8).grid(row=0, column=3, **pad)

        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=0, column=4, **pad)
        self._conn_btn = ttk.Button(conn, text="Connect", command=self._toggle_connect)
        self._conn_btn.grid(row=0, column=5, **pad)

        self._status_lbl = ttk.Label(conn, text="● Disconnected", foreground="red", width=22)
        self._status_lbl.grid(row=0, column=6, **pad)

        # Commands Panel
        tx_frame = ttk.LabelFrame(self, text="Commands  (PC → MCU)")
        tx_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=4)

        ttk.Label(tx_frame, text="#",     width=2,  anchor="center", font=("", 8, "bold")).grid(row=0, column=0)
        ttk.Label(tx_frame, text="Name",  width=16, anchor="w",      font=("", 8, "bold")).grid(row=0, column=1)
        ttk.Label(tx_frame, text="Value", width=12, anchor="center", font=("", 8, "bold")).grid(row=0, column=2)
        ttk.Label(tx_frame, text="Unit",  width=5,  anchor="w",      font=("", 8, "bold")).grid(row=0, column=3)
        ttk.Separator(tx_frame, orient="horizontal").grid(row=1, column=0, columnspan=4, sticky="ew", pady=2)

        self._tx_vars = []
        for i, (label, _, unit, default) in enumerate(TX_SLOTS):
            row = i + 2
            ttk.Label(tx_frame, text=str(i), width=2, anchor="center").grid(row=row, column=0, **pad)
            ttk.Label(tx_frame, text=label,  width=16, anchor="w").grid(row=row, column=1, **pad)
            var = tk.StringVar(value=default)
            entry = ttk.Entry(tx_frame, textvariable=var, width=12, justify="right")
            entry.grid(row=row, column=2, **pad)
            entry.bind("<Return>", lambda _e: self._send_command())
            self._tx_vars.append(var)
            ttk.Label(tx_frame, text=unit, width=5, anchor="w", foreground="gray").grid(row=row, column=3, **pad)

        send_row = PAYLOAD_N + 2
        btn_f = ttk.Frame(tx_frame)
        btn_f.grid(row=send_row, column=0, columnspan=4, pady=8)
        ttk.Button(btn_f, text="Send (Enter)", command=self._send_command, width=14).pack(side="left", padx=4)
        ttk.Button(btn_f, text="STOP Motor", command=self._stop_motor, width=12).pack(side="left", padx=4)

        # Telemetry Panel
        rx_frame = ttk.LabelFrame(self, text="Telemetry  (MCU → PC)")
        rx_frame.grid(row=1, column=1, sticky="nsew", padx=8, pady=4)

        ttk.Label(rx_frame, text="#",     width=2,  anchor="center", font=("", 8, "bold")).grid(row=0, column=0)
        ttk.Label(rx_frame, text="Name",  width=18, anchor="w",      font=("", 8, "bold")).grid(row=0, column=1)
        ttk.Label(rx_frame, text="Value", width=12, anchor="center", font=("", 8, "bold")).grid(row=0, column=2)
        ttk.Label(rx_frame, text="Unit",  width=5,  anchor="w",      font=("", 8, "bold")).grid(row=0, column=3)
        ttk.Separator(rx_frame, orient="horizontal").grid(row=1, column=0, columnspan=4, sticky="ew", pady=2)

        self._rx_vars = []
        for i, (label, scale, unit) in enumerate(RX_SLOTS):
            row = i + 2
            ttk.Label(rx_frame, text=str(i), width=2, anchor="center").grid(row=row, column=0, **pad)
            ttk.Label(rx_frame, text=label,  width=18, anchor="w").grid(row=row, column=1, **pad)
            var = tk.StringVar(value="—")
            entry = ttk.Entry(rx_frame, textvariable=var, width=12, state="readonly", justify="right", font=("Courier", 10))
            entry.grid(row=row, column=2, **pad)
            self._rx_vars.append(var)
            ttk.Label(rx_frame, text=unit, width=5, anchor="w", foreground="gray").grid(row=row, column=3, **pad)

        # Footer
        rx_ind = ttk.Frame(self)
        rx_ind.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=2)

        ttk.Label(rx_ind, text="RX:").pack(side="left", padx=(0, 4))
        self._rx_led = tk.Label(rx_ind, text="●", foreground="gray", font=("", 14), width=2)
        self._rx_led.pack(side="left")
        self._rx_count_var = tk.StringVar(value="0 frames")
        ttk.Label(rx_ind, textvariable=self._rx_count_var, width=14).pack(side="left", padx=4)
        self._rx_rate_var = tk.StringVar(value="0 Hz")
        ttk.Label(rx_ind, textvariable=self._rx_rate_var, width=8).pack(side="left", padx=4)

        self._last_gui_frame_count = 0
        self._rx_rate_count  = 0

        self._build_plots_panel()

        self._log_var = tk.StringVar(value="Ready.")
        ttk.Label(self, textvariable=self._log_var, anchor="w", relief="sunken").grid(
            row=4, column=0, columnspan=2, sticky="ew", padx=8, pady=4
        )

    def _build_plots_panel(self):
        plot_frame = ttk.LabelFrame(self, text="Plots (Custom Signal Matrices)")
        plot_frame.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=8, pady=4)

        # 1. Main Canvas Setup
        self._fig = Figure(figsize=(11, 3.4), dpi=96, facecolor="#f0f0f0")
        self._fig.subplots_adjust(left=0.06, right=0.98, top=0.88, bottom=0.15, wspace=0.25)
        self._ax = [self._fig.add_subplot(1, 2, 1), self._fig.add_subplot(1, 2, 2)]
        
        self._canvas = FigureCanvasTkAgg(self._fig, master=plot_frame)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)

        self._toolbar = NavigationToolbar2Tk(self._canvas, plot_frame, pack_toolbar=False)
        self._toolbar.update()
        self._toolbar.pack(side="bottom", fill="x")

        # 2. Controls & Limit Settings Panels
        ctrl = ttk.Frame(plot_frame)
        ctrl.pack(fill="x", padx=6, pady=2)

        # Time Window Controls
        ttk.Label(ctrl, text="Window:").grid(row=0, column=0, padx=2, sticky="w")
        self._window_var = tk.StringVar(value="5.0")
        self._window_spin = ttk.Spinbox(ctrl, from_=0.5, to=60.0, increment=1.0, textvariable=self._window_var, width=5)
        self._window_spin.grid(row=0, column=1, padx=2, sticky="w")
        ttk.Label(ctrl, text="sec").grid(row=0, column=2, padx=(0, 15), sticky="w")

        # Plot Start/Stop Toggle Button
        self._plot_active_btn = ttk.Button(ctrl, text="Pause Plot", command=self._toggle_plot_active, width=12)
        self._plot_active_btn.grid(row=0, column=3, padx=(0, 15), sticky="w")

        # Plot 1 Limits Controller (Left)
        ttk.Label(ctrl, text="Plot 1 Y:").grid(row=0, column=4, padx=2, sticky="w")
        self._p1_auto_var = tk.BooleanVar(value=True)
        self._p1_auto_cb = ttk.Checkbutton(ctrl, text="Auto", variable=self._p1_auto_var, command=self._rebuild_axes_and_cache)
        self._p1_auto_cb.grid(row=0, column=5, padx=2, sticky="w")
        
        self._p1_min_var = tk.StringVar(value="-100")
        self._p1_max_var = tk.StringVar(value="3500")
        self._p1_min_ent = ttk.Entry(ctrl, textvariable=self._p1_min_var, width=5, justify="right")
        self._p1_max_ent = ttk.Entry(ctrl, textvariable=self._p1_max_var, width=5, justify="right")
        self._p1_min_ent.grid(row=0, column=6, padx=1, sticky="w")
        ttk.Label(ctrl, text="to").grid(row=0, column=7, padx=1)
        self._p1_max_ent.grid(row=0, column=8, padx=1, sticky="w")
        
        self._p1_min_ent.bind("<Return>", lambda e: self._rebuild_axes_and_cache())
        self._p1_max_ent.bind("<Return>", lambda e: self._rebuild_axes_and_cache())

        # Plot 2 Limits Controller (Right)
        ttk.Label(ctrl, text="   Plot 2 Y:").grid(row=0, column=9, padx=2, sticky="w")
        self._p2_auto_var = tk.BooleanVar(value=True)
        self._p2_auto_cb = ttk.Checkbutton(ctrl, text="Auto", variable=self._p2_auto_var, command=self._rebuild_axes_and_cache)
        self._p2_auto_cb.grid(row=0, column=10, padx=2, sticky="w")
        
        self._p2_min_var = tk.StringVar(value="-15")
        self._p2_max_var = tk.StringVar(value="15")
        self._p2_min_ent = ttk.Entry(ctrl, textvariable=self._p2_min_var, width=5, justify="right")
        self._p2_max_ent = ttk.Entry(ctrl, textvariable=self._p2_max_var, width=5, justify="right")
        self._p2_min_ent.grid(row=0, column=11, padx=1, sticky="w")
        ttk.Label(ctrl, text="to").grid(row=0, column=12, padx=1)
        self._p2_max_ent.grid(row=0, column=13, padx=1, sticky="w")
        
        self._p2_min_ent.bind("<Return>", lambda e: self._rebuild_axes_and_cache())
        self._p2_max_ent.bind("<Return>", lambda e: self._rebuild_axes_and_cache())

        # 3. Dynamic Signal Selection Matrix Row Blocks
        matrix_frame = ttk.Frame(plot_frame)
        matrix_frame.pack(fill="x", padx=6, pady=4)

        ttk.Label(matrix_frame, text="Plot 1 (Left):", font=("", 8, "bold")).grid(row=0, column=0, sticky="w", pady=2)
        self._p1_checks = []
        for idx, (name, _, _) in enumerate(RX_SLOTS):
            v = tk.BooleanVar(value=(name == "Udc"))
            cb = ttk.Checkbutton(matrix_frame, text=name, variable=v, command=self._rebuild_axes_and_cache)
            cb.grid(row=0, column=idx+1, padx=4, sticky="w")
            self._p1_checks.append((idx, v))

        ttk.Label(matrix_frame, text="Plot 2 (Right):", font=("", 8, "bold")).grid(row=1, column=0, sticky="w", pady=2)
        self._p2_checks = []
        for idx, (name, _, _) in enumerate(RX_SLOTS):
            v = tk.BooleanVar(value=(name in ["Id", "Iq"]))
            cb = ttk.Checkbutton(matrix_frame, text=name, variable=v, command=self._rebuild_axes_and_cache)
            cb.grid(row=1, column=idx+1, padx=4, sticky="w")
            self._p2_checks.append((idx, v))

        # Core active dictionary references for blitted fast loops
        self._active_lines_p1 = []
        self._active_lines_p2 = []
        self._x_indices = np.arange(self.PLOT_WINDOW_POINTS)

    def _get_current_view_data(self):
        """Helper to compute aligned data array view matching rolling history."""
        head = self._history_head
        if self._total_samples_received < self.PLOT_WINDOW_POINTS:
            active_data = self._history_data[:head]
            pad_len = self.PLOT_WINDOW_POINTS - len(active_data)
            return np.vstack((np.zeros((pad_len, PAYLOAD_N)), active_data))
        else:
            return np.vstack((self._history_data[head:], self._history_data[:head]))

    def _rebuild_axes_and_cache(self):
        """Re-generates vector tracking lines and locks clean background grid states."""
        COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]

        # Clear standard layout components safely 
        self._ax[0].cla()
        self._ax[1].cla()

        # Re-apply static baseline grid geometry
        for ax in self._ax:
            ax.set_xlim(0, self.PLOT_WINDOW_POINTS)
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.tick_params(labelsize=7)

        # CRITICAL FIX: Extract real data instead of empty arrays `[]`
        # This gives Matplotlib fallback context data during toolbar actions.
        view_data = self._get_current_view_data()

        # --- Reconstruct Left Plot (Plot 1) ---
        self._active_lines_p1.clear()
        selected_p1_names = []
        color_idx = 0
        for idx, var in self._p1_checks:
            if var.get():
                name, scale, unit = RX_SLOTS[idx]
                selected_p1_names.append(name)
                # FIX: Initialize with valid data matrices
                line, = self._ax[0].plot(self._x_indices, view_data[:, idx] / scale, color=COLORS[color_idx % len(COLORS)], linewidth=1.2, animated=True, label=name)
                self._active_lines_p1.append((idx, scale, line))
                color_idx += 1
        
        if selected_p1_names:
            self._ax[0].set_title(", ".join(selected_p1_names), fontsize=8)
            if len(selected_p1_names) > 1:
                self._ax[0].legend(fontsize=7, loc="upper left")
        else:
            self._ax[0].set_title("No Signals Selected", fontsize=8)

        # Set manual limits if auto-scale is disabled
        if not self._p1_auto_var.get():
            try:
                self._ax[0].set_ylim(float(self._p1_min_var.get()), float(self._p1_max_var.get()))
            except ValueError:
                self._ax[0].set_ylim(-10, 10)
        else:
            self._ax[0].relim()
            self._ax[0].autoscale_view()

        # --- Reconstruct Right Plot (Plot 2) ---
        self._active_lines_p2.clear()
        selected_p2_names = []
        color_idx = 0
        for idx, var in self._p2_checks:
            if var.get():
                name, scale, unit = RX_SLOTS[idx]
                selected_p2_names.append(name)
                # FIX: Initialize with valid data matrices
                line, = self._ax[1].plot(self._x_indices, view_data[:, idx] / scale, color=COLORS[color_idx % len(COLORS)], linewidth=1.2, animated=True, label=name)
                self._active_lines_p2.append((idx, scale, line))
                color_idx += 1

        if selected_p2_names:
            self._ax[1].set_title(", ".join(selected_p2_names), fontsize=8)
            if len(selected_p2_names) > 1:
                self._ax[1].legend(fontsize=7, loc="upper left")
        else:
            self._ax[1].set_title("No Signals Selected", fontsize=8)

        if not self._p2_auto_var.get():
            try:
                self._ax[1].set_ylim(float(self._p2_min_var.get()), float(self._p2_max_var.get()))
            except ValueError:
                self._ax[1].set_ylim(-1, 1)
        else:
            self._ax[1].relim()
            self._ax[1].autoscale_view()

        # Capture background snapshot layer cache
        self._canvas.draw()
        self._bg_cache = self._canvas.copy_from_bbox(self._fig.bbox)
        self._send_select_command()

    def _update_plot_views(self):
        """Dynamic Blitting Engine."""
        # CRITICAL FIX: If the plot is paused or the toolbar is currently engaged
        # in any interaction tool (Zoom, Pan, Subplot Config, etc.), completely bypass blitting.
        if not self._plot_active or self._toolbar.mode != "":
            return

        if self._bg_cache is None:
            return

        view_data = self._get_current_view_data()

        # Load plot data matrices safely
        for idx, scale, line in self._active_lines_p1:
            line.set_data(self._x_indices, view_data[:, idx] / scale)

        for idx, scale, line in self._active_lines_p2:
            line.set_data(self._x_indices, view_data[:, idx] / scale)

        # THROTTLED AUTO-SCALE ENGINE LOOP
        self._y_scale_ticks += 1
        if self._y_scale_ticks >= 15:  # Check scaling roughly every 450ms
            self._y_scale_ticks = 0
            limits_changed = False

            # Calculate for Plot 1 (If Auto is active)
            if self._p1_auto_var.get() and self._active_lines_p1:
                vals = [view_data[:, idx] / scale for idx, scale, _ in self._active_lines_p1]
                y_min = min(arr.min() for arr in vals)
                y_max = max(arr.max() for arr in vals)
                y_range = max(0.1, y_max - y_min)
                new_ylim = (y_min - y_range * 0.1, y_max + y_range * 0.1)
                
                curr_ylim = self._ax[0].get_ylim()
                if abs(curr_ylim[0] - new_ylim[0]) > (y_range * 0.05) or abs(curr_ylim[1] - new_ylim[1]) > (y_range * 0.05):
                    self._ax[0].set_ylim(new_ylim)
                    # Update numerical fields for user visibility
                    self._p1_min_var.set(f"{new_ylim[0]:.1f}")
                    self._p1_max_var.set(f"{new_ylim[1]:.1f}")
                    limits_changed = True

            # Calculate for Plot 2 (If Auto is active)
            if self._p2_auto_var.get() and self._active_lines_p2:
                vals = [view_data[:, idx] / scale for idx, scale, _ in self._active_lines_p2]
                y_min = min(arr.min() for arr in vals)
                y_max = max(arr.max() for arr in vals)
                y_range = max(0.05, y_max - y_min)
                new_ylim = (y_min - y_range * 0.1, y_max + y_range * 0.1)
                
                curr_ylim = self._ax[1].get_ylim()
                if abs(curr_ylim[0] - new_ylim[0]) > (y_range * 0.05) or abs(curr_ylim[1] - new_ylim[1]) > (y_range * 0.05):
                    self._ax[1].set_ylim(new_ylim)
                    self._p2_min_var.set(f"{new_ylim[0]:.1f}")
                    self._p2_max_var.set(f"{new_ylim[1]:.1f}")
                    limits_changed = True

            if limits_changed:
                self._canvas.draw()
                self._bg_cache = self._canvas.copy_from_bbox(self._fig.bbox)

        # Restore cached grid layer image
        self._canvas.restore_region(self._bg_cache)

        # Re-render active vector elements fast
        for _, _, line in self._active_lines_p1:
            self._ax[0].draw_artist(line)

        for _, _, line in self._active_lines_p2:
            self._ax[1].draw_artist(line)

        # Push bit-blocks directly to display pipeline
        self._canvas.blit(self._fig.bbox)
        self._canvas.flush_events()

    def _start_loops(self):
        self._periodic_ui_update()
        self._tick_rate()

    def _periodic_ui_update(self):
        if self._reader:
            new_data, samples, total_frames = self._reader.get_new_samples()
            
            if new_data and samples:
                last_sample = samples[-1]
                for i, (_, scale, _) in enumerate(RX_SLOTS):
                    val = last_sample[i] / scale
                    decimals = 3 if scale == 1000.0 else 1
                    self._rx_vars[i].set(f"{val:.{decimals}f}")
                
                try:
                    w_sec = float(self._window_var.get())
                except ValueError:
                    w_sec = 5.0
                w_sec = max(0.5, min(60.0, w_sec))
                
                step_skip = max(1, int((w_sec * 20000) / self.PLOT_WINDOW_POINTS))

                for sample in samples:
                    self._downsample_counter += 1
                    if self._downsample_counter >= step_skip:
                        self._downsample_counter = 0
                        if self._plot_active:
                            self._history_data[self._history_head] = sample
                            self._history_head = (self._history_head + 1) % self.PLOT_WINDOW_POINTS
                            self._total_samples_received += 1

                delta_frames = total_frames - self._last_gui_frame_count
                self._last_gui_frame_count = total_frames
                self._rx_rate_count += delta_frames
                self._rx_count_var.set(f"{total_frames} frames")
                self._rx_led.config(foreground="lime green")
                
                if self._plot_active:
                    self._update_plot_views()
            else:
                self._rx_led.config(foreground="gray")

        # FIX: If the user is actively customizing subplots or zooming while paused,
        # make sure lines retain data positions in natural Matplotlib pipeline cycles.
        if not self._plot_active:
            view_data = self._get_current_view_data()
            for idx, scale, line in self._active_lines_p1:
                line.set_data(self._x_indices, view_data[:, idx] / scale)
            for idx, scale, line in self._active_lines_p2:
                line.set_data(self._x_indices, view_data[:, idx] / scale)
            
            # If a toolbar action tool is engaged, allow responsive background render iterations
            if self._toolbar.mode != "":
                self._canvas.draw_idle()

        self.after(30, self._periodic_ui_update)

    def _tick_rate(self):
        self._rx_rate_var.set(f"{self._rx_rate_count} Hz")
        self._rx_rate_count = 0
        self.after(1000, self._tick_rate)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _toggle_connect(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
            self._conn_btn.config(text="Connect")
            self._status_lbl.config(text="● Disconnected", foreground="red")
        else:
            port = self._port_var.get()
            try:
                baud = int(self._baud_var.get())
            except ValueError:
                baud = 115200
            
            self._reader = SerialReader(port, baud, self._on_serial_error)
            self._reset_plots()
            self._reader.start()
            self._conn_btn.config(text="Disconnect")
            self._status_lbl.config(text=f"● {port}", foreground="green")
            self._log(f"Connected to {port} @ {baud}")

    def _on_serial_error(self):
        self.after(0, self._handle_serial_crash)

    def _handle_serial_crash(self):
        err = self._reader.error if self._reader else "Unknown fault"
        self._status_lbl.config(text="● Error", foreground="red")
        self._log(f"Serial error: {err}")
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._conn_btn.config(text="Connect")

    def _send_command(self):
        if not self._reader:
            self._log("Not connected.")
            return
        payload = []
        for i, (label, scale, unit, _) in enumerate(TX_SLOTS):
            raw_str = self._tx_vars[i].get().strip()
            try:
                val = float(raw_str)
            except ValueError:
                self._log(f"Invalid value in slot {i} ({label}): '{raw_str}'")
                return
            payload.append(int(val * scale))
        frame = build_frame(payload, self._seq)
        self._seq += 1
        self._reader.send(frame)

    def _send_select_command(self):
        if not self._reader:
            return
        
        # Determine union of checked variables across Plot 1 and Plot 2
        active_indices = set()
        for idx, var in self._p1_checks:
            if var.get():
                active_indices.add(idx)
        for idx, var in self._p2_checks:
            if var.get():
                active_indices.add(idx)
        
        active_ids = sorted([idx + 1 for idx in active_indices])
        padded_ids = list(active_ids) + [0] * (10 - len(active_ids))
        padded_ids = padded_ids[:10]
        
        USB_SELECT_MAGIC = 0xABCE
        payload_padding = b'\x00' * 20
        frame = struct.pack('<HH10H20s', USB_SELECT_MAGIC, self._seq & 0xFFFF, *padded_ids, payload_padding)
        self._seq += 1
        self._reader.send(frame)

    def _stop_motor(self):
        for var in self._tx_vars:
            var.set("0")
        self._send_command()
        self._log("STOP sent — motor disabled, variables zeroed.")
        self._reset_plots()

    def _reset_plots(self):
        self._history_head = 0
        self._total_samples_received = 0
        self._downsample_counter = 0
        self._y_scale_ticks = 0
        self._history_data.fill(0)
        self._rebuild_axes_and_cache()

    def _toggle_plot_active(self):
        self._plot_active = not self._plot_active
        if self._plot_active:
            # Safely deselect toolbar configuration options on resume
            if self._toolbar.mode != "":
                if self._toolbar.mode == "zoom in":
                    self._toolbar.zoom()
                elif self._toolbar.mode == "pan/zoom":
                    self._toolbar.pan()
            self._plot_active_btn.config(text="Pause Plot")
            self._rebuild_axes_and_cache()
            self._log("Plotting resumed.")
        else:
            self._plot_active_btn.config(text="Plot Paused")
            self._log("Plotting paused. Configuration tools and zoom options are fully functional.")

    def _log(self, msg: str):
        self._log_var.set(msg)

    def _on_close(self):
        if self._reader:
            self._reader.stop()
        self.destroy()

if __name__ == "__main__":
    app = App()
    app.mainloop()
