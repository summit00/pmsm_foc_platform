import collections
import csv
import json
import os
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np
import serial
import serial.tools.list_ports

# ── Registry Loading & Protocol Setup ─────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
JSON_PATH = os.path.join(PROJECT_ROOT, "app", "telemetry_registry.json")

try:
    with open(JSON_PATH, "r") as f:
        REGISTRY = json.load(f)
except Exception as e:
    # Fail-safe default registry if file loading fails
    REGISTRY = {
        "commands": [
            {"name": "mEnable", "type": "uint8_t", "default": 0},
            {"name": "mMode", "type": "uint8_t", "default": 0},
            {"name": "targetSpeed_rpm", "type": "float", "default": 0.0},
            {"name": "mAcceleration_rpm_s", "type": "float", "default": 500.0},
            {"name": "mIsAbs_mA", "type": "float", "default": 0.0},
        ],
        "telemetry": [
            {"id": 1, "name": "Udc_V", "scale": 100.0, "unit": "V", "description": "DC Link Bus Voltage"},
            {"id": 2, "name": "demandSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Demand Electrical Speed"},
            {"id": 3, "name": "feedbackSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Feedback Rotor Speed"},
            {"id": 4, "name": "encoderSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Encoder Measured Speed"},
            {"id": 5, "name": "observerSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Observer Estimated Speed"},
            {"id": 7, "name": "Id_A", "scale": 1000.0, "unit": "A", "description": "D-axis feedback current"},
            {"id": 8, "name": "Iq_A", "scale": 1000.0, "unit": "A", "description": "Q-axis feedback current"},
        ],
    }

RX_MAGIC     = 0xABCD
TX_MAGIC     = 0xDCBA
SAMPLE_BYTES = 3
PAYLOAD_N    = 10

# Map C++ variable names to user friendly labels, scales, units
CMD_MAP = {
    "mEnable": ("Enable (0/1)", 1.0, ""),
    "mMode": ("Mode", 1.0, ""),
    "targetSpeed_rpm": ("Target Speed", 100.0, "rpm"),
    "mAcceleration_rpm_s": ("Accel", 100.0, "rpm/s"),
    "mIsAbs_mA": ("Current Limit", 10.0, "mA"),
    "encoderOffset": ("Encoder Offset", 1.0, "ticks"),
}

TX_SLOTS = []
for cmd in REGISTRY["commands"]:
    name = cmd["name"]
    label, scale, unit = CMD_MAP.get(name, (name, 1.0, ""))
    TX_SLOTS.append((label, scale, unit, str(cmd["default"])))

while len(TX_SLOTS) < 10:
    idx = len(TX_SLOTS)
    TX_SLOTS.append((f"Reserved[{idx}]", 1.0, "", "0"))


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

        self.active_ids = []
        self._current_values = {}

    def set_active_ids(self, active_ids):
        with self._lock:
            self.active_ids = list(active_ids)
            self._current_values.clear()

    def run(self):
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.01)
            buf = b""
            magic_bytes = struct.pack("<H", TX_MAGIC)
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

                with self._lock:
                    current_active = list(self.active_ids)

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

                        if sid in seen_ids:
                            sample = [self._current_values.get(aid, 0) for aid in current_active]
                            batch_samples.append(sample)
                            seen_ids.clear()

                        self._current_values[sid] = val
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
            except Exception:
                pass


# ── Color Palette Configuration ──────────────────────────────────────────────
PLOT_COLORS = [
    "#38bdf8",  # Sky blue
    "#34d399",  # Emerald
    "#f472b6",  # Rose pink
    "#fbbf24",  # Amber
    "#a78bfa",  # Purple
    "#fb923c",  # Orange
    "#2dd4bf",  # Teal
    "#f87171",  # Coral
    "#a3e635",  # Lime
    "#818cf8",  # Indigo
]


# ── GUI App ───────────────────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PMSM FOC — Motor Control & Dynamic Telemetry Platform")
        self.geometry("1480x960")
        self.minsize(1100, 750)
        self._reader = None
        self._seq    = 0

        # Number of active plots (1 to 4)
        self._num_plots = 2

        # Selected signal IDs per plot (0 -> P1, 1 -> P2, 2 -> P3, 3 -> P4)
        self._selected = [
            {1},      # Plot 1: Udc_V
            {7, 8},   # Plot 2: Id_A, Iq_A
            set(),    # Plot 3
            set()     # Plot 4
        ]
        self._current_vals_str = {}

        # Telemetry registry definitions
        self._telemetry_vars = []
        self._telemetry_by_id = {}

        # Resolve registry JSON file path relative to main.py
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        default_json = os.path.join(project_root, "app", "telemetry_registry.json")

        if os.path.exists(default_json):
            self._load_json_file(default_json)
        else:
            self._load_fallback_registry()

        # Circular buffer for live plotting
        self.PLOT_WINDOW_POINTS = 1000
        self._history_data = np.zeros((self.PLOT_WINDOW_POINTS, PAYLOAD_N), dtype=np.float32)
        self._history_head = 0
        self._total_samples_received = 0
        self._downsample_counter = 0

        # Auto-scaling & render cache
        self._y_scale_ticks = 0
        self._bg_cache = None
        self._plot_active = True

        # Data Logging state
        self._logging_active = False
        self._log_records = []
        self._log_start_time = 0.0

        # Subplot axis management
        self._ax = []
        self._ax_twin = []
        self._active_lines = [[], [], [], []]  # list of tuples (col_idx, scale, line, is_twin) per plot
        self._x_indices = np.arange(self.PLOT_WINDOW_POINTS)

        # Plot Controls Variables (Per-plot Same Axis & Y Limits)
        self._same_axis_vars = [tk.BooleanVar(value=True) for _ in range(4)]
        self._p_auto_vars    = [tk.BooleanVar(value=True) for _ in range(4)]
        self._p_min_vars     = [tk.StringVar(value="-100"), tk.StringVar(value="-15"), tk.StringVar(value="-10"), tk.StringVar(value="-10")]
        self._p_max_vars     = [tk.StringVar(value="3500"), tk.StringVar(value="15"), tk.StringVar(value="10"), tk.StringVar(value="10")]

        self._apply_style()
        self._build_ui()
        self._update_tree()
        self._refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.after(400, self._rebuild_axes_and_cache)
        self._start_loops()

    def _load_json_dialog(self):
        filename = filedialog.askopenfilename(
            title="Select Telemetry JSON Registry",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
        )
        if filename:
            self._load_json_file(filename)

    def _load_json_file(self, filename):
        try:
            with open(filename, "r") as f:
                registry = json.load(f)

            self._telemetry_vars = registry["telemetry"]
            self._telemetry_by_id = {v["id"]: v for v in self._telemetry_vars}

            all_ids = set(v["id"] for v in self._telemetry_vars)
            for i in range(4):
                self._selected[i] = {vid for vid in self._selected[i] if vid in all_ids}

            if not self._selected[0] and self._telemetry_vars:
                self._selected[0] = {self._telemetry_vars[0]["id"]}
            if not self._selected[1] and len(self._telemetry_vars) > 1:
                self._selected[1] = {self._telemetry_vars[1]["id"]}

            self._current_vals_str.clear()

            if hasattr(self, "_tree"):
                self._update_tree()
                self._rebuild_axes_and_cache()
                self._log(f"Loaded telemetry: {os.path.basename(filename)}")
        except Exception as e:
            if hasattr(self, "_tree"):
                self._log(f"Failed to load JSON: {e}")
            else:
                print(f"Failed to load default JSON: {e}")

    def _load_fallback_registry(self):
        self._telemetry_vars = [
            {"id": 1, "name": "Udc_V", "scale": 100.0, "unit": "V", "description": "DC Link Bus Voltage"},
            {"id": 2, "name": "demandSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Demand Speed"},
            {"id": 3, "name": "feedbackSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Feedback Speed"},
            {"id": 4, "name": "encoderSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Encoder Speed"},
            {"id": 5, "name": "observerSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Observer Speed"},
            {"id": 7, "name": "Id_A", "scale": 1000.0, "unit": "A", "description": "D-axis Current"},
            {"id": 8, "name": "Iq_A", "scale": 1000.0, "unit": "A", "description": "Q-axis Current"},
        ]
        self._telemetry_by_id = {v["id"]: v for v in self._telemetry_vars}

    def _apply_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")

        bg_main = "#18181b"       # Zinc 900
        bg_card = "#27272a"       # Zinc 800
        fg_main = "#f4f4f5"       # Zinc 100
        accent_indigo = "#6366f1"

        self.configure(bg=bg_main)

        style.configure(".", background=bg_main, foreground=fg_main, fieldbackground=bg_card, font=("Segoe UI", 9))

        # LabelFrames
        style.configure("TLabelframe", background=bg_main, bordercolor="#3f3f46", borderwidth=1, relief="solid")
        style.configure("TLabelframe.Label", background=bg_main, foreground=accent_indigo, font=("Segoe UI", 9, "bold"))

        # Frames
        style.configure("TFrame", background=bg_main)
        style.configure("Card.TFrame", background=bg_card)

        # Buttons
        style.configure("TButton", background=bg_card, foreground=fg_main, borderwidth=1, bordercolor="#3f3f46", relief="flat", padding=(8, 4))
        style.map("TButton",
                  background=[("active", accent_indigo), ("pressed", "#4f46e5")],
                  foreground=[("active", "#ffffff")])

        # Action Buttons
        style.configure("Start.TButton", background="#065f46", foreground="#a7f3d0", borderwidth=1, bordercolor="#059669", font=("Segoe UI", 9, "bold"))
        style.map("Start.TButton", background=[("active", "#059669"), ("pressed", "#047857")], foreground=[("active", "#ffffff")])

        style.configure("Stop.TButton", background="#881337", foreground="#fecdd3", borderwidth=1, bordercolor="#e11d48", font=("Segoe UI", 9, "bold"))
        style.map("Stop.TButton", background=[("active", "#e11d48"), ("pressed", "#be123c")], foreground=[("active", "#ffffff")])

        style.configure("Rec.TButton", background="#701a75", foreground="#f5d0fe", borderwidth=1, bordercolor="#c026d3", font=("Segoe UI", 9, "bold"))
        style.map("Rec.TButton", background=[("active", "#c026d3"), ("pressed", "#a21caf")], foreground=[("active", "#ffffff")])

        # Entries & Spinboxes
        style.configure("TEntry", fieldbackground=bg_card, foreground=fg_main, bordercolor="#3f3f46", borderwidth=1)
        style.configure("TSpinbox", fieldbackground=bg_card, foreground=fg_main, bordercolor="#3f3f46", borderwidth=1)
        style.configure("TCombobox", fieldbackground=bg_card, foreground=fg_main, bordercolor="#3f3f46", borderwidth=1)

        # Checkbutton
        style.configure("TCheckbutton", background=bg_main, foreground=fg_main, font=("Segoe UI", 8))

        # Scrollbars
        style.configure("Vertical.TScrollbar", background=bg_card, bordercolor="#3f3f46", arrowcolor=fg_main, troughcolor=bg_main)

        # Treeview
        style.configure("Treeview",
                        background=bg_card,
                        foreground=fg_main,
                        fieldbackground=bg_card,
                        rowheight=22,
                        borderwidth=0,
                        font=("Segoe UI", 9))
        style.configure("Treeview.Heading",
                        background="#20212b",
                        foreground=accent_indigo,
                        font=("Segoe UI", 8, "bold"),
                        borderwidth=1,
                        bordercolor="#3f3f46")
        style.map("Treeview",
                  background=[("selected", accent_indigo)],
                  foreground=[("selected", "#ffffff")])

    def _build_ui(self):
        pad = {"padx": 4, "pady": 2}
        bg_main = "#18181b"

        # Configure root grid weights
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)  # Plots panel expands vertically

        # ── Top Bar: Connection & Global Status ──────────────────────────────
        conn = ttk.LabelFrame(self, text="Connection & System Status")
        conn.grid(row=0, column=0, sticky="ew", padx=8, pady=(4, 2))

        ttk.Label(conn, text="Port:").pack(side="left", padx=(6, 2))
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(conn, textvariable=self._port_var, width=12, state="readonly")
        self._port_cb.pack(side="left", padx=2)

        ttk.Label(conn, text="Baud:").pack(side="left", padx=(8, 2))
        self._baud_var = tk.StringVar(value="115200")
        ttk.Entry(conn, textvariable=self._baud_var, width=8).pack(side="left", padx=2)

        ttk.Button(conn, text="Refresh", command=self._refresh_ports).pack(side="left", padx=3)
        self._conn_btn = ttk.Button(conn, text="Connect", command=self._toggle_connect)
        self._conn_btn.pack(side="left", padx=3)

        ttk.Button(conn, text="Load JSON Registry", command=self._load_json_dialog).pack(side="left", padx=6)

        self._status_lbl = ttk.Label(conn, text="● Disconnected", foreground="#ef4444", font=("Segoe UI", 9, "bold"))
        self._status_lbl.pack(side="left", padx=8)

        # Logging indicator in top bar
        ttk.Separator(conn, orient="vertical").pack(side="left", fill="y", padx=8, pady=2)
        self._log_status_lbl = ttk.Label(conn, text="Log: IDLE", foreground="#a1a1aa", font=("Segoe UI", 9))
        self._log_status_lbl.pack(side="left", padx=4)

        self._rx_rate_var = tk.StringVar(value="0 Hz")
        ttk.Label(conn, textvariable=self._rx_rate_var, width=10, anchor="e", foreground="#38bdf8").pack(side="right", padx=6)
        ttk.Label(conn, text="RX:").pack(side="right", padx=(4, 0))
        self._rx_led = tk.Label(conn, text="●", foreground="gray", bg=bg_main, font=("Segoe UI", 12))
        self._rx_led.pack(side="right")
        self._rx_count_var = tk.StringVar(value="0 frames")
        ttk.Label(conn, textvariable=self._rx_count_var, width=12, anchor="e").pack(side="right", padx=4)

        self._last_gui_frame_count = 0
        self._rx_rate_count = 0

        # ── Middle Section: Commands (Left) and Telemetry Table (Right) ─────
        mid_paned = ttk.Frame(self)
        mid_paned.grid(row=1, column=0, sticky="ew", padx=8, pady=2)
        mid_paned.columnconfigure(0, weight=0)  # Commands fixed width
        mid_paned.columnconfigure(1, weight=1)  # Telemetry table expands

        # 1. Commands Panel
        tx_frame = ttk.LabelFrame(mid_paned, text="Commands (PC → MCU)")
        tx_frame.grid(row=0, column=0, sticky="nsw", padx=(0, 4), pady=0)

        cmd_header = ttk.Frame(tx_frame)
        cmd_header.pack(fill="x", padx=4, pady=1)
        ttk.Label(cmd_header, text="#", width=2, anchor="center", font=("Segoe UI", 8, "bold"), foreground="#a1a1aa").pack(side="left")
        ttk.Label(cmd_header, text="Name", width=14, anchor="w", font=("Segoe UI", 8, "bold"), foreground="#a1a1aa").pack(side="left", padx=2)
        ttk.Label(cmd_header, text="Value", width=10, anchor="center", font=("Segoe UI", 8, "bold"), foreground="#a1a1aa").pack(side="left", padx=2)
        ttk.Label(cmd_header, text="Unit", width=5, anchor="w", font=("Segoe UI", 8, "bold"), foreground="#a1a1aa").pack(side="left")

        self._tx_vars = []
        for i, (label, _, unit, default) in enumerate(TX_SLOTS):
            row_f = ttk.Frame(tx_frame)
            row_f.pack(fill="x", padx=4, pady=1)
            ttk.Label(row_f, text=str(i), width=2, anchor="center", foreground="#71717a").pack(side="left")
            ttk.Label(row_f, text=label, width=14, anchor="w").pack(side="left", padx=2)
            var = tk.StringVar(value=default)
            entry = ttk.Entry(row_f, textvariable=var, width=10, justify="right")
            entry.pack(side="left", padx=2)
            entry.bind("<Return>", lambda _e: self._send_command())
            self._tx_vars.append(var)
            ttk.Label(row_f, text=unit, width=5, anchor="w", foreground="#a1a1aa").pack(side="left")

        # Command Buttons Panel: Send, Start Motor, Stop Motor
        btn_f = ttk.Frame(tx_frame)
        btn_f.pack(fill="x", padx=4, pady=(6, 4))
        ttk.Button(btn_f, text="Send (Enter)", command=self._send_command, width=12).pack(side="left", padx=2)
        self._start_motor_btn = ttk.Button(btn_f, text="▶ START Motor", style="Start.TButton", command=self._start_motor, width=14)
        self._start_motor_btn.pack(side="left", padx=2)
        self._stop_motor_btn = ttk.Button(btn_f, text="⏹ STOP Motor", style="Stop.TButton", command=self._stop_motor, width=13)
        self._stop_motor_btn.pack(side="left", padx=2)

        # 2. Telemetry Panel (Searchable Treeview with P1, P2, P3, P4 checkboxes)
        rx_frame = ttk.LabelFrame(mid_paned, text="Telemetry Selection (MCU → PC — Click P1-P4 to route to plot)")
        rx_frame.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=0)

        search_f = ttk.Frame(rx_frame)
        search_f.pack(fill="x", padx=6, pady=2)
        ttk.Label(search_f, text="Search:").pack(side="left", padx=2)
        self._search_var = tk.StringVar()
        self._search_var.trace_add("write", lambda *args: self._update_tree())
        search_ent = ttk.Entry(search_f, textvariable=self._search_var, width=16)
        search_ent.pack(side="left", padx=2)
        ttk.Button(search_f, text="Clear", command=lambda: self._search_var.set(""), width=5).pack(side="left", padx=2)

        self._active_sig_count_lbl = ttk.Label(search_f, text="Active MCU Streams: 0 / 10", foreground="#38bdf8", font=("Segoe UI", 8, "bold"))
        self._active_sig_count_lbl.pack(side="right", padx=6)

        tree_f = ttk.Frame(rx_frame)
        tree_f.pack(fill="both", expand=True, padx=6, pady=2)

        cols = ("name", "id", "value", "unit", "p1", "p2", "p3", "p4", "desc")
        self._tree = ttk.Treeview(tree_f, columns=cols, show="headings", height=8)
        self._tree.heading("name", text="Signal Name", anchor="w")
        self._tree.heading("id", text="ID", anchor="center")
        self._tree.heading("value", text="Live Value", anchor="e")
        self._tree.heading("unit", text="Unit", anchor="w")
        self._tree.heading("p1", text="Plot 1", anchor="center")
        self._tree.heading("p2", text="Plot 2", anchor="center")
        self._tree.heading("p3", text="Plot 3", anchor="center")
        self._tree.heading("p4", text="Plot 4", anchor="center")
        self._tree.heading("desc", text="Description", anchor="w")

        self._tree.column("name", width=140, anchor="w")
        self._tree.column("id", width=35, anchor="center")
        self._tree.column("value", width=75, anchor="e")
        self._tree.column("unit", width=45, anchor="w")
        self._tree.column("p1", width=45, anchor="center")
        self._tree.column("p2", width=45, anchor="center")
        self._tree.column("p3", width=45, anchor="center")
        self._tree.column("p4", width=45, anchor="center")
        self._tree.column("desc", width=220, anchor="w")

        self._tree.pack(side="left", fill="both", expand=True)

        scroll = ttk.Scrollbar(tree_f, orient="vertical", command=self._tree.yview)
        self._tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")

        self._tree.bind("<ButtonRelease-1>", self._on_tree_click)

        # ── Lower Section: Multi-Plot Canvas & Controls ─────────────────────
        self._build_plots_panel()

        # ── Status / Log Footer ─────────────────────────────────────────────
        self._log_var = tk.StringVar(value="Ready.")
        status_bar = ttk.Label(self, textvariable=self._log_var, anchor="w", relief="sunken", font=("Segoe UI", 8), foreground="#d4d4d8")
        status_bar.grid(row=4, column=0, sticky="ew", padx=8, pady=(2, 4))

    def _update_tree(self):
        search_term = self._search_var.get().lower()
        selected_item = self._tree.selection()

        for item in self._tree.get_children():
            self._tree.delete(item)

        for var in self._telemetry_vars:
            name = var["name"]
            vid = var["id"]
            unit = var["unit"]
            desc = var["description"]

            if search_term and (
                search_term not in name.lower()
                and search_term not in str(vid)
                and search_term not in desc.lower()
            ):
                continue

            p1_state = "☑" if vid in self._selected[0] else "☐"
            p2_state = "☑" if vid in self._selected[1] else "☐"
            p3_state = "☑" if vid in self._selected[2] else "☐"
            p4_state = "☑" if vid in self._selected[3] else "☐"
            val_str = self._current_vals_str.get(vid, "—")

            self._tree.insert("", "end", iid=str(vid), values=(
                name, vid, val_str, unit, p1_state, p2_state, p3_state, p4_state, desc
            ))

        if selected_item and self._tree.exists(selected_item[0]):
            self._tree.selection_set(selected_item[0])

        # Update active signals count
        active_ids = self._get_active_telemetry_ids()
        self._active_sig_count_lbl.config(text=f"Active MCU Streams: {len(active_ids)} / 10")

    def _get_active_telemetry_ids(self):
        """Returns union of signals selected across all currently active plots."""
        active = set()
        for i in range(self._num_plots):
            active |= self._selected[i]
        return sorted(list(active))

    def _on_tree_click(self, event):
        region = self._tree.identify_region(event.x, event.y)
        if region != "cell":
            return

        column = self._tree.identify_column(event.x)
        row_id = self._tree.identify_row(event.y)
        if not row_id:
            return

        vid = int(row_id)

        # Columns: #1=name, #2=id, #3=value, #4=unit, #5=p1, #6=p2, #7=p3, #8=p4, #9=desc
        col_map = {"#5": 0, "#6": 1, "#7": 2, "#8": 3}
        if column not in col_map:
            return

        plot_idx = col_map[column]

        if vid in self._selected[plot_idx]:
            self._selected[plot_idx].remove(vid)
        else:
            # Check total active signals across all active plots
            test_union = set()
            for i in range(self._num_plots):
                if i == plot_idx:
                    test_union |= (self._selected[i] | {vid})
                else:
                    test_union |= self._selected[i]

            if len(test_union) > 10:
                self._log("Max 10 active signals stream limit reached across active plots!")
                return

            self._selected[plot_idx].add(vid)

        self._rebuild_axes_and_cache()
        self._update_tree()

    def _build_plots_panel(self):
        plot_frame = ttk.LabelFrame(self, text="Real-Time Telemetry Plots")
        plot_frame.grid(row=3, column=0, sticky="nsew", padx=8, pady=2)
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.rowconfigure(1, weight=1)

        # ── Top Control Bar for Plots ─────────────────────────────────────────
        top_ctrl = ttk.Frame(plot_frame)
        top_ctrl.grid(row=0, column=0, sticky="ew", padx=6, pady=3)

        # Plot Count Selector (1 to 4)
        ttk.Label(top_ctrl, text="Plot Count:", font=("Segoe UI", 9, "bold")).pack(side="left", padx=(2, 4))
        self._num_plots_var = tk.IntVar(value=self._num_plots)
        for n in (1, 2, 3, 4):
            rb = ttk.Radiobutton(top_ctrl, text=f"{n}", value=n, variable=self._num_plots_var, command=self._on_plot_count_changed)
            rb.pack(side="left", padx=3)

        ttk.Separator(top_ctrl, orient="vertical").pack(side="left", fill="y", padx=8, pady=2)

        # Window Time
        ttk.Label(top_ctrl, text="Window:").pack(side="left", padx=(2, 2))
        self._window_var = tk.StringVar(value="5.0")
        self._window_spin = ttk.Spinbox(top_ctrl, from_=0.5, to=60.0, increment=1.0, textvariable=self._window_var, width=5)
        self._window_spin.pack(side="left", padx=2)
        ttk.Label(top_ctrl, text="sec").pack(side="left", padx=(0, 8))

        # Pause / Resume Plotting
        self._plot_active_btn = ttk.Button(top_ctrl, text="⏸ Pause Plot", command=self._toggle_plot_active, width=12)
        self._plot_active_btn.pack(side="left", padx=4)

        # Clear Plot Data
        ttk.Button(top_ctrl, text="Clear Plots", command=self._reset_plots, width=10).pack(side="left", padx=4)

        ttk.Separator(top_ctrl, orient="vertical").pack(side="left", fill="y", padx=8, pady=2)

        # Logging Controls (Start/Stop Logging & Export CSV)
        self._log_btn = ttk.Button(top_ctrl, text="⏺ Start Logging", style="Rec.TButton", command=self._toggle_logging, width=14)
        self._log_btn.pack(side="left", padx=4)

        ttk.Button(top_ctrl, text="💾 Export CSV...", command=self._export_csv, width=13).pack(side="left", padx=4)

        # ── Main Matplotlib Canvas ───────────────────────────────────────────
        self._fig = Figure(figsize=(11, 4.2), dpi=96, facecolor="#18181b")
        self._canvas = FigureCanvasTkAgg(self._fig, master=plot_frame)
        self._canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew", padx=4, pady=2)

        # Matplotlib Navigation Toolbar (Zoom, Pan, Home, etc.)
        tb_frame = ttk.Frame(plot_frame)
        tb_frame.grid(row=2, column=0, sticky="ew", padx=4, pady=1)
        self._toolbar = NavigationToolbar2Tk(self._canvas, tb_frame, pack_toolbar=False)
        self._toolbar.update()
        self._toolbar.pack(side="left", fill="x", expand=True)

        # ── Per-Plot Configuration Subpanel (Same Axis, Auto/Manual Limits) ──
        self._plot_settings_frame = ttk.Frame(plot_frame)
        self._plot_settings_frame.grid(row=3, column=0, sticky="ew", padx=6, pady=(2, 4))
        self._build_plot_settings_ui()

    def _build_plot_settings_ui(self):
        """Builds individual Y-axis & Same-Axis controls for each active plot."""
        for child in self._plot_settings_frame.winfo_children():
            child.destroy()

        for i in range(self._num_plots):
            p_box = ttk.Frame(self._plot_settings_frame)
            p_box.pack(side="left", fill="x", expand=True, padx=4, pady=2)

            ttk.Label(p_box, text=f"P{i+1}:", font=("Segoe UI", 9, "bold"), foreground=PLOT_COLORS[i % len(PLOT_COLORS)]).pack(side="left", padx=2)

            # Same Axis vs Dual Axis Checkbox
            same_cb = ttk.Checkbutton(p_box, text="Same Axis", variable=self._same_axis_vars[i], command=self._rebuild_axes_and_cache)
            same_cb.pack(side="left", padx=2)

            # Primary Y Auto Scale
            auto_cb = ttk.Checkbutton(p_box, text="Auto Y", variable=self._p_auto_vars[i], command=self._rebuild_axes_and_cache)
            auto_cb.pack(side="left", padx=2)

            # Primary Y Min / Max
            min_ent = ttk.Entry(p_box, textvariable=self._p_min_vars[i], width=5, justify="right")
            min_ent.pack(side="left", padx=1)
            ttk.Label(p_box, text="..").pack(side="left", padx=1)
            max_ent = ttk.Entry(p_box, textvariable=self._p_max_vars[i], width=5, justify="right")
            max_ent.pack(side="left", padx=1)

            min_ent.bind("<Return>", lambda e: self._rebuild_axes_and_cache())
            max_ent.bind("<Return>", lambda e: self._rebuild_axes_and_cache())

    def _on_plot_count_changed(self):
        self._num_plots = self._num_plots_var.get()
        self._build_plot_settings_ui()
        self._update_tree()
        self._rebuild_axes_and_cache()
        self._log(f"Configured layout for {self._num_plots} plot(s).")

    def _get_current_view_data(self):
        head = self._history_head
        if self._total_samples_received < self.PLOT_WINDOW_POINTS:
            active_data = self._history_data[:head]
            pad_len = self.PLOT_WINDOW_POINTS - len(active_data)
            return np.vstack((np.zeros((pad_len, PAYLOAD_N)), active_data))
        else:
            return np.vstack((self._history_data[head:], self._history_data[:head]))

    def _rebuild_axes_and_cache(self):
        self._fig.clf()
        self._ax.clear()
        self._ax_twin.clear()
        for i in range(4):
            self._active_lines[i].clear()

        # Grid layouts based on plot count
        if self._num_plots == 1:
            self._fig.subplots_adjust(left=0.08, right=0.95, top=0.88, bottom=0.15)
            self._ax = [self._fig.add_subplot(1, 1, 1)]
        elif self._num_plots == 2:
            self._fig.subplots_adjust(left=0.06, right=0.96, top=0.88, bottom=0.15, wspace=0.28)
            self._ax = [self._fig.add_subplot(1, 2, 1), self._fig.add_subplot(1, 2, 2)]
        elif self._num_plots == 3:
            self._fig.subplots_adjust(left=0.05, right=0.97, top=0.88, bottom=0.15, wspace=0.30)
            self._ax = [self._fig.add_subplot(1, 3, 1), self._fig.add_subplot(1, 3, 2), self._fig.add_subplot(1, 3, 3)]
        else:  # 4 plots: 2x2 grid
            self._fig.subplots_adjust(left=0.06, right=0.96, top=0.92, bottom=0.10, wspace=0.28, hspace=0.35)
            self._ax = [
                self._fig.add_subplot(2, 2, 1),
                self._fig.add_subplot(2, 2, 2),
                self._fig.add_subplot(2, 2, 3),
                self._fig.add_subplot(2, 2, 4),
            ]

        # Style primary axes
        for ax in self._ax:
            ax.set_facecolor("#22222a")
            ax.set_xlim(0, self.PLOT_WINDOW_POINTS)
            ax.grid(True, color="#3f3f46", linestyle="--", alpha=0.35)
            ax.tick_params(colors="#d4d4d8", labelsize=7)
            for spine in ax.spines.values():
                spine.set_color("#3f3f46")

        view_data = self._get_current_view_data()
        active_ids = self._get_active_telemetry_ids()

        if self._reader:
            self._reader.set_active_ids(active_ids)

        # Clear stale displayed values
        all_ids = [v["id"] for v in self._telemetry_vars]
        for vid in all_ids:
            if vid not in active_ids:
                self._current_vals_str[vid] = "—"
                if self._tree.exists(str(vid)):
                    self._tree.set(str(vid), "value", "—")

        # Color assignment index across plots
        global_color_idx = 0

        for p_idx in range(self._num_plots):
            ax_primary = self._ax[p_idx]
            selected_ids = sorted(list(self._selected[p_idx]))
            same_axis = self._same_axis_vars[p_idx].get()

            ax_secondary = None
            primary_names = []
            secondary_names = []

            # In Dual-Axis mode: 1st signal on left axis, 2nd+ signals on right axis
            if not same_axis and len(selected_ids) >= 2:
                ax_secondary = ax_primary.twinx()
                ax_secondary.tick_params(colors="#d4d4d8", labelsize=7)
                for spine in ax_secondary.spines.values():
                    spine.set_color("#3f3f46")
                self._ax_twin.append(ax_secondary)
            else:
                self._ax_twin.append(None)

            for sig_idx, vid in enumerate(selected_ids):
                if vid not in self._telemetry_by_id or vid not in active_ids:
                    continue

                var_info = self._telemetry_by_id[vid]
                name = var_info["name"]
                scale = var_info["scale"]
                col_idx = active_ids.index(vid)
                color = PLOT_COLORS[global_color_idx % len(PLOT_COLORS)]
                global_color_idx += 1

                # Determine if this signal goes to secondary axis
                is_twin = (not same_axis and sig_idx >= 1 and ax_secondary is not None)
                target_ax = ax_secondary if is_twin else ax_primary

                # Note: animated=True for live blitting when active; False when paused
                line, = target_ax.plot(
                    self._x_indices,
                    view_data[:, col_idx] / scale,
                    color=color,
                    linewidth=1.3,
                    animated=self._plot_active,
                    label=f"{name} ({var_info['unit']})" if var_info['unit'] else name,
                )

                self._active_lines[p_idx].append((col_idx, scale, line, is_twin))

                if is_twin:
                    secondary_names.append(name)
                    ax_secondary.tick_params(axis='y', labelcolor=color)
                else:
                    primary_names.append(name)

            # Titles & Legends
            if primary_names or secondary_names:
                all_names = primary_names + secondary_names
                ax_primary.set_title(f"P{p_idx+1}: " + ", ".join(all_names), fontsize=8, color="#f4f4f5", pad=4)

                # Combine legends from both axes if twin exists
                lines_all = [tpl[2] for tpl in self._active_lines[p_idx]]
                labels_all = [l.get_label() for l in lines_all]
                if len(lines_all) > 1:
                    leg = ax_primary.legend(lines_all, labels_all, fontsize=7, loc="upper left", facecolor="#27272a", edgecolor="#3f3f46")
                    for text in leg.get_texts():
                        text.set_color("#f4f4f5")
            else:
                ax_primary.set_title(f"Plot {p_idx+1}: (No signals routed)", fontsize=8, color="#71717a", pad=4)

            # Primary Axis Scaling
            if not self._p_auto_vars[p_idx].get():
                try:
                    ax_primary.set_ylim(float(self._p_min_vars[p_idx].get()), float(self._p_max_vars[p_idx].get()))
                except ValueError:
                    ax_primary.set_ylim(-10, 10)
            else:
                ax_primary.relim()
                ax_primary.autoscale_view()

            # Secondary Axis Scaling (if dual axis active)
            if ax_secondary is not None:
                ax_secondary.relim()
                ax_secondary.autoscale_view()

        self._canvas.draw()
        if self._plot_active:
            self._bg_cache = self._canvas.copy_from_bbox(self._fig.bbox)
        self._send_select_command()

    def _update_plot_views(self):
        """High-speed blitted plot update for active waveforms."""
        if not self._plot_active or self._toolbar.mode != "":
            return

        if self._bg_cache is None:
            return

        view_data = self._get_current_view_data()

        # Update line data for all plots
        for p_idx in range(self._num_plots):
            for col_idx, scale, line, is_twin in self._active_lines[p_idx]:
                line.set_data(self._x_indices, view_data[:, col_idx] / scale)

        # Throttled auto-scaling
        self._y_scale_ticks += 1
        if self._y_scale_ticks >= 15:
            self._y_scale_ticks = 0
            limits_changed = False

            for p_idx in range(self._num_plots):
                primary_lines = [l for l in self._active_lines[p_idx] if not l[3]]
                secondary_lines = [l for l in self._active_lines[p_idx] if l[3]]

                # Primary Axis Auto-scaling
                if self._p_auto_vars[p_idx].get() and primary_lines:
                    vals = [view_data[:, c_idx] / sc for c_idx, sc, _, _ in primary_lines]
                    y_min = min(arr.min() for arr in vals)
                    y_max = max(arr.max() for arr in vals)
                    y_range = max(0.1, y_max - y_min)
                    new_ylim = (y_min - y_range * 0.1, y_max + y_range * 0.1)

                    curr_ylim = self._ax[p_idx].get_ylim()
                    if abs(curr_ylim[0] - new_ylim[0]) > (y_range * 0.05) or abs(curr_ylim[1] - new_ylim[1]) > (y_range * 0.05):
                        self._ax[p_idx].set_ylim(new_ylim)
                        self._p_min_vars[p_idx].set(f"{new_ylim[0]:.1f}")
                        self._p_max_vars[p_idx].set(f"{new_ylim[1]:.1f}")
                        limits_changed = True

                # Secondary Axis Auto-scaling (Dual-Axis)
                if secondary_lines and self._ax_twin[p_idx] is not None:
                    vals_sec = [view_data[:, c_idx] / sc for c_idx, sc, _, _ in secondary_lines]
                    y_min = min(arr.min() for arr in vals_sec)
                    y_max = max(arr.max() for arr in vals_sec)
                    y_range = max(0.05, y_max - y_min)
                    new_ylim = (y_min - y_range * 0.1, y_max + y_range * 0.1)

                    curr_ylim = self._ax_twin[p_idx].get_ylim()
                    if abs(curr_ylim[0] - new_ylim[0]) > (y_range * 0.05) or abs(curr_ylim[1] - new_ylim[1]) > (y_range * 0.05):
                        self._ax_twin[p_idx].set_ylim(new_ylim)
                        limits_changed = True

            if limits_changed:
                self._canvas.draw()
                self._bg_cache = self._canvas.copy_from_bbox(self._fig.bbox)

        # Blit artists onto cached background
        self._canvas.restore_region(self._bg_cache)

        for p_idx in range(self._num_plots):
            for _, _, line, is_twin in self._active_lines[p_idx]:
                if is_twin and self._ax_twin[p_idx] is not None:
                    self._ax_twin[p_idx].draw_artist(line)
                else:
                    self._ax[p_idx].draw_artist(line)

        self._canvas.blit(self._fig.bbox)
        self._canvas.flush_events()

    def _start_loops(self):
        self._periodic_ui_update()
        self._tick_rate()

    def _periodic_ui_update(self):
        if self._reader:
            new_data, samples, total_frames = self._reader.get_new_samples()
            active_ids = self._get_active_telemetry_ids()

            if new_data and samples:
                last_sample = samples[-1]

                # Update live value displays in treeview
                for idx, vid in enumerate(active_ids):
                    if idx < len(last_sample) and vid in self._telemetry_by_id:
                        scale = self._telemetry_by_id[vid]["scale"]
                        val = last_sample[idx] / scale
                        decimals = 3 if scale >= 1000.0 else 1
                        val_str = f"{val:.{decimals}f}"
                        self._current_vals_str[vid] = val_str

                        if self._tree.exists(str(vid)):
                            self._tree.set(str(vid), "value", val_str)

                # Append to Data Logging buffer if logging is active
                if self._logging_active:
                    now = time.time() - self._log_start_time
                    for s in samples:
                        # Store timestamp, sample frame, and scaled values
                        row_vals = []
                        for idx, vid in enumerate(active_ids):
                            if idx < len(s):
                                row_vals.append(s[idx] / self._telemetry_by_id[vid]["scale"])
                            else:
                                row_vals.append(0.0)
                        self._log_records.append((now, total_frames, row_vals))

                    self._log_status_lbl.config(
                        text=f"Log: REC ({len(self._log_records):,} samples)",
                        foreground="#f43f5e"
                    )

                # Downsample & push into circular plot buffer
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
                            full_state = np.zeros(PAYLOAD_N, dtype=np.float32)
                            for idx, val in enumerate(sample):
                                if idx < PAYLOAD_N:
                                    full_state[idx] = val
                            self._history_data[self._history_head] = full_state
                            self._history_head = (self._history_head + 1) % self.PLOT_WINDOW_POINTS
                            self._total_samples_received += 1

                # RX Statistics
                delta_frames = total_frames - self._last_gui_frame_count
                self._last_gui_frame_count = total_frames
                self._rx_rate_count += delta_frames
                self._rx_count_var.set(f"{total_frames} frames")
                self._rx_led.config(foreground="#10b981")

                if self._plot_active:
                    self._update_plot_views()
            else:
                self._rx_led.config(foreground="gray")

        # When plotting is paused: update line buffers without blit, allowing smooth Toolbar Zoom/Pan
        if not self._plot_active:
            view_data = self._get_current_view_data()
            for p_idx in range(self._num_plots):
                for col_idx, scale, line, _ in self._active_lines[p_idx]:
                    line.set_data(self._x_indices, view_data[:, col_idx] / scale)

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
            self._status_lbl.config(text="● Disconnected", foreground="#ef4444")
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
            self._status_lbl.config(text=f"● {port}", foreground="#10b981")
            self._log(f"Connected to {port} @ {baud}")

    def _on_serial_error(self):
        self.after(0, self._handle_serial_crash)

    def _handle_serial_crash(self):
        err = self._reader.error if self._reader else "Unknown fault"
        self._status_lbl.config(text="● Error", foreground="#ef4444")
        self._log(f"Serial error: {err}")
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._conn_btn.config(text="Connect")

    def _send_command(self):
        if not self._reader:
            self._log("Not connected to MCU.")
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

        active_ids = self._get_active_telemetry_ids()
        padded_ids = list(active_ids) + [0] * (10 - len(active_ids))
        padded_ids = padded_ids[:10]

        USB_SELECT_MAGIC = 0xABCE
        payload_padding = b'\x00' * 20
        frame = struct.pack('<HH10H20s', USB_SELECT_MAGIC, self._seq & 0xFFFF, *padded_ids, payload_padding)
        self._seq += 1
        self._reader.send(frame)

    def _start_motor(self):
        """Starts the motor by setting mEnable=1 without resetting plots or logging."""
        if len(self._tx_vars) > 0:
            self._tx_vars[0].set("1")  # Slot 0 is mEnable
        self._send_command()
        self._log("START Motor sent — inverter enabled.")

    def _stop_motor(self):
        """Stops the motor by setting mEnable=0 without interrupting plots or logging."""
        if len(self._tx_vars) > 0:
            self._tx_vars[0].set("0")  # Slot 0 is mEnable
        self._send_command()
        self._log("STOP Motor sent — inverter disabled.")

    def _reset_plots(self):
        """Explicit action to clear waveform plot history."""
        self._history_head = 0
        self._total_samples_received = 0
        self._downsample_counter = 0
        self._y_scale_ticks = 0
        self._history_data.fill(0)
        self._rebuild_axes_and_cache()
        self._log("Plots cleared.")

    def _toggle_plot_active(self):
        """Pauses/resumes plotting. When paused, disables animated flags so Zoom/Pan works natively."""
        self._plot_active = not self._plot_active

        if self._plot_active:
            # Resuming live blit animation
            if self._toolbar.mode != "":
                if self._toolbar.mode == "zoom in":
                    self._toolbar.zoom()
                elif self._toolbar.mode == "pan/zoom":
                    self._toolbar.pan()

            self._plot_active_btn.config(text="⏸ Pause Plot")

            # Set animated=True for fast blitting
            for p_idx in range(self._num_plots):
                for _, _, line, _ in self._active_lines[p_idx]:
                    line.set_animated(True)

            self._rebuild_axes_and_cache()
            self._log("Plotting resumed.")
        else:
            # Pausing live animation — switch lines to animated=False for full interactive Zoom & Pan
            self._plot_active_btn.config(text="▶ Resume Plot")

            for p_idx in range(self._num_plots):
                for _, _, line, _ in self._active_lines[p_idx]:
                    line.set_animated(False)

            self._canvas.draw()
            self._log("Plotting paused — Interactive Zoom and Pan enabled.")

    def _toggle_logging(self):
        """Starts or stops continuous telemetry logging to memory buffer."""
        self._logging_active = not self._logging_active

        if self._logging_active:
            self._log_records.clear()
            self._log_start_time = time.time()
            self._log_btn.config(text="⏹ Stop Logging", style="Stop.TButton")
            self._log_status_lbl.config(text="Log: REC (0 samples)", foreground="#f43f5e")
            self._log("Telemetry logging started.")
        else:
            self._log_btn.config(text="⏺ Start Logging", style="Rec.TButton")
            self._log_status_lbl.config(text=f"Log: Saved {len(self._log_records):,} pts", foreground="#10b981")
            self._log(f"Logging stopped. {len(self._log_records):,} samples recorded in memory buffer.")

    def _export_csv(self):
        """Exports recorded logging stream or current plot window buffer to a CSV file."""
        active_ids = self._get_active_telemetry_ids()
        if not active_ids:
            self._log("No active telemetry signals to export.")
            return

        default_name = f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        filename = filedialog.asksaveasfilename(
            title="Export Telemetry Data to CSV",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
        if not filename:
            return

        try:
            with open(filename, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)

                # Column headers
                headers = ["Time_s", "Sample_Index"]
                for vid in active_ids:
                    var_info = self._telemetry_by_id.get(vid, {"name": f"ID_{vid}", "unit": ""})
                    unit_str = f" [{var_info['unit']}]" if var_info["unit"] else ""
                    headers.append(f"{var_info['name']}{unit_str}")
                writer.writerow(headers)

                # Export recorded log stream if available, otherwise export plot history buffer
                if self._log_records:
                    for t_rel, seq, vals in self._log_records:
                        writer.writerow([f"{t_rel:.5f}", seq] + [f"{v:.4f}" for v in vals])
                    row_count = len(self._log_records)
                    self._log(f"Exported {row_count:,} recorded log samples to {os.path.basename(filename)}")
                else:
                    view_data = self._get_current_view_data()
                    try:
                        w_sec = float(self._window_var.get())
                    except ValueError:
                        w_sec = 5.0
                    time_step = w_sec / self.PLOT_WINDOW_POINTS

                    for i in range(self.PLOT_WINDOW_POINTS):
                        t_sec = i * time_step
                        row = [f"{t_sec:.4f}", i]
                        for col_idx, vid in enumerate(active_ids):
                            scale = self._telemetry_by_id[vid]["scale"]
                            val = view_data[i, col_idx] / scale
                            row.append(f"{val:.4f}")
                        writer.writerow(row)
                    row_count = self.PLOT_WINDOW_POINTS
                    self._log(f"Exported {row_count} plot buffer samples to {os.path.basename(filename)}")

        except Exception as e:
            self._log(f"Failed to export CSV: {e}")

    def _log(self, msg: str):
        self._log_var.set(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _on_close(self):
        if self._reader:
            self._reader.stop()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()
