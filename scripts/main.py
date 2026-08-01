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
import os
import json

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
            {"name": "mIsAbs_mA", "type": "float", "default": 0.0}
        ],
        "telemetry": [
            {"id": 1, "name": "Udc_V", "scale": 100.0, "unit": "V", "description": "DC Link Bus Voltage"},
            {"id": 2, "name": "demandSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Demand Electrical Speed"},
            {"id": 3, "name": "feedbackSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Feedback Rotor Speed"},
            {"id": 4, "name": "encoderSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Encoder Measured Speed"},
            {"id": 5, "name": "observerSpeed_rpm", "scale": 1.0, "unit": "rpm", "description": "Observer Estimated Speed"},
            {"id": 7, "name": "Id_A", "scale": 1000.0, "unit": "A", "description": "D-axis feedback current"},
            {"id": 8, "name": "Iq_A", "scale": 1000.0, "unit": "A", "description": "Q-axis feedback current"}
        ]
    }

RX_MAGIC     = 0xABCD
TX_MAGIC     = 0xDCBA
SAMPLE_BYTES = 3
PAYLOAD_N    = 10

# Map C++ variable names to user friendly labels, scales, units
CMD_MAP = {
    "mEnable": ("Enable", 1.0, ""),
    "mMode": ("Mode", 1.0, ""),
    "targetSpeed_rpm": ("Target Speed", 100.0, "rpm"),
    "mAcceleration_rpm_s": ("Accel", 100.0, "rpm/s"),
    "mIsAbs_mA": ("Current Limit", 10.0, "mA"),
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
            except:
                pass

# ── GUI App ───────────────────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PMSM FOC — Motor Control (Dynamic Telemetry)")
        self.resizable(False, False)
        self._reader = None
        self._seq    = 0
        
        self._selected_p1 = {1}
        self._selected_p2 = {7, 8}
        self._current_vals_str = {}
        
        # Resolve registry JSON file path relative to main.py
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        default_json = os.path.join(project_root, "app", "telemetry_registry.json")
        
        self._telemetry_vars = []
        self._telemetry_by_id = {}
        
        if os.path.exists(default_json):
            self._load_json_file(default_json)
        else:
            self._load_fallback_registry()
        
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
        
        self._apply_style()
        self._build_ui()
        self._update_tree()
        self._refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        
        self.after(500, self._rebuild_axes_and_cache)
        self._start_loops()

    def _load_json_dialog(self):
        from tkinter import filedialog
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
            
            # Sanitize selections against new registry IDs
            all_ids = set(v["id"] for v in self._telemetry_vars)
            self._selected_p1 = {vid for vid in self._selected_p1 if vid in all_ids}
            self._selected_p2 = {vid for vid in self._selected_p2 if vid in all_ids}
            
            # Rebuild defaults if selections are empty
            if not self._selected_p1 and self._telemetry_vars:
                self._selected_p1 = {self._telemetry_vars[0]["id"]}
            if not self._selected_p2 and len(self._telemetry_vars) > 1:
                self._selected_p2 = {self._telemetry_vars[1]["id"]}
                
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
            {"id": 8, "name": "Iq_A", "scale": 1000.0, "unit": "A", "description": "Q-axis Current"}
        ]
        self._telemetry_by_id = {v["id"]: v for v in self._telemetry_vars}

    def _apply_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        
        # Elegant Dark Modern Color System
        bg_main = "#1e1e24"
        bg_card = "#2a2b36"
        fg_main = "#f4f4f5"
        accent_indigo = "#6366f1"
        
        self.configure(bg=bg_main)
        
        style.configure(".", background=bg_main, foreground=fg_main, fieldbackground=bg_card, font=("Segoe UI", 9))
        
        # LabelFrames
        style.configure("TLabelframe", background=bg_main, bordercolor="#3f3f46", borderwidth=1, relief="solid")
        style.configure("TLabelframe.Label", background=bg_main, foreground=accent_indigo, font=("Segoe UI", 9, "bold"))
        
        # Frames
        style.configure("TFrame", background=bg_main)
        
        # Buttons
        style.configure("TButton", background=bg_card, foreground=fg_main, borderwidth=1, bordercolor="#3f3f46", relief="flat", padding=(10, 4))
        style.map("TButton",
                  background=[("active", accent_indigo), ("pressed", "#4f46e5")],
                  foreground=[("active", "#ffffff")])
                  
        # Entries
        style.configure("TEntry", fieldbackground=bg_card, foreground=fg_main, bordercolor="#3f3f46", borderwidth=1)
        style.configure("TSpinbox", fieldbackground=bg_card, foreground=fg_main, bordercolor="#3f3f46", borderwidth=1)
        style.configure("TCombobox", fieldbackground=bg_card, foreground=fg_main, bordercolor="#3f3f46", borderwidth=1)
        
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
                        font=("Segoe UI", 9, "bold"),
                        borderwidth=1,
                        bordercolor="#3f3f46")
        style.map("Treeview",
                  background=[("selected", accent_indigo)],
                  foreground=[("selected", "#ffffff")])

    def _build_ui(self):
        pad = {"padx": 5, "pady": 3}
        bg_main = "#1e1e24"

        # Connection panel
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

        ttk.Button(conn, text="Load JSON", command=self._load_json_dialog).grid(row=0, column=6, **pad)

        self._status_lbl = ttk.Label(conn, text="● Disconnected", foreground="#ef4444", width=22)
        self._status_lbl.grid(row=0, column=7, **pad)

        # Commands Panel
        tx_frame = ttk.LabelFrame(self, text="Commands  (PC → MCU)")
        tx_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=4)

        ttk.Label(tx_frame, text="#",     width=2,  anchor="center", font=("Segoe UI", 8, "bold"), foreground="#a1a1aa").grid(row=0, column=0)
        ttk.Label(tx_frame, text="Name",  width=16, anchor="w",      font=("Segoe UI", 8, "bold"), foreground="#a1a1aa").grid(row=0, column=1)
        ttk.Label(tx_frame, text="Value", width=12, anchor="center", font=("Segoe UI", 8, "bold"), foreground="#a1a1aa").grid(row=0, column=2)
        ttk.Label(tx_frame, text="Unit",  width=5,  anchor="w",      font=("Segoe UI", 8, "bold"), foreground="#a1a1aa").grid(row=0, column=3)
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
            ttk.Label(tx_frame, text=unit, width=5, anchor="w", foreground="#a1a1aa").grid(row=row, column=3, **pad)

        send_row = PAYLOAD_N + 2
        btn_f = ttk.Frame(tx_frame)
        btn_f.grid(row=send_row, column=0, columnspan=4, pady=8)
        ttk.Button(btn_f, text="Send (Enter)", command=self._send_command, width=14).pack(side="left", padx=4)
        ttk.Button(btn_f, text="STOP Motor", command=self._stop_motor, width=12).pack(side="left", padx=4)

        # Telemetry Panel (Dynamic Searchable Treeview)
        rx_frame = ttk.LabelFrame(self, text="Telemetry  (MCU → PC)")
        rx_frame.grid(row=1, column=1, sticky="nsew", padx=8, pady=4)

        search_f = ttk.Frame(rx_frame)
        search_f.pack(fill="x", padx=6, pady=4)
        ttk.Label(search_f, text="Search:").pack(side="left", padx=2)
        self._search_var = tk.StringVar()
        self._search_var.trace_add("write", lambda *args: self._update_tree())
        search_ent = ttk.Entry(search_f, textvariable=self._search_var, width=15)
        search_ent.pack(side="left", padx=2)
        ttk.Button(search_f, text="Clear", command=lambda: self._search_var.set(""), width=6).pack(side="left", padx=2)

        tree_f = ttk.Frame(rx_frame)
        tree_f.pack(fill="both", expand=True, padx=6, pady=4)

        self._tree = ttk.Treeview(tree_f, columns=("name", "id", "value", "unit", "p1", "p2", "desc"), show="headings", height=12)
        self._tree.heading("name", text="Name", anchor="w")
        self._tree.heading("id", text="ID", anchor="center")
        self._tree.heading("value", text="Value", anchor="e")
        self._tree.heading("unit", text="Unit", anchor="w")
        self._tree.heading("p1", text="Plot 1", anchor="center")
        self._tree.heading("p2", text="Plot 2", anchor="center")
        self._tree.heading("desc", text="Description", anchor="w")

        self._tree.column("name", width=120, anchor="w")
        self._tree.column("id", width=30, anchor="center")
        self._tree.column("value", width=70, anchor="e")
        self._tree.column("unit", width=40, anchor="w")
        self._tree.column("p1", width=50, anchor="center")
        self._tree.column("p2", width=50, anchor="center")
        self._tree.column("desc", width=160, anchor="w")

        self._tree.pack(side="left", fill="both", expand=True)

        scroll = ttk.Scrollbar(tree_f, orient="vertical", command=self._tree.yview)
        self._tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")

        self._tree.bind("<ButtonRelease-1>", self._on_tree_click)

        # Footer
        rx_ind = ttk.Frame(self)
        rx_ind.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=2)

        ttk.Label(rx_ind, text="RX:").pack(side="left", padx=(0, 4))
        self._rx_led = tk.Label(rx_ind, text="●", foreground="gray", bg=bg_main, font=("Segoe UI", 14), width=2)
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
            
            if search_term and (search_term not in name.lower() and 
                                search_term not in str(vid) and 
                                search_term not in desc.lower()):
                continue
                
            p1_state = "☑" if vid in self._selected_p1 else "☐"
            p2_state = "☑" if vid in self._selected_p2 else "☐"
            val_str = self._current_vals_str.get(vid, "—")
            
            self._tree.insert("", "end", iid=str(vid), values=(
                name, vid, val_str, unit, p1_state, p2_state, desc
            ))
            
        if selected_item and self._tree.exists(selected_item[0]):
            self._tree.selection_set(selected_item[0])

    def _on_tree_click(self, event):
        region = self._tree.identify_region(event.x, event.y)
        if region != "cell":
            return
            
        column = self._tree.identify_column(event.x)
        row_id = self._tree.identify_row(event.y)
        if not row_id:
            return
            
        vid = int(row_id)
        
        if column == "#5":  # Plot 1
            if vid in self._selected_p1:
                self._selected_p1.remove(vid)
            else:
                total_active = len(self._selected_p1 | self._selected_p2 | {vid})
                if total_active > 10:
                    self._log("Max 10 active signals selection reached!")
                    return
                self._selected_p1.add(vid)
            self._rebuild_axes_and_cache()
            self._update_tree()
            
        elif column == "#6":  # Plot 2
            if vid in self._selected_p2:
                self._selected_p2.remove(vid)
            else:
                total_active = len(self._selected_p1 | self._selected_p2 | {vid})
                if total_active > 10:
                    self._log("Max 10 active signals selection reached!")
                    return
                self._selected_p2.add(vid)
            self._rebuild_axes_and_cache()
            self._update_tree()

    def _build_plots_panel(self):
        plot_frame = ttk.LabelFrame(self, text="Plots")
        plot_frame.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=8, pady=4)

        # 1. Main Canvas Setup
        self._fig = Figure(figsize=(11, 3.8), dpi=96, facecolor="#1e1e24")
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

        # Core active dictionary references for blitted fast loops
        self._active_lines_p1 = []
        self._active_lines_p2 = []
        self._x_indices = np.arange(self.PLOT_WINDOW_POINTS)

    def _get_current_view_data(self):
        head = self._history_head
        if self._total_samples_received < self.PLOT_WINDOW_POINTS:
            active_data = self._history_data[:head]
            pad_len = self.PLOT_WINDOW_POINTS - len(active_data)
            return np.vstack((np.zeros((pad_len, PAYLOAD_N)), active_data))
        else:
            return np.vstack((self._history_data[head:], self._history_data[:head]))

    def _rebuild_axes_and_cache(self):
        COLORS = ["#3b82f6", "#10b981", "#6366f1", "#f59e0b", "#ec4899", "#8b5cf6", "#14b8a6", "#f43f5e", "#84cc16", "#06b6d4"]

        self._ax[0].cla()
        self._ax[1].cla()

        for ax in self._ax:
            ax.set_facecolor("#2a2b36")
            ax.set_xlim(0, self.PLOT_WINDOW_POINTS)
            ax.grid(True, color="#4b5563", linestyle="--", alpha=0.3)
            ax.tick_params(colors="#e4e4e7", labelsize=7)
            for spine in ax.spines.values():
                spine.set_color("#4b5563")

        view_data = self._get_current_view_data()
        active_ids = sorted(list(self._selected_p1 | self._selected_p2))

        if self._reader:
            self._reader.set_active_ids(active_ids)

        # Clear values of inactive IDs
        all_ids = [v["id"] for v in self._telemetry_vars]
        for vid in all_ids:
            if vid not in active_ids:
                self._current_vals_str[vid] = "—"
                if self._tree.exists(str(vid)):
                    self._tree.set(str(vid), "value", "—")

        # --- Reconstruct Left Plot (Plot 1) ---
        self._active_lines_p1.clear()
        selected_p1_names = []
        color_idx = 0
        for vid in sorted(list(self._selected_p1)):
            var_info = self._telemetry_by_id[vid]
            name = var_info["name"]
            scale = var_info["scale"]
            selected_p1_names.append(name)
            
            col_idx = active_ids.index(vid)
            line, = self._ax[0].plot(self._x_indices, view_data[:, col_idx] / scale, 
                                     color=COLORS[color_idx % len(COLORS)], linewidth=1.2, 
                                     animated=True, label=name)
            self._active_lines_p1.append((col_idx, scale, line))
            color_idx += 1
        
        if selected_p1_names:
            self._ax[0].set_title(", ".join(selected_p1_names), fontsize=8, color="#e4e4e7")
            if len(selected_p1_names) > 1:
                legend = self._ax[0].legend(fontsize=7, loc="upper left", facecolor="#2a2b36", edgecolor="#4b5563")
                for text in legend.get_texts():
                    text.set_color("#e4e4e7")
        else:
            self._ax[0].set_title("No Signals Selected", fontsize=8, color="#e4e4e7")

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
        for vid in sorted(list(self._selected_p2)):
            var_info = self._telemetry_by_id[vid]
            name = var_info["name"]
            scale = var_info["scale"]
            selected_p2_names.append(name)
            
            col_idx = active_ids.index(vid)
            line, = self._ax[1].plot(self._x_indices, view_data[:, col_idx] / scale, 
                                     color=COLORS[color_idx % len(COLORS)], linewidth=1.2, 
                                     animated=True, label=name)
            self._active_lines_p2.append((col_idx, scale, line))
            color_idx += 1

        if selected_p2_names:
            self._ax[1].set_title(", ".join(selected_p2_names), fontsize=8, color="#e4e4e7")
            if len(selected_p2_names) > 1:
                legend = self._ax[1].legend(fontsize=7, loc="upper left", facecolor="#2a2b36", edgecolor="#4b5563")
                for text in legend.get_texts():
                    text.set_color("#e4e4e7")
        else:
            self._ax[1].set_title("No Signals Selected", fontsize=8, color="#e4e4e7")

        if not self._p2_auto_var.get():
            try:
                self._ax[1].set_ylim(float(self._p2_min_var.get()), float(self._p2_max_var.get()))
            except ValueError:
                self._ax[1].set_ylim(-1, 1)
        else:
            self._ax[1].relim()
            self._ax[1].autoscale_view()

        self._canvas.draw()
        self._bg_cache = self._canvas.copy_from_bbox(self._fig.bbox)
        self._send_select_command()

    def _update_plot_views(self):
        if not self._plot_active or self._toolbar.mode != "":
            return

        if self._bg_cache is None:
            return

        view_data = self._get_current_view_data()

        for idx, scale, line in self._active_lines_p1:
            line.set_data(self._x_indices, view_data[:, idx] / scale)

        for idx, scale, line in self._active_lines_p2:
            line.set_data(self._x_indices, view_data[:, idx] / scale)

        self._y_scale_ticks += 1
        if self._y_scale_ticks >= 15:
            self._y_scale_ticks = 0
            limits_changed = False

            if self._p1_auto_var.get() and self._active_lines_p1:
                vals = [view_data[:, idx] / scale for idx, scale, _ in self._active_lines_p1]
                y_min = min(arr.min() for arr in vals)
                y_max = max(arr.max() for arr in vals)
                y_range = max(0.1, y_max - y_min)
                new_ylim = (y_min - y_range * 0.1, y_max + y_range * 0.1)
                
                curr_ylim = self._ax[0].get_ylim()
                if abs(curr_ylim[0] - new_ylim[0]) > (y_range * 0.05) or abs(curr_ylim[1] - new_ylim[1]) > (y_range * 0.05):
                    self._ax[0].set_ylim(new_ylim)
                    self._p1_min_var.set(f"{new_ylim[0]:.1f}")
                    self._p1_max_var.set(f"{new_ylim[1]:.1f}")
                    limits_changed = True

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

        self._canvas.restore_region(self._bg_cache)

        for _, _, line in self._active_lines_p1:
            self._ax[0].draw_artist(line)

        for _, _, line in self._active_lines_p2:
            self._ax[1].draw_artist(line)

        self._canvas.blit(self._fig.bbox)
        self._canvas.flush_events()

    def _start_loops(self):
        self._periodic_ui_update()
        self._tick_rate()

    def _periodic_ui_update(self):
        if self._reader:
            new_data, samples, total_frames = self._reader.get_new_samples()
            active_ids = sorted(list(self._selected_p1 | self._selected_p2))
            
            if new_data and samples:
                last_sample = samples[-1]
                
                for idx, vid in enumerate(active_ids):
                    if idx < len(last_sample):
                        val = last_sample[idx] / self._telemetry_by_id[vid]["scale"]
                        decimals = 3 if self._telemetry_by_id[vid]["scale"] >= 1000.0 else 1
                        val_str = f"{val:.{decimals}f}"
                        self._current_vals_str[vid] = val_str
                        
                        if self._tree.exists(str(vid)):
                            self._tree.set(str(vid), "value", val_str)
                
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

                delta_frames = total_frames - self._last_gui_frame_count
                self._last_gui_frame_count = total_frames
                self._rx_rate_count += delta_frames
                self._rx_count_var.set(f"{total_frames} frames")
                self._rx_led.config(foreground="lime green")
                
                if self._plot_active:
                    self._update_plot_views()
            else:
                self._rx_led.config(foreground="gray")

        if not self._plot_active:
            view_data = self._get_current_view_data()
            for idx, scale, line in self._active_lines_p1:
                line.set_data(self._x_indices, view_data[:, idx] / scale)
            for idx, scale, line in self._active_lines_p2:
                line.set_data(self._x_indices, view_data[:, idx] / scale)
            
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
        
        active_ids = sorted(list(self._selected_p1 | self._selected_p2))
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
            self._log("Plotting paused.")

    def _log(self, msg: str):
        self._log_var.set(msg)

    def _on_close(self):
        if self._reader:
            self._reader.stop()
        self.destroy()

if __name__ == "__main__":
    app = App()
    app.mainloop()
