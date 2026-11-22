"""
Rhino Control Dashboard
=======================

Desktop operator console for the Rhino 5-DOF robotic arm, rebuilt from the
project's graduation documentation.

  - CustomTkinter UI (dark theme, single organized screen)
  - Teach pendant: jog J1-J4, track L and gripper G with live digital twin
  - Cartesian (IK) control with live validation and nearest-point suggestion
  - Sequence builder: record way-points (gripper angle + wait), save/load JSON
  - Real-time 3D digital twin (Matplotlib), full workspace view (Plotly)
  - PySerial worker thread: serial console logs commands in both directions

Run:  python3 rhino_dashboard.py
"""

import json
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox

import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import rhino_kinematics as K

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # allow the UI to open without pyserial installed
    serial = None
    list_ports = None

BAUD = 115200
ACCENT = "#d97b29"
GREEN = "#2fa572"
RED = "#c0392b"


# --------------------------------------------------------------------------
# Serial link (worker thread)
# --------------------------------------------------------------------------
class SerialLink:
    """Background serial reader/writer; the GUI stays responsive."""

    def __init__(self, on_line, on_state):
        self.on_line = on_line      # callback(direction, text)
        self.on_state = on_state    # callback(connected: bool)
        self.ser = None
        self.thread = None
        self.running = False
        self.tx_queue = queue.Queue()

    def ports(self):
        if list_ports is None:
            return []
        return [p.device for p in list_ports.comports()]

    def connect(self, port):
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()
        self.on_state(True)

    def disconnect(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.on_state(False)

    def send(self, text):
        """Queue a command line; returns the line actually sent."""
        line = text.strip() + "\n"
        if self.ser and self.running:
            self.tx_queue.put(line)
        self.on_line("TX", text.strip())

    def _worker(self):
        buf = b""
        while self.running and self.ser:
            try:
                while not self.tx_queue.empty():
                    self.ser.write(self.tx_queue.get_nowait().encode())
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode(errors="replace").strip()
                        if line:
                            self.on_line("RX", line)
                else:
                    time.sleep(0.02)
            except Exception as exc:  # cable pulled, port vanished ...
                self.on_line("RX", f"[serial error] {exc}")
                break
        self.on_state(False)


# --------------------------------------------------------------------------
# Digital twin (Matplotlib 3D)
# --------------------------------------------------------------------------
class Twin3D:
    def __init__(self, master):
        self.fig = Figure(figsize=(5.2, 4.2), facecolor="#1d2620")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_facecolor("#1d2620")
        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.toolbar = NavigationToolbar2Tk(self.canvas, master)
        self.toolbar.config(background="#1d2620")
        self.show_cloud = True
        self._cloud = None
        self.arm_color = ACCENT

    def widget(self):
        return self.canvas.get_tk_widget()

    def _style(self, warn):
        ax = self.ax
        ax.clear()
        ax.set_facecolor("#1d2620")
        for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
            pane.pane.set_facecolor("#1d2620")
        ax.tick_params(colors="white", labelsize=7)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.label.set_color("white")
        ax.set_xlabel("X side (cm)")
        ax.set_ylabel("Y fwd (cm)")
        ax.set_zlabel("Z height (cm)")
        ax.set_xlim(-30, 190)
        ax.set_ylim(-45, 45)
        ax.set_zlim(0, 90)
        title = "RHINO - WRIST LIMIT!" if warn else "RHINO"
        ax.set_title(title, color=RED if warn else "white", fontsize=10)

    def update(self, j1, j2, j3, j4, track):
        pts = K.fk_from_joints(j1, j2, j3, j4, track)
        warn = not pts["wrist_level"]
        self._style(warn)
        chain = [pts["base"], pts["shoulder"], pts["elbow"], pts["wrist"], pts["tip"]]
        xs = [p[0] for p in chain]
        ys = [p[1] for p in chain]
        zs = [p[2] for p in chain]
        color = RED if warn else ACCENT
        # track rail
        self.ax.plot([K.TRACK_MIN, K.TRACK_MAX], [0, 0], [0, 0],
                     color="#8a5a20", lw=8, solid_capstyle="round")
        self.ax.plot(xs, ys, zs, color=color, lw=5, solid_capstyle="round")
        self.ax.scatter(xs[1:], ys[1:], zs[1:], color=color, s=60,
                        edgecolors="white")
        if self.show_cloud:
            if self._cloud is None:
                self._cloud = K.workspace_cloud(1500)
            cx = [p[0] for p in self._cloud]
            cy = [p[1] for p in self._cloud]
            cz = [p[2] for p in self._cloud]
            self.ax.scatter(cx, cy, cz, s=1, c=cz, cmap="viridis", alpha=0.35)
        self.canvas.draw_idle()

    def reset_view(self):
        self.ax.view_init(elev=20, azim=-60)
        self.canvas.draw_idle()

    def open_full_workspace(self):
        import plotly.graph_objects as go
        pts = K.workspace_cloud(8000)
        fig = go.Figure(go.Scatter3d(
            x=[p[0] for p in pts], y=[p[1] for p in pts], z=[p[2] for p in pts],
            mode="markers",
            marker=dict(size=2, color=[p[2] for p in pts], colorscale="Viridis"),
        ))
        fig.update_layout(title="Rhino - reachable workspace (gripper level)",
                          scene=dict(xaxis_title="X side (cm)",
                                     yaxis_title="Y fwd (cm)",
                                     zaxis_title="Z up (cm)"))
        fig.show()


# --------------------------------------------------------------------------
# Main application
# --------------------------------------------------------------------------
class RhinoDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Rhino Control Dashboard")
        self.geometry("1480x900")
        ctk.set_appearance_mode("dark")

        self.link = SerialLink(self._on_line, self._on_state)
        self.connected = False
        self.mode = "SIM"              # SIM = no motion (safe)
        self.ir_trigger = False        # IR switch: run GUI sequence on detect
        self.state = {"j1": 130.0, "j2": 90.0, "j3": 90.0, "j4": 90.0,
                      "track": 0.0, "grip": K.GRIP_CLOSED}
        self.steps = []                # sequence builder rows
        self.seq_running = False
        self._seq_thread = None

        self._build_topbar()
        self._build_body()
        self._build_console()
        self._refresh_twin()
        self._validate_xyz()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- UI construction ----------------
    def _build_topbar(self):
        bar = ctk.CTkFrame(self)
        bar.pack(fill="x", padx=8, pady=(8, 4))
        ctk.CTkLabel(bar, text="RHINO", font=("Arial", 20, "bold"),
                     text_color=GREEN).pack(side="left", padx=10)

        ctk.CTkLabel(bar, text="Port:").pack(side="left", padx=(14, 2))
        self.port_var = tk.StringVar(value="(no ports found)")
        self.port_menu = ctk.CTkOptionMenu(bar, variable=self.port_var,
                                           values=self._scan_ports(), width=150)
        self.port_menu.pack(side="left")
        ctk.CTkButton(bar, text="\u27f3", width=32,
                      command=self._refresh_ports).pack(side="left", padx=4)
        self.conn_btn = ctk.CTkButton(bar, text="Connect", fg_color=GREEN,
                                      command=self._toggle_connection)
        self.conn_btn.pack(side="left", padx=6)
        self.status_lbl = ctk.CTkLabel(bar, text="\u25cf DISCONNECTED",
                                       text_color="grey")
        self.status_lbl.pack(side="left", padx=6)

        self.mode_btn = ctk.CTkButton(bar, text="MODE: SIM",
                                      fg_color="#8a6d1d", command=self._toggle_mode)
        self.mode_btn.pack(side="left", padx=10)
        ctk.CTkButton(bar, text="HOME", command=lambda: self._send("HOME")).pack(
            side="left", padx=3)
        ctk.CTkButton(bar, text="HOME1", command=lambda: self._send("HOME1")).pack(
            side="left", padx=3)
        ctk.CTkButton(bar, text="\u26d4 E-STOP", fg_color=RED,
                      command=self._estop).pack(side="right", padx=10)

    def _build_body(self):
        body = ctk.CTkFrame(self)
        body.pack(fill="both", expand=True, padx=8, pady=4)
        body.columnconfigure(0, weight=0, minsize=280)
        body.columnconfigure(1, weight=1)
        body.columnconfigure(2, weight=0, minsize=330)
        body.rowconfigure(0, weight=1)

        # ---- left column: teach pendant + cartesian ----
        left = ctk.CTkScrollableFrame(body)
        left.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self._build_teach(left)
        self._build_cartesian(left)
        self._build_gripper(left)
        self._build_pickplace(left)

        # ---- center: digital twin + readout ----
        center = ctk.CTkFrame(body)
        center.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        head = ctk.CTkFrame(center, fg_color="transparent")
        head.pack(fill="x")
        ctk.CTkLabel(head, text="3D Digital Twin (live FK - scroll=zoom, drag=rotate)",
                     font=("Arial", 13, "bold"), text_color=ACCENT).pack(side="left", padx=6)
        self.cloud_var = tk.BooleanVar(value=True)
        ctk.CTkSwitch(head, text="Cloud", variable=self.cloud_var,
                      command=self._toggle_cloud).pack(side="right", padx=4)
        ctk.CTkButton(head, text="Full Workspace (Plotly)", fg_color=ACCENT,
                      command=self._open_plotly).pack(side="right", padx=4)
        ctk.CTkButton(head, text="Reset View", width=90,
                      command=lambda: self.twin.reset_view()).pack(side="right", padx=4)

        self.twin = Twin3D(center)
        self.twin.toolbar.pack(fill="x")
        self.twin.widget().pack(fill="both", expand=True)

        self.readout = ctk.CTkFrame(center)
        self.readout.pack(fill="x", pady=4)
        self.readout_vals = {}
        for i, name in enumerate(("X", "Y", "Z", "J1", "J2", "J3", "J4")):
            ctk.CTkLabel(self.readout, text=name).grid(row=0, column=i, padx=10)
            lbl = ctk.CTkLabel(self.readout, text="-", font=("Consolas", 13, "bold"),
                               text_color=GREEN)
            lbl.grid(row=1, column=i, padx=10, pady=(0, 6))
            self.readout_vals[name] = lbl

        # ---- right column: sequence builder ----
        right = ctk.CTkFrame(body)
        right.grid(row=0, column=2, sticky="nsew", padx=4, pady=4)
        self._build_sequence(right)

    def _build_teach(self, parent):
        box = ctk.CTkFrame(parent)
        box.pack(fill="x", padx=4, pady=6)
        ctk.CTkLabel(box, text="Teach Pendant (Manual)",
                     font=("Arial", 14, "bold"), text_color=ACCENT).pack(anchor="w", padx=8)
        specs = [
            ("j1", "J1 - base", K.J1_MIN, K.J1_MAX, "J1"),
            ("j2", "J2 - shoulder", K.J2_MIN, K.J2_MAX, "J2"),
            ("j3", "J3 - elbow", K.J3_MIN, K.J3_MAX, "J3"),
            ("j4", "J4 - wrist", K.J4_MIN, K.J4_MAX, "J4"),
            ("track", "L - track", K.TRACK_MIN, K.TRACK_MAX, "L"),
            ("grip", "G - gripper", K.GRIP_OPEN, K.GRIP_CLOSED, "G"),
        ]
        self.sliders = {}
        self.slider_lbls = {}
        for key, label, lo, hi, cmd in specs:
            row = ctk.CTkFrame(box, fg_color="transparent")
            row.pack(fill="x", padx=8, pady=2)
            ctk.CTkLabel(row, text=label, width=90, anchor="w").pack(side="left")
            val = ctk.CTkLabel(row, text="", width=64)
            val.pack(side="right")
            sl = ctk.CTkSlider(box, from_=lo, to=hi,
                               command=lambda v, k=key, c=cmd: self._on_jog(k, c, v))
            sl.set(self.state[key])
            sl.pack(fill="x", padx=8, pady=(0, 6))
            self.sliders[key] = sl
            self.slider_lbls[key] = val
        self._update_slider_labels()

    def _build_cartesian(self, parent):
        box = ctk.CTkFrame(parent)
        box.pack(fill="x", padx=4, pady=6)
        ctk.CTkLabel(box, text="Cartesian Control (IK)",
                     font=("Arial", 14, "bold"), text_color=ACCENT).pack(anchor="w", padx=8)
        self.xyz_vars = {}
        row = ctk.CTkFrame(box, fg_color="transparent")
        row.pack(fill="x", padx=8, pady=4)
        for i, axis in enumerate(("X", "Y", "Z")):
            ctk.CTkLabel(row, text=f"Target {axis}").grid(row=0, column=i, padx=4)
            var = tk.StringVar(value={"X": "0", "Y": "30", "Z": "10"}[axis])
            var.trace_add("write", lambda *_: self._validate_xyz())
            ctk.CTkEntry(row, textvariable=var, width=64).grid(row=1, column=i, padx=4)
            self.xyz_vars[axis] = var
        self.xyz_status = ctk.CTkLabel(box, text="", font=("Arial", 12, "bold"))
        self.xyz_status.pack(anchor="w", padx=10)
        btns = ctk.CTkFrame(box, fg_color="transparent")
        btns.pack(fill="x", padx=8, pady=4)
        ctk.CTkButton(btns, text="\u25b6 GO TO XYZ", fg_color=GREEN,
                      command=self._go_xyz).pack(side="left", padx=3)
        ctk.CTkButton(btns, text="\u21a9 Use Suggested Point",
                      command=self._use_suggested).pack(side="left", padx=3)
        ctk.CTkButton(btns, text="\U0001f4cd Capture \u2192 XYZ",
                      command=self._capture_xyz).pack(side="left", padx=3)
        ctk.CTkButton(btns, text="\u2795 Capture \u2192 Add Step",
                      command=self._capture_step).pack(side="left", padx=3)

    def _build_gripper(self, parent):
        box = ctk.CTkFrame(parent)
        box.pack(fill="x", padx=4, pady=6)
        ctk.CTkLabel(box, text="Gripper - Open & Close",
                     font=("Arial", 14, "bold"), text_color=ACCENT).pack(anchor="w", padx=8)
        row = ctk.CTkFrame(box, fg_color="transparent")
        row.pack(fill="x", padx=8, pady=4)
        ctk.CTkButton(row, text=f"Open (G{int(K.GRIP_OPEN)})",
                      command=lambda: self._set_grip(K.GRIP_OPEN)).pack(side="left", padx=4)
        ctk.CTkButton(row, text=f"Close (G{int(K.GRIP_CLOSED)})",
                      command=lambda: self._set_grip(K.GRIP_CLOSED)).pack(side="left", padx=4)

    def _build_pickplace(self, parent):
        box = ctk.CTkFrame(parent)
        box.pack(fill="x", padx=4, pady=6)
        ctk.CTkLabel(box, text="Pick & Place - Firmware Automation",
                     font=("Arial", 14, "bold"), text_color=ACCENT).pack(anchor="w", padx=8)
        row = ctk.CTkFrame(box, fg_color="transparent")
        row.pack(fill="x", padx=8, pady=4)
        ctk.CTkButton(row, text="\u25b6 GO (if IR)",
                      command=lambda: self._send("GO")).pack(side="left", padx=3)
        ctk.CTkButton(row, text="\u27f3 CYCLE (force)",
                      command=lambda: self._send("CYCLE")).pack(side="left", padx=3)
        row2 = ctk.CTkFrame(box, fg_color="transparent")
        row2.pack(fill="x", padx=8, pady=4)
        ctk.CTkButton(row2, text="AUTO RUN (firmware)",
                      command=lambda: self._send("RUN")).pack(side="left", padx=3)
        ctk.CTkButton(row2, text="STOP AUTO", fg_color=RED,
                      command=lambda: self._send("STOP")).pack(side="left", padx=3)
        self.ir_switch = ctk.CTkSwitch(box, text="\U0001f534 IR Sensor Auto-Trigger (GUI sequence)",
                                       command=self._toggle_ir)
        self.ir_switch.pack(anchor="w", padx=10, pady=4)

    def _build_sequence(self, parent):
        ctk.CTkLabel(parent, text="Sequence Builder",
                     font=("Arial", 14, "bold"), text_color=ACCENT).pack(anchor="w", padx=8, pady=4)
        form = ctk.CTkFrame(parent, fg_color="transparent")
        form.pack(fill="x", padx=8)
        self.seq_vars = {a: tk.StringVar(value=v) for a, v in
                         (("X", "0"), ("Y", "30"), ("Z", "10"), ("G", ""), ("W", "1"))}
        for i, a in enumerate(("X", "Y", "Z")):
            ctk.CTkLabel(form, text=a).grid(row=0, column=2 * i, padx=2)
            ctk.CTkEntry(form, textvariable=self.seq_vars[a], width=54).grid(
                row=0, column=2 * i + 1, padx=2)
        self.grip_var = tk.StringVar(value="Keep")
        ctk.CTkLabel(form, text="Grip").grid(row=1, column=0)
        ctk.CTkOptionMenu(form, variable=self.grip_var,
                          values=["Open", "Close", "Keep", "Custom"],
                          width=80).grid(row=1, column=1)
        ctk.CTkLabel(form, text="G\u00b0").grid(row=1, column=2)
        ctk.CTkEntry(form, textvariable=self.seq_vars["G"], width=54).grid(row=1, column=3)
        ctk.CTkLabel(form, text="Wait(s)").grid(row=1, column=4)
        ctk.CTkEntry(form, textvariable=self.seq_vars["W"], width=54).grid(row=1, column=5)
        btns = ctk.CTkFrame(parent, fg_color="transparent")
        btns.pack(fill="x", padx=8, pady=4)
        ctk.CTkButton(btns, text="\u2795 Add", fg_color=GREEN,
                      command=self._seq_add).pack(side="left", padx=3)
        ctk.CTkButton(btns, text="\u2796 Remove", fg_color=ACCENT,
                      command=self._seq_remove).pack(side="left", padx=3)
        ctk.CTkButton(btns, text="\U0001f5d1 Clear",
                      command=self._seq_clear).pack(side="left", padx=3)
        self.seq_list = tk.Listbox(parent, bg="#202020", fg="white",
                                   font=("Consolas", 10), height=10,
                                   activestyle="none")
        self.seq_list.pack(fill="both", expand=True, padx=8, pady=4)
        io = ctk.CTkFrame(parent, fg_color="transparent")
        io.pack(fill="x", padx=8, pady=4)
        ctk.CTkButton(io, text="\U0001f4be Save", fg_color=ACCENT,
                      command=self._seq_save).pack(side="left", padx=3)
        ctk.CTkButton(io, text="\U0001f4c2 Load", fg_color=ACCENT,
                      command=self._seq_load).pack(side="left", padx=3)
        self.run_btn = ctk.CTkButton(parent, text="\u25b6\u25b6 RUN SEQUENCE",
                                     fg_color=GREEN, font=("Arial", 13, "bold"),
                                     command=self._seq_run)
        self.run_btn.pack(fill="x", padx=8, pady=6)

    def _build_console(self):
        box = ctk.CTkFrame(self)
        box.pack(fill="x", padx=8, pady=(4, 8))
        head = ctk.CTkFrame(box, fg_color="transparent")
        head.pack(fill="x")
        ctk.CTkLabel(head, text="Serial Console & Path Log",
                     font=("Arial", 13, "bold"), text_color=ACCENT).pack(side="left", padx=8)
        self.autoscroll = tk.BooleanVar(value=True)
        ctk.CTkCheckBox(head, text="Auto-scroll", variable=self.autoscroll).pack(side="right", padx=4)
        ctk.CTkButton(head, text="\U0001f4be Save Log", width=90, fg_color=ACCENT,
                      command=self._save_log).pack(side="right", padx=4)
        ctk.CTkButton(head, text="\U0001f5d1 Clear Log", width=90,
                      command=lambda: self.console.delete("1.0", "end")).pack(side="right", padx=4)
        self.console = ctk.CTkTextbox(box, height=130, font=("Consolas", 11))
        self.console.pack(fill="x", padx=8, pady=4)
        self._log("SYS", "Rhino Control Dashboard ready - connect a port to begin.")

    # ---------------- helpers ----------------
    def _log(self, direction, text):
        tag = {"TX": ">>", "RX": "<<", "SYS": "--"}.get(direction, "  ")
        self.console.insert("end", f"{tag} {text}\n")
        if self.autoscroll.get():
            self.console.see("end")

    def _save_log(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt")
        if path:
            with open(path, "w") as fh:
                fh.write(self.console.get("1.0", "end"))

    def _scan_ports(self):
        ports = self.link.ports()
        return ports if ports else ["(no ports found)"]

    def _refresh_ports(self):
        vals = self._scan_ports()
        self.port_menu.configure(values=vals)
        self.port_var.set(vals[0])

    def _toggle_connection(self):
        if self.connected:
            self.link.disconnect()
        else:
            port = self.port_var.get()
            if "no ports" in port:
                self._log("SYS", "No serial port available.")
                return
            try:
                self.link.connect(port)
            except Exception as exc:
                self._log("SYS", f"Connect failed: {exc}")

    def _on_state(self, connected):
        def apply():
            self.connected = connected
            self.status_lbl.configure(
                text="\u25cf CONNECTED" if connected else "\u25cf DISCONNECTED",
                text_color=GREEN if connected else "grey")
            self.conn_btn.configure(text="Disconnect" if connected else "Connect",
                                    fg_color=RED if connected else GREEN)
        self.after(0, apply)

    def _on_line(self, direction, text):
        self.after(0, lambda: self._log(direction, text))
        if direction == "RX" and self.ir_trigger and "IR_DETECT" in text:
            self.after(0, self._seq_run)

    def _send(self, text):
        self.link.send(text)

    def _toggle_mode(self):
        self.mode = "LIVE" if self.mode == "SIM" else "SIM"
        self.mode_btn.configure(text=f"MODE: {self.mode}",
                                fg_color=RED if self.mode == "LIVE" else "#8a6d1d")
        self._send(self.mode)

    def _estop(self):
        # documented behaviour: E-STOP drops the robot to SIM and halts motion
        self.mode = "SIM"
        self.mode_btn.configure(text="MODE: SIM", fg_color="#8a6d1d")
        self._send("ESTOP")
        self._log("SYS", "E-STOP pressed - robot halted, mode forced to SIM.")

    def _toggle_ir(self):
        self.ir_trigger = bool(self.ir_switch.get())
        self._log("SYS", f"IR auto-trigger (GUI sequence) "
                         f"{'ON' if self.ir_trigger else 'OFF'}")

    # ---------------- teach pendant ----------------
    def _on_jog(self, key, cmd, value):
        self.state[key] = float(value)
        self._update_slider_labels()
        if cmd == "L":
            self._send(f"L {value:.1f}")
        elif cmd == "G":
            self._send(f"G {value:.0f}")
        else:
            self._send(f"{cmd} {value:.1f}")
        self._refresh_twin()
        self._validate_xyz()

    def _update_slider_labels(self):
        for key, lbl in self.slider_lbls.items():
            unit = "cm" if key == "track" else "\u00b0"
            lbl.configure(text=f"{self.state[key]:.0f}{unit}")

    def _set_grip(self, angle):
        self.state["grip"] = angle
        self.sliders["grip"].set(angle)
        self._update_slider_labels()
        self._send(f"G {angle:.0f}")

    def _refresh_twin(self):
        s = self.state
        self.twin.update(s["j1"], s["j2"], s["j3"], s["j4"], s["track"])
        tip = K.fk_from_joints(s["j1"], s["j2"], s["j3"], s["j4"], s["track"])["tip"]
        for name, v in zip(("X", "Y", "Z"), tip):
            self.readout_vals[name].configure(text=f"{v:.1f}")
        for name, key in (("J1", "j1"), ("J2", "j2"), ("J3", "j3"), ("J4", "j4")):
            self.readout_vals[name].configure(text=f"{s[key]:.1f}")

    def _toggle_cloud(self):
        self.twin.show_cloud = bool(self.cloud_var.get())
        self._refresh_twin()

    def _open_plotly(self):
        threading.Thread(target=self.twin.open_full_workspace, daemon=True).start()

    # ---------------- cartesian ----------------
    def _xyz(self):
        try:
            return (float(self.xyz_vars["X"].get()),
                    float(self.xyz_vars["Y"].get()),
                    float(self.xyz_vars["Z"].get()))
        except ValueError:
            return None

    def _validate_xyz(self):
        tgt = self._xyz()
        if tgt is None:
            self.xyz_status.configure(text="invalid numbers", text_color=RED)
            return
        st, msg, sug = K.validate_target(*tgt, track=self.state["track"])
        self._suggested = sug
        colors = {"ok": GREEN, "warn": ACCENT, "bad": RED}
        marks = {"ok": "\u2705", "warn": "\u26a0", "bad": "\u274c"}
        self.xyz_status.configure(text=f"{marks[st]} {msg}", text_color=colors[st])

    def _go_xyz(self):
        tgt = self._xyz()
        if tgt is None:
            return
        st, msg, _ = K.validate_target(*tgt, track=self.state["track"])
        if st == "bad":
            self.bell()
            self._log("SYS", f"Refused unreachable target {tgt}")
            return
        ok, res = K.ik_to_joints(*tgt, track=self.state["track"])
        if ok and abs(res["track"] - self.state["track"]) > 0.5:
            self._send(f"L {res['track']:.1f}")
            self.state["track"] = res["track"]
            self.sliders["track"].set(res["track"])
        self._send(f"XYZ {tgt[0]:.1f} {tgt[1]:.1f} {tgt[2]:.1f}")

    def _use_suggested(self):
        sug = getattr(self, "_suggested", None)
        if sug:
            for axis, v in zip(("X", "Y", "Z"), sug):
                self.xyz_vars[axis].set(f"{v:.1f}")

    def _capture_xyz(self):
        s = self.state
        tip = K.fk_from_joints(s["j1"], s["j2"], s["j3"], s["j4"], s["track"])["tip"]
        for axis, v in zip(("X", "Y", "Z"), tip):
            self.xyz_vars[axis].set(f"{v:.1f}")

    def _capture_step(self):
        self._capture_xyz()
        self.seq_vars["G"].set(f"{self.state['grip']:.0f}")
        self._seq_add()

    # ---------------- sequence builder ----------------
    def _fmt_step(self, st):
        return (f"({st['x']:6.1f}, {st['y']:6.1f}, {st['z']:5.1f})  "
                f"grip={st['grip']!s:>5}  wait={st['wait']}s")

    def _seq_add(self):
        try:
            st = {
                "x": float(self.seq_vars["X"].get()),
                "y": float(self.seq_vars["Y"].get()),
                "z": float(self.seq_vars["Z"].get()),
                "wait": float(self.seq_vars["W"].get() or 0),
            }
        except ValueError:
            self._log("SYS", "Sequence step has invalid numbers.")
            return
        mode = self.grip_var.get()
        custom = self.seq_vars["G"].get().strip()
        if custom:
            st["grip"] = float(custom)
        else:
            st["grip"] = {"Open": K.GRIP_OPEN, "Close": K.GRIP_CLOSED,
                          "Keep": "keep"}[mode]
        self.steps.append(st)
        self.seq_list.insert("end", self._fmt_step(st))

    def _seq_remove(self):
        sel = self.seq_list.curselection()
        if sel:
            i = sel[0]
            self.seq_list.delete(i)
            del self.steps[i]

    def _seq_clear(self):
        if messagebox.askyesno("Clear", "Remove all steps?"):
            self.steps.clear()
            self.seq_list.delete(0, "end")

    def _seq_save(self):
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON", "*.json")])
        if path:
            with open(path, "w") as fh:
                json.dump(self.steps, fh, indent=2)
            self._log("SYS", f"Sequence saved to {path}")

    def _seq_load(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            with open(path) as fh:
                self.steps = json.load(fh)
            self.seq_list.delete(0, "end")
            for st in self.steps:
                self.seq_list.insert("end", self._fmt_step(st))
            self._log("SYS", f"Loaded {len(self.steps)} steps from {path}")

    def _seq_run(self):
        if self.seq_running:
            self.seq_running = False
            self.run_btn.configure(text="\u25b6\u25b6 RUN SEQUENCE")
            return
        if not self.steps:
            self._log("SYS", "Sequence is empty.")
            return
        self.seq_running = True
        self.run_btn.configure(text="\u23f9 STOP SEQUENCE")
        self._seq_thread = threading.Thread(target=self._seq_worker, daemon=True)
        self._seq_thread.start()

    def _seq_worker(self):
        for st in self.steps:
            if not self.seq_running:
                break
            self._send(f"XYZ {st['x']:.1f} {st['y']:.1f} {st['z']:.1f}")
            if st["grip"] != "keep":
                self._send(f"G {float(st['grip']):.0f}")
            time.sleep(max(0.0, float(st["wait"])))
        self.seq_running = False
        self.after(0, lambda: self.run_btn.configure(text="\u25b6\u25b6 RUN SEQUENCE"))

    # ---------------- shutdown ----------------
    def _on_close(self):
        self.seq_running = False
        self.link.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = RhinoDashboard()
    app.mainloop()
