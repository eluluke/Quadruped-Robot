#!/usr/bin/env python3
"""

Sends PDO2/PDO3 position commands to three STM32G431 B-G431B-ESC1 boards
running the UCB Recoil FOC firmware, over CANable SocketCAN.

Run:
    sudo bash scripts/setup_can.sh   # once, before starting
    python3 main.py

need:  python3, tkinter (usually built-in), python-can
to install:   pip3 install python-can
"""

# Imports
import math
import struct
import time
import tkinter as tk
from tkinter import font as tkfont
import threading
import logging
import platform

import config
from ik_solver import solve, IKResult
from can_interface import RecoilCAN, JointTelemetry

# Not sure what this does research later
logging.basicConfig(level=logging.DEBUG, format="%(levelname)s %(name)s: %(message)s")

# Conversions
DEG = 180 / math.pi
RAD = math.pi / 180

# clamps values between two values so that they don't go beyond the limits.
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ── Colour palette ────────────────────────────────────────────────────────────
if config.DARK_MODE:
    C = dict(
        bg="#1a1a18", grid="#2a2a28", axis="#3a3a38",
        reach="#1a2a3a", reach_bd="#2a4a6a",
        bone_hip="#378ADD", bone_knee="#1D9E75", bone_ank="#D85A30",
        joint="#d4d2c8", joint_bg="#1a1a18",
        foot="#E24B4A", target="#E24B4A", target_r="#4a1a1a",
        text="#9a9890", text2="#c2c0b6",
        panel_bg="#111110", panel_bd="#2a2a28",
        section="#555550", value="#e8e6dc",
        bar_bg="#2a2a28",
        ok="#1D9E75", warn="#E24B4A",
        can_ok="#1D9E75", can_err="#E24B4A", can_sim="#D85A30",
        telem="#378ADD",
    )
else:
    C = dict(
        bg="#f5f4ee", grid="#e8e6de", axis="#ccc9be",
        reach="#e8f2fc", reach_bd="#b5d4f4",
        bone_hip="#185FA5", bone_knee="#0F6E56", bone_ank="#993C1D",
        joint="#2c2c2a", joint_bg="#f5f4ee",
        foot="#A32D2D", target="#A32D2D", target_r="#fce8e8",
        text="#888780", text2="#3d3d3a",
        panel_bg="#ffffff", panel_bd="#e0ddd5",
        section="#aaa89e", value="#1a1a18",
        bar_bg="#e0ddd5",
        ok="#0F6E56", warn="#A32D2D",
        can_ok="#0F6E56", can_err="#A32D2D", can_sim="#993C1D",
        telem="#185FA5",
    )


# ═══════════════════════════════════════════════════════════════════════════════
class App(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title(config.WINDOW_TITLE)
        self.configure(bg=C["panel_bg"])
        self.resizable(True, True)
        self.minsize(960, 600)

        # Link lengths (live-editable)
        self.L1 = tk.DoubleVar(value=config.L1)
        self.L2 = tk.DoubleVar(value=config.L2)
        self.L3 = tk.DoubleVar(value=config.L3)

        # IK state
        self.solution_var = tk.IntVar(value=config.DEFAULT_SOLUTION if hasattr(config, "DEFAULT_SOLUTION") else 0)
        self.shoulder_angle = tk.DoubleVar(value=0.0)  # Shoulder tilt angle (radians)
        self.target_x = config.L1 * 0.5
        self.target_y = -(config.L2) * 0.9
        self.ik_result: IKResult | None = None
        self.reachable = False

        # CAN
        self.can = RecoilCAN(
            interface=config.CAN_INTERFACE,
            channel=config.CAN_CHANNEL,
            bitrate=config.CAN_BITRATE,
            timeout=config.CAN_TIMEOUT,
        )
        self.can_connected = False
        self._last_send_t  = 0.0
        self._send_interval = 1.0 / config.SEND_RATE_HZ
        self._motors_armed  = False   # True once MODE_POSITION has been sent
        self._simulation_mode = False  # Simulation-only mode (logs without sending)
        self._last_limit_violation = False  # Track if last command violated limits

        # Heartbeat thread
        self._hb_stop  = threading.Event()
        self._hb_thread: threading.Thread | None = None

        # Canvas helpers (see zoom/pan state below)
        self._dragging = False
        self._panning   = False
        self._pan_start = (0, 0)
        self.scale      = config.PIXELS_PER_MM
        self._zoom_min  = 0.2
        self._zoom_max  = 12.0
        self.origin_x   = 0
        self.origin_y   = 0

        # Device ID overrides (editable in panel)
        self.did_hip   = tk.IntVar(value=config.DEVICE_ID_HIP)
        self.did_knee  = tk.IntVar(value=config.DEVICE_ID_KNEE)
        self.did_ankle = tk.IntVar(value=config.DEVICE_ID_ANKLE)

        # Angle offsets
        self.off_hip   = tk.DoubleVar(value=config.HIP_OFFSET_RAD)
        self.off_knee  = tk.DoubleVar(value=config.KNEE_OFFSET_RAD)
        self.off_ankle = tk.DoubleVar(value=config.ANKLE_OFFSET_RAD)

        # Joint limits (degrees — easier for users to type)
        self.lim_shoulder_lo = tk.DoubleVar(value=math.degrees(config.SHOULDER_LIMIT_RAD[0]))
        self.lim_shoulder_hi = tk.DoubleVar(value=math.degrees(config.SHOULDER_LIMIT_RAD[1]))
        self.lim_thigh_lo   = tk.DoubleVar(value=math.degrees(config.HIP_LIMIT_RAD[0]))
        self.lim_thigh_hi   = tk.DoubleVar(value=math.degrees(config.HIP_LIMIT_RAD[1]))
        self.lim_knee_lo  = tk.DoubleVar(value=math.degrees(config.KNEE_LIMIT_RAD[0]))
        self.lim_knee_hi  = tk.DoubleVar(value=math.degrees(config.KNEE_LIMIT_RAD[1]))
        self.lim_shin_lo  = tk.DoubleVar(value=math.degrees(config.ANKLE_LIMIT_RAD[0]))
        self.lim_shin_hi  = tk.DoubleVar(value=math.degrees(config.ANKLE_LIMIT_RAD[1]))
        self._show_limits = tk.BooleanVar(value=True)

        self._build_ui()
        self.after(50, self._solve_and_draw)
        self.after(500, self._poll_telemetry)

    # ── UI layout ─────────────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Font definitions (customize font size here) ──────────────────────
        mono   = ("Courier New", 9)      # Regular monospace font for labels/entries
        mono_s = ("Courier New", 8)      # Small monospace font
        mono_b = ("Courier New", 9, "bold")  # Bold monospace font for buttons

        # ── Toolbar ────────────────────────────────────────────────────────────
        tb = tk.Frame(self, bg=C["panel_bg"])  # Top bar frame
        tb.pack(side=tk.TOP, fill=tk.X)
        tk.Label(tb, text="QUAD IK  /  RECOIL CAN",  # Change window title text here
                 bg=C["panel_bg"], fg=C["text2"],
                 font=("Courier New", 11, "bold")).pack(side=tk.LEFT, padx=14, pady=8)  # padx/pady = spacing

        self.btn_connect = tk.Button(
            tb, text="● CONNECT CAN",  # Button label text
            bg=C["panel_bg"], fg=C["can_sim"],  # bg = button background, fg = text color
            font=mono, relief=tk.FLAT, bd=0,  # relief=FLAT for flat style, bd=border depth
            activebackground=C["panel_bg"], cursor="hand2",  # cursor="hand2" = pointy hand on hover
            command=self._toggle_can)  # Function called when clicked
        self.btn_connect.pack(side=tk.RIGHT, padx=14, pady=6)  # padx/pady = margin spacing

        self.lbl_status = tk.Label(
            tb, text="NOT CONNECTED",  # Initial status text
            bg=C["panel_bg"], fg=C["section"], font=mono_s)  # mono_s = small font
        self.lbl_status.pack(side=tk.RIGHT, padx=4)

        tk.Frame(self, bg=C["panel_bd"], height=1).pack(fill=tk.X)  # Divider line under toolbar

        # main row
        main = tk.Frame(self, bg=C["panel_bg"])  # Container for canvas + right panel
        main.pack(fill=tk.BOTH, expand=True)

        # ── Canvas container (holds both side + top views) ────────────────────
        canvas_container = tk.Frame(main, bg=C["bg"])
        canvas_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # ── Side view canvas (sagittal plane) ──────────────────────────────────
        self.canvas_side = tk.Canvas(canvas_container, bg=C["bg"], highlightthickness=0, cursor="crosshair")
        self.canvas_side.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas_side.bind("<Configure>",        self._on_resize_side)
        self.canvas_side.bind("<ButtonPress-1>",    self._on_mouse_down)
        self.canvas_side.bind("<B1-Motion>",         self._on_mouse_drag)
        self.canvas_side.bind("<ButtonRelease-1>",   self._on_mouse_up)
        # Zoom: scroll wheel
        self.canvas_side.bind("<MouseWheel>",        self._on_zoom)
        self.canvas_side.bind("<Button-4>",          self._on_zoom)
        self.canvas_side.bind("<Button-5>",          self._on_zoom)
        # Pan: right-button drag (or middle-button drag)
        self.canvas_side.bind("<ButtonPress-3>",     self._on_pan_start)
        self.canvas_side.bind("<B3-Motion>",         self._on_pan_drag)
        self.canvas_side.bind("<ButtonRelease-3>",   self._on_pan_end)
        self.canvas_side.bind("<ButtonPress-2>",     self._on_pan_start)
        self.canvas_side.bind("<B2-Motion>",         self._on_pan_drag)
        self.canvas_side.bind("<ButtonRelease-2>",   self._on_pan_end)
        # Zoom-to-fit: double-click
        self.canvas_side.bind("<Double-Button-1>",   self._zoom_to_fit)
        
        # Keep old name for compatibility
        self.canvas = self.canvas_side

        # ── Top view canvas (top-down, showing shoulder tilt) ─────────────────
        self.canvas_top = tk.Canvas(canvas_container, bg=C["bg"], highlightthickness=0, cursor="crosshair")
        self.canvas_top.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas_top.bind("<Configure>",        self._on_resize_top)
        self.canvas_top.bind("<ButtonPress-1>",    self._on_mouse_down_top)
        self.canvas_top.bind("<B1-Motion>",         self._on_mouse_drag_top)
        self.canvas_top.bind("<ButtonRelease-1>",   self._on_mouse_up)
        # Zoom: scroll wheel (shared with side view)
        self.canvas_top.bind("<MouseWheel>",        self._on_zoom_top)
        self.canvas_top.bind("<Button-4>",          self._on_zoom_top)
        self.canvas_top.bind("<Button-5>",          self._on_zoom_top)
        # Pan: right-button drag
        self.canvas_top.bind("<ButtonPress-3>",     self._on_pan_start_top)
        self.canvas_top.bind("<B3-Motion>",         self._on_pan_drag_top)
        self.canvas_top.bind("<ButtonRelease-3>",   self._on_pan_end_top)
        self.canvas_top.bind("<ButtonPress-2>",     self._on_pan_start_top)
        self.canvas_top.bind("<B2-Motion>",         self._on_pan_drag_top)
        self.canvas_top.bind("<ButtonRelease-2>",   self._on_pan_end_top)
        
        # Top-down view pan/zoom state
        self._pan_start_top = (0, 0)
        self._panning_top = False
        self.scale_top = config.PIXELS_PER_MM
        self._zoom_min_top = 0.2
        self._zoom_max_top = 12.0
        self.origin_x_top = 0
        self.origin_y_top = 0

        # ── Scrollable right panel ─────────────────────────────────────────
        panel_outer = tk.Frame(main, bg=C["panel_bg"], width=262)  # Outer container; width=262 sets panel width (pixels)
        panel_outer.pack(side=tk.RIGHT, fill=tk.Y)
        panel_outer.pack_propagate(False)  # Keeps fixed width even if contents are smaller

        # Scrollbar styling
        pscroll = tk.Scrollbar(panel_outer, 
                               bg="#2DB88A",                    # Scrollbar thumb color (forest green)
                               activebackground="#3DD4A5",      # Color when hovering (lighter green)
                               troughcolor=C["panel_bg"],       # Track background color (matches panel)
                               orient=tk.VERTICAL, width=12)    # width=12 sets scrollbar thickness (pixels)
        pscroll.pack(side=tk.RIGHT, fill=tk.Y)

        # Canvas acts as the scrollable viewport
        self._panel_canvas = tk.Canvas(
            panel_outer,
            bg=C["panel_bg"],  # Background color inside scrollable area
            highlightthickness=0,  # Border around canvas
            yscrollcommand=pscroll.set,  # Connect scrollbar to canvas
        )
        self._panel_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        pscroll.config(command=self._panel_canvas.yview)  # Connect canvas to scrollbar

        # Inner frame that actually holds all widgets
        panel = tk.Frame(self._panel_canvas, bg=C["panel_bg"])
        self._panel_window = self._panel_canvas.create_window(
            (0, 0), window=panel, anchor="nw"  # "nw" = northwest (top-left)
        )

        def _on_panel_configure(event):
            # Update scrollable region whenever panel content changes
            self._panel_canvas.configure(scrollregion=self._panel_canvas.bbox("all"))
        panel.bind("<Configure>", _on_panel_configure)

        def _on_panel_canvas_resize(event):
            # Make panel content stretch to fill canvas width
            self._panel_canvas.itemconfig(self._panel_window, width=event.width)
        self._panel_canvas.bind("<Configure>", _on_panel_canvas_resize)

        # Mouse-wheel scroll on the panel
        def _panel_scroll(event):
            # Scroll up (negative) or down (positive)
            if event.num == 4 or event.delta > 0:
                self._panel_canvas.yview_scroll(-1, "units")
            else:
                self._panel_canvas.yview_scroll(1, "units")
        for widget in (self._panel_canvas, panel):
            widget.bind("<MouseWheel>", _panel_scroll)  # Windows/Mac scroll wheel
            widget.bind("<Button-4>",   _panel_scroll)  # Linux scroll up
            widget.bind("<Button-5>",   _panel_scroll)  # Linux scroll down

        self._build_panel(panel, mono, mono_s, mono_b)

    def _build_panel(self, P, mono, mono_s, mono_b):
        # ── Helper function: Section separator ─────────────────────────────────
        def sep(label):
            # Creates a horizontal line with a section label
            tk.Frame(P, bg=C["panel_bd"], height=1).pack(fill=tk.X, padx=8, pady=(10,3))  # Line
            tk.Label(P, text=label, bg=C["panel_bg"], fg=C["section"],
                     font=("Courier New", 7), anchor="w").pack(fill=tk.X, padx=12)  # Label text

        # ── Helper function: Entry row ─────────────────────────────────────────
        def entry_row(parent, label, var, w=7):
            # Creates a label + text entry field pair
            f = tk.Frame(parent, bg=C["panel_bg"])
            f.pack(fill=tk.X, padx=12, pady=1)  # padx/pady = spacing
            tk.Label(f, text=label, bg=C["panel_bg"], fg=C["text"],
                     font=mono, width=9, anchor="w").pack(side=tk.LEFT)
            e = tk.Entry(f, textvariable=var, font=mono,
                         bg=C["bar_bg"], fg=C["value"], relief=tk.FLAT,
                         insertbackground=C["value"], width=w, bd=2)  # width=w sets field width (pixels)
            e.pack(side=tk.LEFT)
            return e

        # ──────────────────────────────────────────────────────────────────────
        # ── TARGET POSITION INPUT (mm) ─────────────────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("TARGET  (mm)")  # Section title
        cf = tk.Frame(P, bg=C["panel_bg"])
        cf.pack(fill=tk.X, padx=12, pady=2)
        for lbl, attr in [("X", "entry_x"), ("Y", "entry_y")]:  # Create X and Y input fields
            tk.Label(cf, text=lbl, bg=C["panel_bg"], fg=C["text"],
                     font=mono, width=2).pack(side=tk.LEFT)
            e = tk.Entry(cf, font=mono, bg=C["bar_bg"], fg=C["value"],
                         relief=tk.FLAT, insertbackground=C["value"],
                         width=7, bd=2)  # width=7 sets field width
            e.pack(side=tk.LEFT, padx=(0,6))
            setattr(self, attr, e)
            e.bind("<Return>", self._apply_coords)  # Apply coords when user presses Enter

        tk.Button(P, text="APPLY  ↵", bg=C["bar_bg"], fg=C["text2"],
                  font=mono_s, relief=tk.FLAT, bd=0, cursor="hand2",
                  command=self._apply_coords).pack(fill=tk.X, padx=12, pady=(3,0))

        # ──────────────────────────────────────────────────────────────────────
        # ── SHOULDER TILT (lateral angle) ──────────────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("SHOULDER TILT  (rad / °)")  # Section title
        
        # Shoulder slider
        sf = tk.Frame(P, bg=C["panel_bg"])
        sf.pack(fill=tk.X, padx=12, pady=2)
        tk.Label(sf, text="TILT", bg=C["panel_bg"], fg=C["text"],
                 font=mono, width=9, anchor="w").pack(side=tk.LEFT)
        self.shoulder_slider = tk.Scale(
            sf, from_=-90, to=90, orient=tk.HORIZONTAL,
            variable=tk.IntVar(),  # We'll handle sync manually
            bg=C["bar_bg"], fg=C["value"], troughcolor=C["bar_bg"],
            highlightthickness=0, relief=tk.FLAT, bd=0,
            command=self._on_shoulder_slider)
        self.shoulder_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
        
        # Shoulder angle display
        self.shoulder_angle_label = tk.Label(sf, text="0.000  0°", bg=C["panel_bg"], 
                                             fg=C["value"], font=mono, width=10, anchor="e")
        self.shoulder_angle_label.pack(side=tk.LEFT)

        # ──────────────────────────────────────────────────────────────────────
        # ── LINK LENGTHS (editable thigh/shin length) ──────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("LINK LENGTHS  (mm)")  # Section title
        entry_row(P, "L1 thigh", self.L1)  # Thigh length
        entry_row(P, "L2 shin ", self.L2)  # Shin length
        entry_row(P, "L3 foot ", self.L3)  # Foot length
        tk.Button(P, text="REBUILD  ↵", bg=C["bar_bg"], fg=C["text2"],
                  font=mono_s, relief=tk.FLAT, bd=0, cursor="hand2",
                  command=self._solve_and_draw).pack(fill=tk.X, padx=12, pady=(3,0))

        # ──────────────────────────────────────────────────────────────────────
        # ── IK SOLUTION (choose elbow position) ────────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("IK SOLUTION")  # Section title
        sf = tk.Frame(P, bg=C["panel_bg"])
        sf.pack(fill=tk.X, padx=12, pady=2)
        for i, lbl in enumerate(("ELBOW DOWN", "ELBOW UP")):  # Two orientation options
            tk.Radiobutton(sf, text=lbl, variable=self.solution_var, value=i,
                           bg=C["panel_bg"], fg=C["text"], selectcolor=C["bar_bg"],
                           activebackground=C["panel_bg"],
                           font=mono_s, command=self._solve_and_draw).pack(side=tk.LEFT, padx=(0,8))

        # ──────────────────────────────────────────────────────────────────────
        # ── REACHABILITY STATUS ────────────────────────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        self.lbl_reach = tk.Label(P, text="REACHABLE",
                                  bg=C["ok"], fg="#ffffff",  # bg = green when reachable, changes to red (C["warn"]) when not
                                  font=("Courier New", 8, "bold"), pady=3)
        self.lbl_reach.pack(fill=tk.X, padx=12, pady=5)

        # ──────────────────────────────────────────────────────────────────────
        # ── JOINT ANGLES (computed by IK solver) ───────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("JOINT ANGLES  (rad / °)")  # Section title
        self._angle_vars  = []  # Store angle value labels
        self._bar_canvases = []  # Store progress bars for angle ranges
        for name, col in [("SHLD θ₁", C["bone_hip"]),     # Shoulder angle
                           ("THGH θ₂", C["bone_knee"]),   # Thigh angle
                           ("SHIN θ₃", C["bone_ank"])]:   # Shin angle
            f = tk.Frame(P, bg=C["panel_bg"])
            f.pack(fill=tk.X, padx=12, pady=2)
            tk.Label(f, text=name, bg=C["panel_bg"], fg=col,  # Label with joint color
                     font=mono_s, width=10, anchor="w").pack(side=tk.LEFT)
            var = tk.StringVar(value="—")
            tk.Label(f, textvariable=var, bg=C["panel_bg"], fg=C["value"],
                     font=("Courier New", 9, "bold"), width=15, anchor="e").pack(side=tk.RIGHT)  # Readout
            self._angle_vars.append(var)
            bc = tk.Canvas(P, height=4, bg=C["bar_bg"], highlightthickness=0)  # Progress bar
            bc.pack(fill=tk.X, padx=12, pady=(0,2))
            self._bar_canvases.append((bc, col))

        # ──────────────────────────────────────────────────────────────────────
        # ── MOTOR TELEMETRY (feedback from ESC hardware) ──────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("MOTOR TELEMETRY  (rx)")  # Section title
        self._telem_vars = []  # Store telemetry value labels
        for name, col in [("SHLD pos", C["bone_hip"]),      # Shoulder position feedback
                           ("THGH pos", C["bone_knee"]),    # Thigh position feedback
                           ("SHIN pos", C["bone_ank"])]:    # Shin position feedback
            f = tk.Frame(P, bg=C["panel_bg"])
            f.pack(fill=tk.X, padx=12, pady=1)
            tk.Label(f, text=name, bg=C["panel_bg"], fg=col,
                     font=mono_s, width=10, anchor="w").pack(side=tk.LEFT)
            var = tk.StringVar(value="—")
            tk.Label(f, textvariable=var, bg=C["panel_bg"], fg=C["telem"],
                     font=("Courier New", 8), width=12, anchor="e").pack(side=tk.RIGHT)  # Readout
            self._telem_vars.append(var)

        # ──────────────────────────────────────────────────────────────────────
        # ── DEVICE IDs (CANopen node addresses) ────────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("DEVICE IDs  (node address)")  # Section title
        for lbl, var in [("HIP  ", self.did_hip),       # Change these to match your motor IDs
                         ("KNEE ", self.did_knee),
                         ("ANKLE", self.did_ankle)]:
            entry_row(P, lbl, var, w=4)  # w=4 makes field narrower for short numbers

        # ──────────────────────────────────────────────────────────────────────
        # ── ZERO OFFSETS (calibration adjustment) ──────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("ZERO OFFSETS  (rad)")  # Section title - adjust these to calibrate joint angles
        for lbl, var in [("HIP  ", self.off_hip),       # Offset in radians
                         ("KNEE ", self.off_knee),
                         ("ANKLE", self.off_ankle)]:
            entry_row(P, lbl, var, w=7)

        # ──────────────────────────────────────────────────────────────────────
        # ── JOINT LIMITS (angle constraints per joint) ─────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("JOINT LIMITS  (degrees)")  # Section title

        # Toggle + reset row
        lf0 = tk.Frame(P, bg=C["panel_bg"])
        lf0.pack(fill=tk.X, padx=12, pady=(2,4))
        tk.Checkbutton(lf0, text="show on canvas",  # Toggle limit visualization
                       variable=self._show_limits,
                       bg=C["panel_bg"], fg=C["text"],
                       selectcolor=C["bar_bg"],
                       activebackground=C["panel_bg"],
                       font=mono_s,
                       command=self._draw).pack(side=tk.LEFT)
        tk.Button(lf0, text="reset",  # Reset limits to config.py defaults
                  bg=C["bar_bg"], fg=C["text2"],
                  font=mono_s, relief=tk.FLAT, bd=0, cursor="hand2",
                  command=self._reset_limits).pack(side=tk.RIGHT)

        # Per-joint lo/hi rows
        limit_rows = [
            ("SHLD", self.lim_shoulder_lo, self.lim_shoulder_hi, C["bone_hip"]),    # Min/max shoulder tilt (lateral)
            ("THGH", self.lim_thigh_lo,    self.lim_thigh_hi,     C["bone_knee"]),   # Min/max thigh angle
            ("SHIN", self.lim_shin_lo,    self.lim_shin_hi,     C["bone_ank"]),    # Min/max shin angle
        ]
        for jname, lo_var, hi_var, jcol in limit_rows:
            lf = tk.Frame(P, bg=C["panel_bg"])
            lf.pack(fill=tk.X, padx=12, pady=2)
            tk.Label(lf, text=jname, bg=C["panel_bg"], fg=jcol,
                     font=mono_s, width=4, anchor="w").pack(side=tk.LEFT)
            tk.Label(lf, text="lo", bg=C["panel_bg"], fg=C["text"],
                     font=mono_s).pack(side=tk.LEFT, padx=(2,1))
            lo_e = tk.Entry(lf, textvariable=lo_var, font=mono,
                            bg=C["bar_bg"], fg=C["value"], relief=tk.FLAT,
                            insertbackground=C["value"], width=6, bd=2)  # Low limit field
            lo_e.pack(side=tk.LEFT)
            lo_e.bind("<Return>", lambda e: self._solve_and_draw())
            tk.Label(lf, text="hi", bg=C["panel_bg"], fg=C["text"],
                     font=mono_s).pack(side=tk.LEFT, padx=(4,1))
            hi_e = tk.Entry(lf, textvariable=hi_var, font=mono,
                            bg=C["bar_bg"], fg=C["value"], relief=tk.FLAT,
                            insertbackground=C["value"], width=6, bd=2)  # High limit field
            hi_e.pack(side=tk.LEFT)
            hi_e.bind("<Return>", lambda e: self._solve_and_draw())

        # ──────────────────────────────────────────────────────────────────────
        # ── IK MATH DISPLAY (solver internals) ──────────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("IK MATH")  # Section title
        self.math_text = tk.Text(P, height=10, bg=C["bar_bg"], fg=C["text"],  # height=10 sets text box height (lines)
                                 font=("Courier New", 7), relief=tk.FLAT,  # 7 = very small font
                                 bd=4, state=tk.DISABLED, wrap=tk.NONE)  # Read-only, no line wrap
        self.math_text.pack(fill=tk.X, padx=12, pady=2)

        # ──────────────────────────────────────────────────────────────────────
        # ── CAN LOG (communication history) ────────────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        sep("CAN TX LOG")  # Section title
        self.can_log = tk.Text(P, height=6, bg=C["bar_bg"], fg=C["can_ok"],  # height=6 sets text box height
                               font=("Courier New", 7), relief=tk.FLAT,
                               bd=4, state=tk.DISABLED, wrap=tk.NONE)  # Read-only, auto-scrolls with each message
        self.can_log.pack(fill=tk.X, padx=12, pady=(2,4))

        # ──────────────────────────────────────────────────────────────────────
        # ── MOTOR CONTROL BUTTONS ──────────────────────────────────────────────
        # ──────────────────────────────────────────────────────────────────────
        bf = tk.Frame(P, bg=C["panel_bg"])
        bf.pack(fill=tk.X, padx=12, pady=(0,4))
        self.btn_arm = tk.Button(
            bf, text="ARM MOTORS",  # Arm motors for position control
            bg=C["bar_bg"], fg=C["text2"],
            font=mono_s, relief=tk.FLAT, bd=0, cursor="hand2",
            command=self._arm_motors)  # Send MODE_POSITION to all joints
        self.btn_arm.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,4))

        self.btn_send = tk.Button(
            bf, text="▶ SEND",  # Send joint commands manually
            bg=C["bone_hip"], fg="#ffffff",  # bg = shoulder color (eye-catching)
            font=mono_b, relief=tk.FLAT, bd=0, cursor="hand2",
            pady=5, command=self._manual_send)  # Send command once
        self.btn_send.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Simulation toggle button
        self.btn_simulate = tk.Button(
            P, text="▶ SIMULATE (log only, no hardware)",
            bg=C["can_sim"], fg="#ffffff",
            font=mono_s, relief=tk.FLAT, bd=0, cursor="hand2",
            command=self._toggle_simulation)
        self.btn_simulate.pack(fill=tk.X, padx=12, pady=(4,0))

        # Limit violation warning
        self.lbl_limit_warn = tk.Label(P, text="",
                                       bg=C["panel_bg"], fg=C["warn"],
                                       font=("Courier New", 7))
        self.lbl_limit_warn.pack(fill=tk.X, padx=12, pady=(2,0))

        tk.Label(P, text="drag target on canvas · or type coords",
                 bg=C["panel_bg"], fg=C["section"],
                 font=("Courier New", 7)).pack(pady=(0,8))  # Instructions/hint

    # ── Canvas drawing ─────────────────────────────────────────────────────────

    def _on_resize(self, event):
        # Keep this for backwards compatibility, but it's now _on_resize_side
        self._on_resize_side(event)

    def _on_resize_side(self, event):
        # On first resize set origin; afterwards keep existing pan/zoom
        if self.origin_x == 0:
            self.origin_x = int(event.width  * 0.38)
            self.origin_y = int(event.height * 0.30)
            # Auto-fit scale so leg fills ~60% of canvas
            L1, L2 = self.L1.get(), self.L2.get()
            max_r  = L1 + L2
            margin = 80
            if max_r > 0:
                self.scale = min(
                    (event.width  * 0.6 - margin) / max_r,
                    (event.height * 0.8 - margin) / max_r,
                )
                self.scale = max(self._zoom_min, min(self._zoom_max, self.scale))
        self._draw_sagittal()

    def _on_resize_top(self, event):
        # Initialize top-down view on first resize
        if self.origin_x_top == 0:
            self.origin_x_top = event.width // 2
            self.origin_y_top = event.height // 2
            # Auto-fit scale for reach circle
            L1, L2 = self.L1.get(), self.L2.get()
            max_r = L1 + L2
            margin = 40
            if max_r > 0:
                self.scale_top = min(
                    (event.width  * 0.8 - margin) / (max_r * 2),
                    (event.height * 0.8 - margin) / (max_r * 2),
                )
                self.scale_top = max(self._zoom_min_top, min(self._zoom_max_top, self.scale_top))
        self._draw_topdown()

    def _w2c(self, wx, wy):
        return self.origin_x + wx * self.scale, self.origin_y - wy * self.scale

    def _c2w(self, cx, cy):
        return (cx - self.origin_x) / self.scale, -(cy - self.origin_y) / self.scale

    def _draw(self):
        """Unified draw method that calls both views."""
        self._draw_sagittal()
        self._draw_topdown()

    def _draw_sagittal(self):
        c = self.canvas_side
        c.delete("all")
        W, H = c.winfo_width(), c.winfo_height()
        if W < 10 or H < 10:
            return
        L1, L2, L3 = self.L1.get(), self.L2.get(), self.L3.get()
        sc = self.scale
        ox, oy = self.origin_x, self.origin_y

        c.create_rectangle(0, 0, W, H, fill=C["bg"], outline="")

        # grid
        step = 40
        for gx in range(ox % step, W, step):
            c.create_line(gx, 0, gx, H, fill=C["grid"])
        for gy in range(oy % step, H, step):
            c.create_line(0, gy, W, gy, fill=C["grid"])

        # workspace
        mr = (L1+L2)*sc
        c.create_oval(ox-mr, oy-mr, ox+mr, oy+mr,
                      fill=C["reach"], outline=C["reach_bd"], dash=(4,4))

        # axes
        c.create_line(ox-mr-20, oy, ox+mr+20, oy, fill=C["axis"])
        c.create_line(ox, 20, ox, H-20, fill=C["axis"])
        c.create_text(ox+mr+8, oy-8, text="X", fill=C["text"], font=("Courier New", 8))
        c.create_text(ox+8, 14,     text="Y", fill=C["text"], font=("Courier New", 8))

        # ruler ticks
        for mm in range(-int(mr/sc+50), int(mr/sc+51), 50):
            px = ox + mm*sc
            if 0 < px < W:
                c.create_line(px, oy-4, px, oy+4, fill=C["axis"])
                c.create_text(px, oy+12, text=str(mm),
                              fill=C["section"], font=("Courier New", 6))

        r = self.ik_result

        def cp(wx, wy):
            return self._w2c(wx, wy)

        if r is None:
            ang = math.atan2(self.target_y, self.target_x)
            pts = [cp(0,0),
                   cp(L1*math.cos(ang), L1*math.sin(ang)),
                   cp((L1+L2)*math.cos(ang),(L1+L2)*math.sin(ang))]
            for i in range(2):
                c.create_line(*pts[i], *pts[i+1],
                              fill=C["section"], width=4, capstyle=tk.ROUND)
        else:
            j0 = cp(*r.j0)
            j1 = cp(*r.j1)
            j2 = cp(*r.j2)
            j3 = cp(*r.j3)

            # ── Joint limit wedges (drawn before bones so bones sit on top) ──
            if self._show_limits.get():
                lims = self._get_limits_rad()
                limit_specs = [
                    (j0, lims[0], C["bone_hip"],  60),   # shoulder
                    (j1, lims[1], C["bone_knee"], 50),   # thigh/knee
                    (j2, lims[2], C["bone_ank"],  40),   # shin
                ]
                for (lx, ly), (l_lo, l_hi), lcol, lr in limit_specs:
                    # Filled wedge = allowed range.
                    # tkinter arc: 0° = 3-o-clock (+X), increases CCW — same as IK convention.
                    # So convert radians straight to degrees WITHOUT negating.
                    lo_deg = math.degrees(l_lo)
                    hi_deg = math.degrees(l_hi)
                    start_d  = min(lo_deg, hi_deg)
                    extent_d = abs(hi_deg - lo_deg)
                    if extent_d > 0.5:
                        c.create_arc(lx-lr, ly-lr, lx+lr, ly+lr,
                                     start=start_d, extent=extent_d,
                                     style="pieslice",
                                     fill=lcol, outline="", stipple="gray25")
                    # Limit boundary lines — use positive angle (screen Y is flipped, so sin negated)
                    for ang_r in (l_lo, l_hi):
                        ex = lx + lr * math.cos(ang_r)
                        ey = ly - lr * math.sin(ang_r)   # subtract because canvas Y increases downward
                        c.create_line(lx, ly, ex, ey,
                                      fill=lcol, width=1, dash=(3, 3))

            # ── Angle arcs (show actual current angle) ──────────────────────
            ar = 22
            def arc(px, py, a_from, a_to, col):
                if abs(a_to - a_from) < 0.02:
                    return
                lo = min(-math.degrees(a_from), -math.degrees(a_to))
                hi = max(-math.degrees(a_from), -math.degrees(a_to))
                extent = hi - lo
                if abs(extent) < 0.5:
                    return
                c.create_arc(px-ar, py-ar, px+ar, py+ar,
                             start=lo, extent=extent,
                             style="arc", outline=col, width=1)

            arc(*j0, 0, r.t1, C["bone_hip"])
            arc(*j1, r.t1, r.t1+r.t2, C["bone_knee"])
            arc(*j2, r.t1+r.t2, r.t1+r.t2+r.t3, C["bone_ank"])

            # bones
            for p1, p2, col in [(j0, j1, C["bone_hip"]),
                                 (j1, j2, C["bone_knee"]),
                                 (j2, j3, C["bone_ank"])]:
                c.create_line(*p1, *p2, fill=col, width=6, capstyle=tk.ROUND)

            # joints
            def joint(p, col, r_px=9):
                x, y = p
                c.create_oval(x-r_px, y-r_px, x+r_px, y+r_px,
                              fill=C["joint_bg"], outline=C["joint"], width=2)
                c.create_oval(x-r_px+2, y-r_px+2, x+r_px-2, y+r_px-2,
                              fill=col, outline="")

            joint(j0, C["bone_hip"], 10)
            joint(j1, C["bone_knee"], 9)
            joint(j2, C["bone_ank"], 8)

            fx, fy = j3
            c.create_oval(fx-6, fy-6, fx+6, fy+6,
                          fill=C["foot"], outline=C["joint"], width=1)

            for p, lbl, col in [(j0,"HIP/SHLD",C["bone_hip"]),
                                 (j1,"KNEE",C["bone_knee"]),
                                 (j2,"FOOT",C["bone_ank"])]:
                c.create_text(p[0]+14, p[1]-6, text=lbl,
                              fill=col, font=("Courier New",7), anchor="w")

        # target crosshair
        tx, ty = cp(self.target_x, self.target_y)
        c.create_oval(tx-14, ty-14, tx+14, ty+14, fill=C["target_r"], outline="")
        c.create_line(tx-10, ty, tx+10, ty, fill=C["target"], width=2)
        c.create_line(tx, ty-10, tx, ty+10, fill=C["target"], width=2)
        c.create_oval(tx-3, ty-3, tx+3, ty+3, fill=C["target"], outline="")
        c.create_text(tx+16, ty-8,
                      text=f"({self.target_x:.0f}, {self.target_y:.0f})",
                      fill=C["text"], font=("Courier New",8), anchor="w")

    def _draw_topdown(self):
        """Draw top-down view showing shoulder tilt and leg reach."""
        c = self.canvas_top
        c.delete("all")
        W, H = c.winfo_width(), c.winfo_height()
        if W < 10 or H < 10:
            return
        
        L1, L2 = self.L1.get(), self.L2.get()
        sc = self.scale_top
        ox, oy = self.origin_x_top, self.origin_y_top

        c.create_rectangle(0, 0, W, H, fill=C["bg"], outline="")

        # Grid
        step = 40
        for gx in range(ox % step, W, step):
            c.create_line(gx, 0, gx, H, fill=C["grid"])
        for gy in range(oy % step, H, step):
            c.create_line(0, gy, W, gy, fill=C["grid"])

        # Reach circle (viewed from top)
        mr = (L1 + L2) * sc
        c.create_oval(ox-mr, oy-mr, ox+mr, oy+mr,
                      fill=C["reach"], outline=C["reach_bd"], dash=(4,4))

        # Axes (Y/Z plane — vertical/lateral, since shoulder rotates about X-axis)
        c.create_line(ox-mr-20, oy, ox+mr+20, oy, fill=C["axis"])  # Z axis (lateral)
        c.create_line(ox, 20, ox, H-20, fill=C["axis"])  # Y axis (vertical)
        c.create_text(ox+mr+8, oy-8, text="Z", fill=C["text"], font=("Courier New", 8))
        c.create_text(ox+8, 14,     text="Y", fill=C["text"], font=("Courier New", 8))

        # Ruler ticks
        for mm in range(-int(mr/sc+50), int(mr/sc+51), 50):
            px = ox + mm*sc
            if 0 < px < W:
                c.create_line(px, oy-4, px, oy+4, fill=C["axis"])
                c.create_text(px, oy+12, text=str(mm),
                              fill=C["section"], font=("Courier New", 6))

        # Shoulder tilt angle limits (wedge visualization)
        shoulder_lims = self._get_limits_rad()[0]  # (lo, hi) in radians
        shoulder_deg_lo = math.degrees(shoulder_lims[0])
        shoulder_deg_hi = math.degrees(shoulder_lims[1])
        extent = abs(shoulder_deg_hi - shoulder_deg_lo)
        shoulder_deg_lo = min(shoulder_deg_lo, shoulder_deg_hi)
        if extent > 0.5:
            wedge_r = max(mr * 0.3, 60)
            c.create_arc(ox-wedge_r, oy-wedge_r, ox+wedge_r, oy+wedge_r,
                        start=shoulder_deg_lo, extent=extent,
                        style="pieslice",
                        fill=C["bone_hip"], outline="", stipple="gray25")

        # Current shoulder tilt (leg projection)
        shoulder_rad = math.radians(self.shoulder_angle.get())
        
        # Draw leg in top-down view, rotated by shoulder angle
        if self.ik_result is not None:
            r = self.ik_result
            # Thigh end point (knee) — use Y (vertical) component and rotate it about X-axis
            # This gives Z (lateral) offset. Y stays constant in top view.
            knee_y_sag = r.j_knee[1]   # sagittal Y (vertical in leg frame)
            # Z offset from shoulder rotation: Y * sin(shoulder_angle)
            knee_z = knee_y_sag * math.sin(shoulder_rad)
            # Horizontal (Z) position becomes: ox + Z_offset
            # Vertical (Y) position stays: oy - knee_y_sag * cos(shoulder_rad) (inverted for screen coords)
            kx_px = ox + knee_z * sc
            ky_px = oy - knee_y_sag * sc * math.cos(shoulder_rad)

            # Foot end point
            foot_y_sag = r.j_foot[1]
            foot_z = foot_y_sag * math.sin(shoulder_rad)
            fx_px = ox + foot_z * sc
            fy_px = oy - foot_y_sag * sc * math.cos(shoulder_rad)

            # Draw thigh bone
            c.create_line(ox, oy, kx_px, ky_px, fill=C["bone_knee"], width=6, capstyle=tk.ROUND)

            # Draw shin bone
            c.create_line(kx_px, ky_px, fx_px, fy_px, fill=C["bone_ank"], width=5, capstyle=tk.ROUND)

            # Draw joints
            c.create_oval(ox-8, oy-8, ox+8, oy+8, fill=C["joint_bg"], outline=C["joint"], width=2)
            c.create_oval(ox-6, oy-6, ox+6, oy+6, fill=C["bone_hip"], outline="")
            
            c.create_oval(kx_px-7, ky_px-7, kx_px+7, ky_px+7, fill=C["joint_bg"], outline=C["joint"], width=1)
            c.create_oval(kx_px-5, ky_px-5, kx_px+5, ky_px+5, fill=C["bone_knee"], outline="")
            
            c.create_oval(fx_px-6, fy_px-6, fx_px+6, fy_px+6, fill=C["foot"], outline=C["joint"], width=1)

            # Labels
            c.create_text(ox+12, oy-10, text="HIP", fill=C["bone_hip"], font=("Courier New", 7), anchor="w")
            c.create_text(kx_px+10, ky_px-6, text="KNEE", fill=C["bone_knee"], font=("Courier New", 7), anchor="w")
        
        # Current shoulder angle indicator
        shoulder_deg = self.shoulder_angle.get()   # already degrees
        c.create_text(ox, oy-mr-20, text=f"Shoulder: {shoulder_deg:+.1f}°",
                      fill=C["text2"], font=("Courier New", 8), anchor="n")

    # ── IK solve + panel ───────────────────────────────────────────────────────

    def _get_limits_rad(self):
        """Return current joint limits as (lo_rad, hi_rad) tuples from the live UI vars."""
        return (
            (math.radians(self.lim_shoulder_lo.get()), math.radians(self.lim_shoulder_hi.get())),
            (math.radians(self.lim_thigh_lo.get()),  math.radians(self.lim_thigh_hi.get())),
            (math.radians(self.lim_knee_lo.get()), math.radians(self.lim_knee_hi.get())),
            (math.radians(self.lim_shin_lo.get()), math.radians(self.lim_shin_hi.get())),
        )

    def _solve_and_draw(self, *_):
        L1, L2, L3 = self.L1.get(), self.L2.get(), self.L3.get()
        shoulder_rad = math.radians(self.shoulder_angle.get())   # shoulder_angle is in degrees
        r = solve(self.target_x, self.target_y, L1, L2, L3,
                  solution=self.solution_var.get(),
                  shin_is_absolute=config.SHIN_IS_ABSOLUTE,
                  shoulder_to_hip=config.SHOULDER_TO_HIP_MM,
                  tz=0.0)
        self.ik_result = r
        self.reachable = r is not None

        self.lbl_reach.config(
            text="  REACHABLE  " if r else " OUT OF REACH ",
            bg=C["ok"] if r else C["warn"])

        self._update_angle_panel(r)
        self._update_math_panel(r)
        self._draw()

        self.entry_x.delete(0, tk.END)
        self.entry_x.insert(0, f"{self.target_x:.1f}")
        self.entry_y.delete(0, tk.END)
        self.entry_y.insert(0, f"{self.target_y:.1f}")

        if self.can_connected and self._motors_armed and r is not None:
            now = time.monotonic()
            if now - self._last_send_t >= self._send_interval:
                self._send_joint_commands(r)
                self._last_send_t = now

    def _update_angle_panel(self, r: IKResult | None):
        # Shoulder angle (from slider, always valid)
        shoulder_deg = self.shoulder_angle.get()            # already in degrees
        shoulder_rad_val = math.radians(shoulder_deg)
        self._angle_vars[0].set(f"{shoulder_rad_val:+.4f}  {shoulder_deg:+.1f}°")
        
        # Thigh and shin angles (from IK result)
        angles = ([self.shoulder_angle.get()] + [r.t2, r.t3] if r else [0.0, 0.0, 0.0])
        limits = self._get_limits_rad()
        if r:
            # Thigh angle with correct limit index (was using limits[2] instead of limits[1])
            ang = r.t2
            lo, hi = limits[1][0], limits[1][1]  # Thigh limits
            ok = lo <= ang <= hi
            flag = "  !" if not ok else ""
            self._angle_vars[1].set(f"{ang:+.4f}  {ang*DEG:+.1f}°{flag}")
            bc, col = self._bar_canvases[1]
            bc.delete("all")
            bw = bc.winfo_width() or 200
            bh = bc.winfo_height() or 4
            bc.create_rectangle(0, 0, bw, bh, fill=C["bar_bg"], outline="")
            lo_pct = clamp((lo + math.pi) / (2 * math.pi), 0, 1)
            hi_pct = clamp((hi + math.pi) / (2 * math.pi), 0, 1)
            zone_col = col if ok else C["warn"]
            bc.create_rectangle(int(bw * lo_pct), 0, int(bw * hi_pct), bh,
                                 fill=zone_col, outline="")
            cur_pct = clamp((ang + math.pi) / (2 * math.pi), 0, 1)
            cx_ = int(bw * cur_pct)
            cursor_col = C["warn"] if not ok else "#ffffff"
            bc.create_rectangle(max(0, cx_ - 1), 0, min(bw, cx_ + 2), bh,
                                 fill=cursor_col, outline="")
            
            # Shin angle with correct limit index
            ang = r.t3
            lo, hi = limits[3][0], limits[3][1]  # Shin limits
            ok = lo <= ang <= hi
            flag = "  !" if not ok else ""
            self._angle_vars[2].set(f"{ang:+.4f}  {ang*DEG:+.1f}°{flag}")
            bc, col = self._bar_canvases[2]
            bc.delete("all")
            bw = bc.winfo_width() or 200
            bh = bc.winfo_height() or 4
            bc.create_rectangle(0, 0, bw, bh, fill=C["bar_bg"], outline="")
            lo_pct = clamp((lo + math.pi) / (2 * math.pi), 0, 1)
            hi_pct = clamp((hi + math.pi) / (2 * math.pi), 0, 1)
            zone_col = col if ok else C["warn"]
            bc.create_rectangle(int(bw * lo_pct), 0, int(bw * hi_pct), bh,
                                 fill=zone_col, outline="")
            cur_pct = clamp((ang + math.pi) / (2 * math.pi), 0, 1)
            cx_ = int(bw * cur_pct)
            cursor_col = C["warn"] if not ok else "#ffffff"
            bc.create_rectangle(max(0, cx_ - 1), 0, min(bw, cx_ + 2), bh,
                                 fill=cursor_col, outline="")

    def _update_math_panel(self, r: IKResult | None):
        self.math_text.config(state=tk.NORMAL)
        self.math_text.delete("1.0", tk.END)
        if r is None:
            d = math.hypot(self.target_x, self.target_y)
            lines = [
                f"target  ({self.target_x:.1f}, {self.target_y:.1f}) mm",
                f"dist    {d:.2f} mm",
                f"max     {self.L1.get()+self.L2.get():.1f} mm",
                "", "NO SOLUTION",
            ]
        else:
            h = r.t1 + self.off_hip.get()
            k = r.t2 + self.off_knee.get()
            a = r.t3 + self.off_ankle.get()
            lines = [
                f"target  ({self.target_x:.1f}, {self.target_y:.1f}) mm",
                f"wrist   ({r.wrist_x:.2f}, {r.wrist_y:.2f})",
                f"|wrist| {r.wrist_dist:.3f} mm",
                "",
                f"cos(θ₂) {r.cos_knee:.5f}",
                f"θ₂ raw  {math.degrees(r.knee_ang):.3f}°",
                f"α       {math.degrees(r.alpha):.3f}°",
                f"α₂      {math.degrees(r.alpha2):.3f}°",
                "─────────────────────",
                f"θ₁ shld  {r.t1:+.4f} rad",
                f"θ₂ thigh {r.t2:+.4f} rad",
                f"θ₃ shin  {r.t3:+.4f} rad (abs={config.SHIN_IS_ABSOLUTE})",
                "─────────────────────",
                f"+ off shld  {h:+.4f} rad",
                f"+ off thigh {k:+.4f} rad",
                f"+ off shin  {a:+.4f} rad",
                "─────────────────────",
                f"lim shld  [{math.degrees(self._get_limits_rad()[0][0]):+.0f}, {math.degrees(self._get_limits_rad()[0][1]):+.0f}]°",
                f"lim thgh  [{math.degrees(self._get_limits_rad()[1][0]):+.0f}, {math.degrees(self._get_limits_rad()[1][1]):+.0f}]°",
                f"lim shin  [{math.degrees(self._get_limits_rad()[2][0]):+.0f}, {math.degrees(self._get_limits_rad()[2][1]):+.0f}]°",
                f"knee incl {r.knee_angle_deg:+.2f}°",
                f"PDO{config.USE_PDO} → 8-byte float32 LE",
            ]
        self.math_text.insert(tk.END, "\n".join(lines))
        self.math_text.config(state=tk.DISABLED)

    # ── Telemetry polling ──────────────────────────────────────────────────────

    def _poll_telemetry(self):
        dids = [self.did_hip.get(), self.did_knee.get(), self.did_ankle.get()]
        for i, did in enumerate(dids):
            t = self.can.get_telemetry(did)
            if t:
                age = time.monotonic() - t.timestamp
                if age < 2.0:
                    self._telem_vars[i].set(f"{t.position_rad:+.4f} rad")
                else:
                    self._telem_vars[i].set("stale")
            else:
                self._telem_vars[i].set("—")
        self.after(100, self._poll_telemetry)

    # ── CAN operations ─────────────────────────────────────────────────────────

    def _applied_angles(self, r: IKResult) -> tuple[float, float, float, bool]:
        """Return (shoulder, thigh, shin, has_violation) with offsets, clamped to live limits."""
        lims = self._get_limits_rad()
        shoulder = clamp(self.shoulder_angle.get(), *lims[0])
        thigh   = clamp(r.t2 + self.off_hip.get(),  *lims[1])
        shin  = clamp(r.t3 + self.off_ankle.get(), *lims[2])
        
        # Check for violations
        shoulder_ok = (self.shoulder_angle.get() == shoulder)
        thigh_ok = ((r.t2 + self.off_hip.get()) == thigh)
        shin_ok = ((r.t3 + self.off_ankle.get()) == shin)
        has_violation = not (shoulder_ok and thigh_ok and shin_ok)
        
        return shoulder, thigh, shin, has_violation

    def _send_joint_commands(self, r: IKResult) -> bool:
        shoulder, thigh, shin, has_violation = self._applied_angles(r)
        self._last_limit_violation = has_violation
        
        # Update warning label
        if has_violation:
            self.lbl_limit_warn.config(text="⚠ Joint limit violation — command clamped")
        else:
            self.lbl_limit_warn.config(text="")
        
        # Block send if limits violated
        if has_violation and not self._simulation_mode:
            self._log_line("BLOCKED: Joint limit violation", error=True)
            return False
        
        vel  = config.VELOCITY_FEEDFORWARD
        torq = config.TORQUE_FEEDFORWARD
        dids = [self.did_hip.get(), self.did_knee.get(), self.did_ankle.get()]
        angs = [shoulder, thigh, shin]
        names = ["SHLD", "THGH", "SHIN"]

        ok_all = True
        
        # If in simulation mode, just log without sending
        if self._simulation_mode:
            self._log_tx(dids, angs, names, sim=True)
            return True
        
        # Otherwise, send to hardware
        for did, ang, name in zip(dids, angs, names):
            if config.USE_PDO == 2:
                ok = bool(self.can.send_pdo2(did, ang, vel))
            else:
                ok = bool(self.can.send_pdo3(did, ang, torq))
            ok_all = ok_all and ok

        self._log_tx(dids, angs, names)
        return ok_all

    def _arm_motors(self):
        """Send NMT MODE_POSITION to all three joints to enable position control."""
        if not self.can_connected:
            self._log_line("NOT CONNECTED — cannot arm", error=True)
            return
        dids = [self.did_hip.get(), self.did_knee.get(), self.did_ankle.get()]
        for did in dids:
            self.can.set_mode(did, config.MODE_POSITION)
        self._motors_armed = True
        self.btn_arm.config(bg=C["ok"], fg="#ffffff", text="ARMED ✓")
        self._log_line(f"NMT → MODE_POSITION sent to nodes {dids}")

    def _manual_send(self):
        if not self.can_connected and not self._simulation_mode:
            self._log_line("NOT CONNECTED", error=True)
            return
        if self.ik_result is None:
            self._log_line("NO IK SOLUTION — not sending", error=True)
            return
        if not self._motors_armed and not self._simulation_mode:
            self._log_line("Motors not armed — click ARM MOTORS first", error=True)
            return
        self._send_joint_commands(self.ik_result)

    def _toggle_simulation(self):
        """Toggle simulation-only mode (logs CAN frames without hardware send)."""
        self._simulation_mode = not self._simulation_mode
        
        if self._simulation_mode:
            self.btn_simulate.config(bg=C["can_sim"], text="■ SIMULATE (ON)")
            self._log_line("[SIMULATION MODE ON] — commands logged but NOT sent to hardware")
        else:
            self.btn_simulate.config(bg=C["bar_bg"], text="▶ SIMULATE (log only, no hardware)")
            self._log_line("[SIMULATION MODE OFF] — back to normal operation")

    def _log_tx(self, dids, angs, names, sim=False):
        lines = []
        if sim:
            lines.append(f"[{time.strftime('%H:%M:%S')}]  [SIM]  PDO{config.USE_PDO}")
        else:
            lines.append(f"[{time.strftime('%H:%M:%S')}]  PDO{config.USE_PDO}")
        for did, ang, name in zip(dids, angs, names):
            raw = struct.pack("<f", ang).hex().upper()
            can_id = (0x04 if config.USE_PDO == 2 else 0x05) << 7 | did
            lines.append(f"  0x{can_id:03X}  {name:5s}  {ang:+.4f} rad  [{raw}]")
        self._log_line("\n".join(lines))

    def _log_line(self, text, error=False):
        self.can_log.config(state=tk.NORMAL)
        col = C["can_err"] if error else (C["can_sim"] if self.can.simulation_mode else C["can_ok"])
        self.can_log.config(fg=col)
        self.can_log.insert(tk.END, text + "\n")
        self.can_log.see(tk.END)
        n = int(self.can_log.index("end-1c").split(".")[0])
        if n > 100:
            self.can_log.delete("1.0", f"{n-80}.0")
        self.can_log.config(state=tk.DISABLED)

    # ── CAN connect / disconnect ───────────────────────────────────────────────

    def _toggle_can(self):
        if self.can_connected:
            self._stop_heartbeat()
            self.can.disconnect()
            self.can_connected = False
            self._motors_armed = False
            self.btn_arm.config(bg=C["bar_bg"], fg=C["text2"], text="ARM MOTORS")
        else:
            ok = self.can.connect()
            self.can_connected = ok
            if ok:
                self._start_heartbeat()
                mode = "SIM" if self.can.simulation_mode else "HW"
                self._log_line(f"Connected [{mode}]  {config.CAN_CHANNEL}  {config.CAN_BITRATE} bps")
            else:
                self._log_line(f"CONNECT FAILED — {config.CAN_CHANNEL}", error=True)
        self._refresh_status()

    def _refresh_status(self):
        if self.can_connected:
            col  = C["can_sim"] if self.can.simulation_mode else C["can_ok"]
            txt  = f"● {'SIM' if self.can.simulation_mode else config.CAN_CHANNEL}"
            btxt = "■ DISCONNECT"
        else:
            col  = C["can_err"]
            txt  = "○ NOT CONNECTED"
            btxt = "● CONNECT CAN"
        self.lbl_status.config(text=txt, fg=col)
        self.btn_connect.config(text=btxt, fg=col)
        self.after(1000, self._refresh_status)

    # ── Heartbeat thread ───────────────────────────────────────────────────────

    def _start_heartbeat(self):
        self._hb_stop.clear()
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="HB")
        self._hb_thread.start()

    def _stop_heartbeat(self):
        self._hb_stop.set()
        if self._hb_thread:
            self._hb_thread.join(timeout=1.0)

    def _heartbeat_loop(self):
        interval = config.HEARTBEAT_INTERVAL_MS / 1000.0
        dids = [self.did_hip.get(), self.did_knee.get(), self.did_ankle.get()]
        while not self._hb_stop.is_set():
            if self.can_connected:
                self.can.send_heartbeat_all(dids)
            time.sleep(interval)

    # ── Mouse drag / zoom / pan ────────────────────────────────────────────────

    def _hit_target(self, cx, cy, thresh=20):
        tx, ty = self._w2c(self.target_x, self.target_y)
        return math.hypot(cx-tx, cy-ty) < thresh

    def _on_mouse_down(self, event):
        if self._hit_target(event.x, event.y):
            self._dragging = True

    def _on_mouse_drag(self, event):
        if self._dragging:
            wx, wy = self._c2w(event.x, event.y)
            self.target_x = round(wx, 1)
            self.target_y = round(wy, 1)
            self._solve_and_draw()

    def _on_mouse_up(self, event):
        self._dragging = False

    # ── Top-down view mouse handlers ───────────────────────────────────────

    def _on_mouse_down_top(self, event):
        # Optional: allow clicking on targets in top view (future enhancement)
        pass

    def _on_mouse_drag_top(self, event):
        # Optional: allow dragging to control shoulder angle in top view
        pass

    def _on_pan_start_top(self, event):
        self._panning_top = True
        self._pan_start_top = (event.x, event.y)
        self.canvas_top.config(cursor="fleur")

    def _on_pan_drag_top(self, event):
        if not self._panning_top:
            return
        dx = event.x - self._pan_start_top[0]
        dy = event.y - self._pan_start_top[1]
        self.origin_x_top += dx
        self.origin_y_top += dy
        self._pan_start_top = (event.x, event.y)
        self._draw_topdown()

    def _on_pan_end_top(self, event):
        self._panning_top = False
        self.canvas_top.config(cursor="crosshair")

    def _on_zoom_top(self, event):
        # Zoom for top view
        if event.num == 4 or (hasattr(event, 'delta') and event.delta > 0):
            factor = 1.15
        else:
            factor = 1.0 / 1.15

        new_scale = self.scale_top * factor
        new_scale = max(self._zoom_min_top, min(self._zoom_max_top, new_scale))
        if new_scale == self.scale_top:
            return
        self.scale_top = new_scale
        self._draw_topdown()

    # ── Zoom ─────────────────────────────────────────────────────────────────

    def _on_zoom(self, event):
        # Determine zoom direction
        if event.num == 4 or (hasattr(event, 'delta') and event.delta > 0):
            factor = 1.15
        else:
            factor = 1.0 / 1.15

        new_scale = self.scale * factor
        new_scale = max(self._zoom_min, min(self._zoom_max, new_scale))
        if new_scale == self.scale:
            return

        # Zoom toward the cursor position on side view
        cx, cy = event.x, event.y
        wx, wy = self._c2w(cx, cy)
        self.scale = new_scale
        # Recompute origin so (wx,wy) stays under the cursor
        self.origin_x = cx - wx * self.scale
        self.origin_y = cy + wy * self.scale
        self._draw_sagittal()

    def _zoom_to_fit(self, event=None):
        W = self.canvas_side.winfo_width()
        H = self.canvas_side.winfo_height()
        if W < 10 or H < 10:
            return
        L1, L2 = self.L1.get(), self.L2.get()
        max_r = L1 + L2
        margin = 60
        fit_scale = min(
            (W - margin * 2) / (max_r * 2),
            (H - margin * 2) / (max_r * 2),
        )
        self.scale    = max(self._zoom_min, min(self._zoom_max, fit_scale))
        self.origin_x = W * 0.38
        self.origin_y = H * 0.30
        self._draw_sagittal()

    # ── Pan ──────────────────────────────────────────────────────────────────

    def _on_pan_start(self, event):
        self._panning   = True
        self._pan_start = (event.x, event.y)
        self.canvas_side.config(cursor="fleur")

    def _on_pan_drag(self, event):
        if not self._panning:
            return
        dx = event.x - self._pan_start[0]
        dy = event.y - self._pan_start[1]
        self.origin_x += dx
        self.origin_y += dy
        self._pan_start = (event.x, event.y)
        self._draw_sagittal()

    def _on_pan_end(self, event):
        self._panning = False
        self.canvas_side.config(cursor="crosshair")

    # ── Coord entry ────────────────────────────────────────────────────────────

    def _apply_coords(self, *_):
        try:
            self.target_x = float(self.entry_x.get())
            self.target_y = float(self.entry_y.get())
            self._solve_and_draw()
        except ValueError:
            pass

    def _on_shoulder_slider(self, value):
        """Called when shoulder slider moves. shoulder_angle stores DEGREES."""
        deg = float(value)
        rad = math.radians(deg)
        self.shoulder_angle.set(deg)          # store degrees
        self.shoulder_angle_label.config(text=f"{rad:+.4f}  {deg:+.0f}°")
        self._solve_and_draw()

    # ── Limit helpers ──────────────────────────────────────────────────────────

    def _reset_limits(self):
        """Restore joint limits to the values in config.py."""
        self.lim_shoulder_lo.set(math.degrees(config.SHOULDER_LIMIT_RAD[0]))
        self.lim_shoulder_hi.set(math.degrees(config.SHOULDER_LIMIT_RAD[1]))
        self.lim_thigh_lo.set(math.degrees(config.HIP_LIMIT_RAD[0]))
        self.lim_thigh_hi.set(math.degrees(config.HIP_LIMIT_RAD[1]))
        self.lim_knee_lo.set(math.degrees(config.KNEE_LIMIT_RAD[0]))
        self.lim_knee_hi.set(math.degrees(config.KNEE_LIMIT_RAD[1]))
        self.lim_shin_lo.set(math.degrees(config.ANKLE_LIMIT_RAD[0]))
        self.lim_shin_hi.set(math.degrees(config.ANKLE_LIMIT_RAD[1]))
        self._solve_and_draw()

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def on_close(self):
        self._stop_heartbeat()
        if self.can_connected and self._motors_armed:
            dids = [self.did_hip.get(), self.did_knee.get(), self.did_ankle.get()]
            for did in dids:
                self.can.set_mode(did, config.MODE_IDLE)
        self.can.disconnect()
        self.destroy()


if __name__ == "__main__":
    # ── Windows DPI awareness — must be called before any tkinter window opens ──
    if platform.system() == "Windows":
        try:
            # Per-monitor DPI awareness (Windows 10 1703+)
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except Exception:
            try:
                # Fallback: system DPI awareness (Windows Vista+)
                import ctypes
                ctypes.windll.user32.SetProcessDPIAware()
            except Exception:
                pass

    app = App()

    # Tell tkinter to scale fonts/widgets for the actual DPI
    if platform.system() == "Windows":
        try:
            # Get the real DPI from the primary monitor
            import ctypes
            dpi = ctypes.windll.shcore.GetDpiForSystem()
            # 96 DPI = 100% scaling (baseline)
            if dpi and dpi != 96:
                scale = dpi / 96.0
                app.tk.call("tk", "scaling", scale)
        except Exception:
            pass

    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()

