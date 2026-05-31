import queue
import tkinter as tk
from tkinter import ttk
from collections import defaultdict
import json
import math
import time
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator
from matplotlib.patches import Polygon, Rectangle
from itertools import cycle
from messages import PauseMessage, ResumeMessage, StopMessage, ToggleGripperMessage, PathAssignmentMessage, Waypoint, MotionSettings



class TelemetryGUI:
    """
    Thread-safe telemetry dashboard.

    Server threads call:
        gui.update_robot(robot_id, telemetry_dict)

    GUI runs on main thread and consumes queued updates.
    """

    # Sensor geometry
    MAX_VALID_ULTRASONIC_CM = 200.0
    MIN_VALID_ULTRASONIC_CM = 1.0

    FRONT_SENSOR_FORWARD_OFFSET_CM = 9.5
    FRONT_SENSOR_LATERAL_OFFSET_CM = 0.0
    FRONT_SENSOR_ANGLE_OFFSET_DEG = 0.0

    LEFT_SENSOR_FORWARD_OFFSET_CM = 9.5
    LEFT_SENSOR_LATERAL_OFFSET_CM = 7.0
    LEFT_SENSOR_ANGLE_OFFSET_DEG = 0.0

    RIGHT_SENSOR_FORWARD_OFFSET_CM = 9.5
    RIGHT_SENSOR_LATERAL_OFFSET_CM = -7.0
    RIGHT_SENSOR_ANGLE_OFFSET_DEG = 0.0

    # Arena display settings: fixed 4m x 4m = 400cm x 400cm
    ARENA_SIZE_CM = 400
    HALF_ARENA_CM = ARENA_SIZE_CM / 2
    GRID_SPACING_CM = 10
    ECHO_WINDOW_S = 10.0
    GRID_DIM_CELLS = 40
    DEFAULT_TEST_DISTANCE_CM = 30.0
    DEFAULT_TURN_SPEED = 150
    DEFAULT_DRIVE_SPEED = 200
    PERMANENT_OBSTACLES_PATH = Path(__file__).with_name("permanent_obstacles.json")

    # Robot safety box
    SAFETY_BOX_SIZE_CM = 40.0
    SAFETY_BOX_HALF_CM = SAFETY_BOX_SIZE_CM / 2.0

    # ----------------------------
    # Toyota theme palette
    # ----------------------------
    C_BG = "#15171c"        # app background (near-black)
    C_PANEL = "#1e2128"     # card surfaces
    C_PANEL_2 = "#262a33"   # inputs / nested surfaces
    C_PANEL_3 = "#2f343f"   # button idle
    C_BORDER = "#333a47"
    C_TEXT = "#e8eaed"
    C_TEXT_DIM = "#9aa0a8"
    C_RED = "#EB0A1E"       # Toyota red
    C_RED_DK = "#b80818"
    C_PLOT_BG = "#1b1e25"
    C_GRID_MAJ = "#3a4150"
    C_GRID_MIN = "#272b34"
    FONT_FAMILY = "Helvetica Neue"

    # State -> row/text color for the fleet table and status cues.
    STATE_COLORS = {
        "executing_path": "#37E29A",
        "waypoint_reached": "#37E29A",
        "driving": "#37E29A",
        "moving": "#37E29A",
        "scanning": "#FFD43B",
        "replanning": "#FFD43B",
        "avoiding_obstacle": "#FFD43B",
        "blocked": "#FF5D6C",
        "needs_replan": "#FF5D6C",
        "idle": "#9aa0a8",
        "ready": "#00C2FF",
        "connected": "#00C2FF",
    }

    # Bright, distinct robot colors that pop on the dark arena.
    ROBOT_PALETTE = [
        "#00C2FF", "#FF7A00", "#37E29A", "#FF5D6C", "#B68CFF",
        "#FFD43B", "#FF5DA2", "#8AE234", "#4DD0E1", "#FFA94D",
    ]

    def __init__(self, command_sender=None):
        self.root = tk.Tk()
        self.root.title("Toyota Fleet Control")
        self.root.geometry("1680x940")
        self.root.minsize(1200, 720)
        self.root.configure(bg=self.C_BG)

        self._setup_style()

        self.command_sender = command_sender
        self.telemetry_queue = queue.Queue()
        self.test_path_counter = int(time.time())

        # latest per-robot state
        self.robot_states = {}

        # per-robot history
        self.robot_history = defaultdict(lambda: {
            "t": [],
            "x": [],
            "y": [],
            "theta": [],
            "front_ultra": [],
            "left_ultra": [],
            "right_ultra": [],
            "front_echo_t": [],
            "front_echo_x": [],
            "front_echo_y": [],
            "left_echo_t": [],
            "left_echo_x": [],
            "left_echo_y": [],
            "right_echo_t": [],
            "right_echo_x": [],
            "right_echo_y": [],
        })

        self.robot_colors = {}
        self.obstacle_cells = {}
        self.permanent_obstacle_cells = self._load_permanent_obstacles()
        self.priority_goal_cell = (5, 5)
        self.priority_goal_cm = (
            self.priority_goal_cell[1] * self.GRID_SPACING_CM + self.GRID_SPACING_CM / 2.0,
            self.priority_goal_cell[0] * self.GRID_SPACING_CM + self.GRID_SPACING_CM / 2.0,
        )
        self.color_cycle = cycle(self.ROBOT_PALETTE)

        # Arena view state: preserve user pan/zoom across live redraws.
        self._plot_initialized = False
        self._reset_view = False
        # Suppress the heavy 10 Hz rebuild while the user is actively panning or
        # zooming so the interaction stays smooth.
        self._mouse_down = False
        self._last_interact = 0.0

        self._build_layout()
        self.root.after(100, self._process_queue)

    # ----------------------------
    # Theme
    # ----------------------------
    def _setup_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        f = self.FONT_FAMILY

        style.configure(
            ".",
            background=self.C_PANEL,
            foreground=self.C_TEXT,
            fieldbackground=self.C_PANEL_2,
            bordercolor=self.C_BORDER,
            lightcolor=self.C_BORDER,
            darkcolor=self.C_BORDER,
            troughcolor=self.C_BG,
            font=(f, 10),
        )

        style.configure("TFrame", background=self.C_BG)
        style.configure("Card.TFrame", background=self.C_PANEL)
        style.configure("Header.TFrame", background=self.C_BG)
        style.configure("Accent.TFrame", background=self.C_RED)

        style.configure("TLabel", background=self.C_PANEL, foreground=self.C_TEXT, font=(f, 10))
        style.configure("Dim.TLabel", background=self.C_PANEL, foreground=self.C_TEXT_DIM, font=(f, 9))
        style.configure("Title.TLabel", background=self.C_BG, foreground=self.C_TEXT, font=(f, 20, "bold"))
        style.configure("Subtitle.TLabel", background=self.C_BG, foreground=self.C_TEXT_DIM, font=(f, 10))
        style.configure("Brand.TLabel", background=self.C_RED, foreground="#ffffff", font=(f, 18, "bold"))

        style.configure(
            "TLabelframe",
            background=self.C_PANEL,
            bordercolor=self.C_BORDER,
            relief="solid",
            borderwidth=1,
        )
        style.configure(
            "TLabelframe.Label",
            background=self.C_PANEL,
            foreground=self.C_RED,
            font=(f, 11, "bold"),
        )

        style.configure(
            "TButton",
            background=self.C_PANEL_3,
            foreground=self.C_TEXT,
            bordercolor=self.C_BORDER,
            focuscolor=self.C_PANEL_3,
            font=(f, 9, "bold"),
            padding=(10, 7),
            relief="flat",
        )
        style.map(
            "TButton",
            background=[("pressed", "#3a4150"), ("active", "#39404d")],
            foreground=[("disabled", self.C_TEXT_DIM)],
        )

        style.configure(
            "Accent.TButton",
            background=self.C_RED,
            foreground="#ffffff",
            bordercolor=self.C_RED,
            focuscolor=self.C_RED,
            font=(f, 9, "bold"),
            padding=(10, 8),
            relief="flat",
        )
        style.map(
            "Accent.TButton",
            background=[("pressed", self.C_RED_DK), ("active", self.C_RED_DK)],
        )

        style.configure(
            "Treeview",
            background=self.C_PANEL_2,
            fieldbackground=self.C_PANEL_2,
            foreground=self.C_TEXT,
            bordercolor=self.C_BORDER,
            rowheight=26,
            font=(f, 9),
        )
        style.configure(
            "Treeview.Heading",
            background="#2b303a",
            foreground=self.C_TEXT_DIM,
            font=(f, 9, "bold"),
            relief="flat",
            padding=(4, 6),
        )
        style.map("Treeview.Heading", background=[("active", "#343b47")])
        style.map(
            "Treeview",
            background=[("selected", self.C_RED)],
            foreground=[("selected", "#ffffff")],
        )

        style.configure(
            "TCombobox",
            fieldbackground=self.C_PANEL_2,
            background=self.C_PANEL_3,
            foreground=self.C_TEXT,
            arrowcolor=self.C_TEXT,
            bordercolor=self.C_BORDER,
            padding=(6, 5),
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", self.C_PANEL_2)],
            foreground=[("readonly", self.C_TEXT)],
        )

        style.configure(
            "TEntry",
            fieldbackground=self.C_PANEL_2,
            foreground=self.C_TEXT,
            bordercolor=self.C_BORDER,
            insertcolor=self.C_TEXT,
            padding=(6, 5),
        )

        style.configure("TRadiobutton", background=self.C_PANEL, foreground=self.C_TEXT, font=(f, 9))
        style.map(
            "TRadiobutton",
            background=[("active", self.C_PANEL)],
            indicatorcolor=[("selected", self.C_RED)],
        )

        style.configure(
            "Vertical.TScrollbar",
            background=self.C_PANEL_3,
            troughcolor=self.C_BG,
            bordercolor=self.C_BG,
            arrowcolor=self.C_TEXT_DIM,
        )
        style.map("Vertical.TScrollbar", background=[("active", "#39404d")])

        # Combobox drop-down list is a classic tk Listbox; theme it too.
        self.root.option_add("*TCombobox*Listbox.background", self.C_PANEL_2)
        self.root.option_add("*TCombobox*Listbox.foreground", self.C_TEXT)
        self.root.option_add("*TCombobox*Listbox.selectBackground", self.C_RED)
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")
        self.root.option_add("*TCombobox*Listbox.font", (f, 9))

    # ----------------------------
    # Public API
    # ----------------------------
    def update_robot(self, robot_id: str, telemetry: dict):
        self.telemetry_queue.put((robot_id, telemetry))

    def run(self):
        self.root.mainloop()

    # ----------------------------
    # Layout
    # ----------------------------
    
    def _build_layout(self):
        # ---- Header bar with Toyota branding ----
        header = ttk.Frame(self.root, style="Header.TFrame", padding=(14, 10))
        header.pack(side=tk.TOP, fill=tk.X)

        brand = ttk.Label(header, text="  TOYOTA  ", style="Brand.TLabel")
        brand.pack(side=tk.LEFT, padx=(0, 14))
        title_box = ttk.Frame(header, style="Header.TFrame")
        title_box.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(title_box, text="Fleet Control", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            title_box,
            text="Autonomous robot fleet · live arena telemetry",
            style="Subtitle.TLabel",
        ).pack(anchor="w")

        # Thin Toyota-red accent strip under the header.
        ttk.Frame(self.root, style="Accent.TFrame", height=3).pack(side=tk.TOP, fill=tk.X)

        mainframe = ttk.Frame(self.root, padding=10)
        mainframe.pack(fill=tk.BOTH, expand=True)

        # ---- Left panel: scrollable stack of control cards ----
        left_panel = ttk.Frame(mainframe, width=400)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, expand=False, padx=(0, 10))
        left_panel.pack_propagate(False)

        canvas = tk.Canvas(left_panel, bg=self.C_BG, highlightthickness=0, bd=0)
        scrollbar = ttk.Scrollbar(left_panel, orient="vertical", command=canvas.yview)

        scrollable_frame = ttk.Frame(canvas)
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        window_id = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.bind(
            "<Configure>",
            lambda e: canvas.itemconfigure(window_id, width=e.width),
        )
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Only scroll the card stack while the pointer is actually over it, so
        # the wheel can zoom the arena plot when hovering the map instead.
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        left_panel.bind("<Enter>", lambda _e: canvas.bind_all("<MouseWheel>", _on_mousewheel))
        left_panel.bind("<Leave>", lambda _e: canvas.unbind_all("<MouseWheel>"))

        # ---- Right panel: arena map with toolbar ----
        right_panel = ttk.Frame(mainframe, style="Card.TFrame", padding=8)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        map_header = ttk.Frame(right_panel, style="Card.TFrame")
        map_header.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))
        ttk.Label(
            map_header,
            text="ARENA MAP",
            style="TLabel",
            font=(self.FONT_FAMILY, 12, "bold"),
        ).pack(side=tk.LEFT)
        ttk.Label(
            map_header,
            text="left-click: set goal / paint obstacle   ·   scroll: zoom   ·   toolbar: pan",
            style="Dim.TLabel",
        ).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(
            map_header,
            text="Reset View",
            command=self._reset_view_clicked,
        ).pack(side=tk.RIGHT)

        self.fig = Figure(figsize=(10, 8), tight_layout=True, facecolor=self.C_PANEL)
        self.ax_traj = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event", self._on_plot_click)
        self.canvas.mpl_connect("scroll_event", self._on_scroll_zoom)
        self.canvas.mpl_connect("button_press_event", self._on_canvas_press)
        self.canvas.mpl_connect("button_release_event", self._on_canvas_release)

        toolbar_frame = ttk.Frame(right_panel, style="Card.TFrame")
        toolbar_frame.pack(side=tk.BOTTOM, fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        self._theme_toolbar(self.toolbar)

        # ---- Card: Fleet table ----
        robots_frame = ttk.LabelFrame(scrollable_frame, text="Fleet", padding=10)
        robots_frame.pack(fill=tk.X, expand=False, pady=(0, 10))

        columns = ("robot_id", "state", "battery", "x", "y", "theta")
        self.tree = ttk.Treeview(robots_frame, columns=columns, show="headings", height=6)

        headings = {
            "robot_id": "Robot", "state": "State", "battery": "Batt",
            "x": "X", "y": "Y", "theta": "θ",
        }
        for col in columns:
            self.tree.heading(col, text=headings[col])

        self.tree.column("robot_id", width=90, anchor="center")
        self.tree.column("state", width=92, anchor="center")
        self.tree.column("battery", width=58, anchor="center")
        self.tree.column("x", width=52, anchor="center")
        self.tree.column("y", width=52, anchor="center")
        self.tree.column("theta", width=56, anchor="center")

        # Per-state row coloring (configured here, applied in _refresh_table).
        for state, color in self.STATE_COLORS.items():
            self.tree.tag_configure(state, foreground=color)

        self.tree.pack(fill=tk.X, expand=False)

        # ---- Card: Manual Control ----
        controls_frame = ttk.LabelFrame(scrollable_frame, text="Manual Control", padding=10)
        controls_frame.pack(fill=tk.X, expand=False, pady=(0, 10))

        ttk.Label(controls_frame, text="Target Robot", style="Dim.TLabel").pack(fill=tk.X, pady=(0, 2))

        self.selected_robot_var = tk.StringVar(value="")
        self.robot_selector = ttk.Combobox(
            controls_frame,
            textvariable=self.selected_robot_var,
            state="readonly",
            values=[],
        )
        self.robot_selector.pack(fill=tk.X, pady=(0, 6))

        self.selected_robot_summary_var = tk.StringVar(
            value="Select a robot to send a test path."
        )
        ttk.Label(
            controls_frame,
            textvariable=self.selected_robot_summary_var,
            wraplength=320,
            justify=tk.LEFT,
            style="Dim.TLabel",
        ).pack(fill=tk.X, pady=(0, 8))

        quick_buttons = ttk.Frame(controls_frame, style="Card.TFrame")
        quick_buttons.pack(fill=tk.X, pady=(0, 6))
        quick_buttons.columnconfigure((0, 1), weight=1, uniform="quick")
        ttk.Button(quick_buttons, text="Pause", command=self._send_pause).grid(row=0, column=0, sticky="ew", padx=(0, 3), pady=2)
        ttk.Button(quick_buttons, text="Resume", command=self._send_resume).grid(row=0, column=1, sticky="ew", padx=(3, 0), pady=2)
        ttk.Button(quick_buttons, text="Stop", command=self._send_stop).grid(row=1, column=0, sticky="ew", padx=(0, 3), pady=2)
        ttk.Button(quick_buttons, text="Gripper", command=self._send_toggle_gripper).grid(row=1, column=1, sticky="ew", padx=(3, 0), pady=2)

        ttk.Button(controls_frame, text="Straight Test", command=self._send_straight_test_path).pack(fill=tk.X, pady=2)
        ttk.Button(
            controls_frame,
            text="Drive Forward Until Stop",
            command=self._send_continuous_drive,
            style="Accent.TButton",
        ).pack(fill=tk.X, pady=2)
        ttk.Button(controls_frame, text="180 Turn Test", command=self._send_turnaround_test_path).pack(fill=tk.X, pady=2)
        ttk.Button(controls_frame, text="L Test Path", command=self._send_test_path).pack(fill=tk.X, pady=2)

        # ---- Card: Grid Coordination ----
        coordination_frame = ttk.LabelFrame(scrollable_frame, text="Grid Coordination", padding=10)
        coordination_frame.pack(fill=tk.X, expand=False, pady=(0, 10))

        ttk.Label(
            coordination_frame,
            text="4m x 4m arena split into 40 x 40 cells at 10 cm resolution.",
            wraplength=320,
            justify=tk.LEFT,
            style="Dim.TLabel",
        ).pack(fill=tk.X, pady=(0, 8))

        ttk.Label(coordination_frame, text="Robot 1", style="Dim.TLabel").pack(fill=tk.X)

        self.grid_robot_one_var = tk.StringVar(value="")
        self.grid_robot_one_selector = ttk.Combobox(
            coordination_frame,
            textvariable=self.grid_robot_one_var,
            state="readonly",
            values=[],
        )
        self.grid_robot_one_selector.pack(fill=tk.X, pady=(0, 4))

        robot_one_goal = ttk.Frame(coordination_frame, style="Card.TFrame")
        robot_one_goal.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(robot_one_goal, text="Goal row").grid(row=0, column=0, sticky="w")
        self.grid_robot_one_row_var = tk.StringVar(value="10")
        ttk.Entry(robot_one_goal, textvariable=self.grid_robot_one_row_var, width=6).grid(row=0, column=1, padx=(6, 12))
        ttk.Label(robot_one_goal, text="Goal col").grid(row=0, column=2, sticky="w")
        self.grid_robot_one_col_var = tk.StringVar(value="10")
        ttk.Entry(robot_one_goal, textvariable=self.grid_robot_one_col_var, width=6).grid(row=0, column=3, padx=(6, 0))
        ttk.Label(coordination_frame, text="Robot 2", style="Dim.TLabel").pack(fill=tk.X)

        self.grid_robot_two_var = tk.StringVar(value="")
        self.grid_robot_two_selector = ttk.Combobox(
            coordination_frame,
            textvariable=self.grid_robot_two_var,
            state="readonly",
            values=[],
        )
        self.grid_robot_two_selector.pack(fill=tk.X, pady=(0, 4))

        robot_two_goal = ttk.Frame(coordination_frame, style="Card.TFrame")
        robot_two_goal.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(robot_two_goal, text="Goal row").grid(row=0, column=0, sticky="w")
        self.grid_robot_two_row_var = tk.StringVar(value="20")
        ttk.Entry(robot_two_goal, textvariable=self.grid_robot_two_row_var, width=6).grid(row=0, column=1, padx=(6, 12))
        ttk.Label(robot_two_goal, text="Goal col").grid(row=0, column=2, sticky="w")
        self.grid_robot_two_col_var = tk.StringVar(value="20")
        ttk.Entry(robot_two_goal, textvariable=self.grid_robot_two_col_var, width=6).grid(row=0, column=3, padx=(6, 0))

        self.grid_plan_summary_var = tk.StringVar(value="Select two robots and set destination cells (0-39).")
        ttk.Label(
            coordination_frame,
            textvariable=self.grid_plan_summary_var,
            wraplength=320,
            justify=tk.LEFT,
            style="Dim.TLabel",
        ).pack(fill=tk.X, pady=(0, 8))

        ttk.Button(
            coordination_frame,
            text="Start Two-Robot Traverse",
            command=self._send_two_robot_traverse,
        ).pack(fill=tk.X, pady=2)

        # ---- Card: Priority Task ----
        task_frame = ttk.LabelFrame(scrollable_frame, text="Priority Task", padding=10)
        task_frame.pack(fill=tk.X, expand=False, pady=(0, 10))

        ttk.Label(
            task_frame,
            text="Click the arena to choose a priority goal or paint known obstacle cells.",
            wraplength=320,
            justify=tk.LEFT,
            style="Dim.TLabel",
        ).pack(fill=tk.X, pady=(0, 8))

        self.map_click_mode_var = tk.StringVar(value="goal")
        mode_frame = ttk.Frame(task_frame, style="Card.TFrame")
        mode_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Radiobutton(mode_frame, text="Goal", value="goal", variable=self.map_click_mode_var).pack(side=tk.LEFT)
        ttk.Radiobutton(mode_frame, text="Known obstacle", value="obstacle", variable=self.map_click_mode_var).pack(side=tk.LEFT, padx=(10, 0))

        goal_frame = ttk.Frame(task_frame, style="Card.TFrame")
        goal_frame.pack(fill=tk.X, pady=(4, 8))
        goal_frame.columnconfigure((1, 3), weight=1)

        ttk.Label(goal_frame, text="Goal row").grid(row=0, column=0, sticky="w")
        self.priority_goal_row_var = tk.StringVar(value="5")
        ttk.Entry(goal_frame, textvariable=self.priority_goal_row_var, width=6).grid(row=0, column=1, sticky="ew", padx=(6, 12))
        ttk.Label(goal_frame, text="Goal col").grid(row=0, column=2, sticky="w")
        self.priority_goal_col_var = tk.StringVar(value="5")
        ttk.Entry(goal_frame, textvariable=self.priority_goal_col_var, width=6).grid(row=0, column=3, sticky="ew", padx=(6, 0))

        self.task_summary_var = tk.StringVar(value="Goal cell (5, 5) selected.")
        ttk.Label(
            task_frame,
            textvariable=self.task_summary_var,
            wraplength=320,
            justify=tk.LEFT,
            style="Dim.TLabel",
        ).pack(fill=tk.X, pady=(0, 4))

        self.map_status_var = tk.StringVar(value="Mode: Goal | known obstacles: 0")
        ttk.Label(
            task_frame,
            textvariable=self.map_status_var,
            wraplength=320,
            justify=tk.LEFT,
            style="Dim.TLabel",
        ).pack(fill=tk.X, pady=(0, 8))

        ttk.Button(
            task_frame,
            text="Dispatch Priority Task",
            command=self._send_priority_dispatch,
            style="Accent.TButton",
        ).pack(fill=tk.X, pady=(2, 4))

        ttk.Button(
            task_frame,
            text="Print Fleet Summary",
            command=self._send_fleet_query,
        ).pack(fill=tk.X, pady=2)

        ttk.Button(
            task_frame,
            text="Clear Known Obstacles",
            command=self._clear_permanent_obstacles,
        ).pack(fill=tk.X, pady=2)

        self.map_click_mode_var.trace_add("write", lambda *_: self._refresh_map_status())
        self._refresh_map_status()

    def _theme_toolbar(self, toolbar):
        """Recolor the matplotlib navigation toolbar to match the dark theme."""
        try:
            toolbar.config(background=self.C_PANEL)
        except tk.TclError:
            pass
        for child in toolbar.winfo_children():
            try:
                child.config(background=self.C_PANEL)
            except tk.TclError:
                pass
            if isinstance(child, tk.Label):
                try:
                    child.config(foreground=self.C_TEXT_DIM)
                except tk.TclError:
                    pass

    # ----------------------------
    # Arena view controls
    # ----------------------------
    def _reset_view_clicked(self):
        self._reset_view = True
        self._refresh_plot()

    def _on_canvas_press(self, event):
        self._mouse_down = True
        self._last_interact = time.monotonic()

    def _on_canvas_release(self, event):
        self._mouse_down = False
        self._last_interact = time.monotonic()

    def _user_is_interacting(self) -> bool:
        # True while a drag is in progress or just after a scroll, so the live
        # loop can skip the expensive axes rebuild and let pan/zoom stay fluid.
        if self._mouse_down:
            return True
        return (time.monotonic() - self._last_interact) < 0.35

    def _on_scroll_zoom(self, event):
        if event.inaxes != self.ax_traj or event.xdata is None or event.ydata is None:
            return
        # Finer step than the default so repeated scrolling feels gradual.
        base = 1.1
        scale = (1.0 / base) if event.button == "up" else base

        x0, x1 = self.ax_traj.get_xlim()
        y0, y1 = self.ax_traj.get_ylim()
        new_w = (x1 - x0) * scale
        new_h = (y1 - y0) * scale
        relx = (event.xdata - x0) / (x1 - x0) if (x1 - x0) else 0.5
        rely = (event.ydata - y0) / (y1 - y0) if (y1 - y0) else 0.5

        self.ax_traj.set_xlim(event.xdata - new_w * relx, event.xdata + new_w * (1 - relx))
        self.ax_traj.set_ylim(event.ydata - new_h * rely, event.ydata + new_h * (1 - rely))
        self._last_interact = time.monotonic()
        self.canvas.draw_idle()

    def _get_robot_color(self, robot_id: str) -> str:
        if robot_id not in self.robot_colors:
            self.robot_colors[robot_id] = next(self.color_cycle)
        return self.robot_colors[robot_id]

    def _draw_robot_safety_box(self, ax, x: float, y: float, theta_deg: float, label: str, color: str):
        """
        Draw a rotated 40cm x 40cm safety square centered on the robot.
        """
        half = self.SAFETY_BOX_HALF_CM
        theta_rad = math.radians(theta_deg)

        # Square corners in robot-local coordinates
        local_corners = [
            (-half, -half),
            ( half, -half),
            ( half,  half),
            (-half,  half),
        ]

        world_corners = []
        for lx, ly in local_corners:
            wx = x + lx * math.cos(theta_rad) - ly * math.sin(theta_rad)
            wy = y + lx * math.sin(theta_rad) + ly * math.cos(theta_rad)
            world_corners.append((wx, wy))

        patch = Polygon(
            world_corners,
            closed=True,
            fill=False,
            linewidth=1.5,
            linestyle="-",
            alpha=0.8,
            label=label,
            edgecolor=color,
        )
        ax.add_patch(patch)

    # ----------------------------
    # Queue processing
    # ----------------------------
    def _prune_echo_history(self, hist: dict, current_t_s: float):
        cutoff_t_s = current_t_s - self.ECHO_WINDOW_S

        while hist["front_echo_t"] and hist["front_echo_t"][0] < cutoff_t_s:
            hist["front_echo_t"].pop(0)
            hist["front_echo_x"].pop(0)
            hist["front_echo_y"].pop(0)

        while hist["left_echo_t"] and hist["left_echo_t"][0] < cutoff_t_s:
            hist["left_echo_t"].pop(0)
            hist["left_echo_x"].pop(0)
            hist["left_echo_y"].pop(0)

        while hist["right_echo_t"] and hist["right_echo_t"][0] < cutoff_t_s:
            hist["right_echo_t"].pop(0)
            hist["right_echo_x"].pop(0)
            hist["right_echo_y"].pop(0)

    def _remember_obstacle_cell(self, x_cm: float, y_cm: float, t_s: float | None):
        if not (0.0 <= x_cm <= self.ARENA_SIZE_CM and 0.0 <= y_cm <= self.ARENA_SIZE_CM):
            return
        row = int(y_cm // self.GRID_SPACING_CM)
        col = int(x_cm // self.GRID_SPACING_CM)
        self.obstacle_cells[(row, col)] = t_s if t_s is not None else time.time()

    def _prune_obstacle_cells(self, current_t_s: float):
        cutoff_t_s = current_t_s - self.ECHO_WINDOW_S
        expired = [
            cell for cell, seen_at in self.obstacle_cells.items()
            if seen_at < cutoff_t_s
        ]
        for cell in expired:
            self.obstacle_cells.pop(cell, None)

    def _process_queue(self):
        updated = False

        while not self.telemetry_queue.empty():
            robot_id, telemetry = self.telemetry_queue.get()

            self.robot_states[robot_id] = telemetry
            hist = self.robot_history[robot_id]

            t = telemetry.get("t_ms")
            x = telemetry.get("x_cm")
            y = telemetry.get("y_cm")
            theta = telemetry.get("theta_deg")
            front = telemetry.get("front_ultrasonic_cm")
            left = telemetry.get("left_ultrasonic_cm")
            right = telemetry.get("right_ultrasonic_cm")
            t_s = float(t) / 1000.0 if t is not None else None

            if t is not None:
                hist["t"].append(t_s)
            if x is not None:
                hist["x"].append(float(x))
            if y is not None:
                hist["y"].append(float(y))
            if theta is not None:
                hist["theta"].append(float(theta))
            if front is not None:
                hist["front_ultra"].append(float(front))
            if left is not None:
                hist["left_ultra"].append(float(left))
            if right is not None:
                hist["right_ultra"].append(float(right))

            if x is not None and y is not None and theta is not None:
                x = float(x)
                y = float(y)
                theta = float(theta)

                if front is not None:
                    pt = self._compute_echo_point(
                        x, y, theta, float(front),
                        self.FRONT_SENSOR_FORWARD_OFFSET_CM,
                        self.FRONT_SENSOR_LATERAL_OFFSET_CM,
                        self.FRONT_SENSOR_ANGLE_OFFSET_DEG,
                    )
                    if pt is not None:
                        ex, ey = pt
                        hist["front_echo_t"].append(t_s if t_s is not None else 0.0)
                        hist["front_echo_x"].append(ex)
                        hist["front_echo_y"].append(ey)
                        self._remember_obstacle_cell(ex, ey, t_s)

                if left is not None:
                    pt = self._compute_echo_point(
                        x, y, theta, float(left),
                        self.LEFT_SENSOR_FORWARD_OFFSET_CM,
                        self.LEFT_SENSOR_LATERAL_OFFSET_CM,
                        self.LEFT_SENSOR_ANGLE_OFFSET_DEG,
                    )
                    if pt is not None:
                        ex, ey = pt
                        hist["left_echo_t"].append(t_s if t_s is not None else 0.0)
                        hist["left_echo_x"].append(ex)
                        hist["left_echo_y"].append(ey)
                        self._remember_obstacle_cell(ex, ey, t_s)

                if right is not None:
                    pt = self._compute_echo_point(
                        x, y, theta, float(right),
                        self.RIGHT_SENSOR_FORWARD_OFFSET_CM,
                        self.RIGHT_SENSOR_LATERAL_OFFSET_CM,
                        self.RIGHT_SENSOR_ANGLE_OFFSET_DEG,
                    )
                    if pt is not None:
                        ex, ey = pt
                        hist["right_echo_t"].append(t_s if t_s is not None else 0.0)
                        hist["right_echo_x"].append(ex)
                        hist["right_echo_y"].append(ey)
                        self._remember_obstacle_cell(ex, ey, t_s)

            if t_s is not None:
                self._prune_echo_history(hist, t_s)
                self._prune_obstacle_cells(t_s)

            updated = True

        if updated:
            self._refresh_table()
            self._refresh_robot_selector()
            # Skip the heavy axes rebuild while the user is panning/zooming; it
            # would stutter their interaction. We catch up on the next tick.
            if not self._user_is_interacting():
                self._refresh_plot()

        self.root.after(100, self._process_queue)

    # ----------------------------
    # Math helpers
    # ----------------------------
    def _world_sensor_position(
        self,
        x: float,
        y: float,
        theta_deg: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
    ):
        theta_rad = math.radians(theta_deg)

        sensor_x = (
            x
            + forward_offset_cm * math.cos(theta_rad)
            - lateral_offset_cm * math.sin(theta_rad)
        )
        sensor_y = (
            y
            + forward_offset_cm * math.sin(theta_rad)
            + lateral_offset_cm * math.cos(theta_rad)
        )
        return sensor_x, sensor_y

    def _compute_echo_point(
        self,
        x: float,
        y: float,
        theta_deg: float,
        ultrasonic_cm: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
        sensor_angle_offset_deg: float,
    ):
        if not (self.MIN_VALID_ULTRASONIC_CM <= ultrasonic_cm <= self.MAX_VALID_ULTRASONIC_CM):
            return None

        sensor_x, sensor_y = self._world_sensor_position(
            x, y, theta_deg, forward_offset_cm, lateral_offset_cm
        )

        ray_angle_deg = theta_deg + sensor_angle_offset_deg
        ray_angle_rad = math.radians(ray_angle_deg)

        ex = sensor_x + ultrasonic_cm * math.cos(ray_angle_rad)
        ey = sensor_y + ultrasonic_cm * math.sin(ray_angle_rad)

        return ex, ey

    def _draw_latest_ray(
        self,
        ax,
        x: float,
        y: float,
        theta_deg: float,
        ultrasonic_cm: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
        sensor_angle_offset_deg: float,
        label: str,
        linestyle: str,
        color: str,
    ):
        if not (self.MIN_VALID_ULTRASONIC_CM <= ultrasonic_cm <= self.MAX_VALID_ULTRASONIC_CM):
            return

        sensor_x, sensor_y = self._world_sensor_position(
            x, y, theta_deg, forward_offset_cm, lateral_offset_cm
        )

        ray_angle_rad = math.radians(theta_deg + sensor_angle_offset_deg)
        ex = sensor_x + ultrasonic_cm * math.cos(ray_angle_rad)
        ey = sensor_y + ultrasonic_cm * math.sin(ray_angle_rad)

        ax.plot([sensor_x, ex], [sensor_y, ey], linestyle=linestyle, label=label, color=color)

    def _draw_sensor_marker(
        self,
        ax,
        x: float,
        y: float,
        theta_deg: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
        color: str,
    ):
        sensor_x, sensor_y = self._world_sensor_position(
            x, y, theta_deg, forward_offset_cm, lateral_offset_cm
        )
        ax.scatter(
            [sensor_x],
            [sensor_y],
            s=28,
            marker="s",
            facecolor="white",
            edgecolor=color,
            linewidth=1.4,
            zorder=6,
        )

    def _cell_center_cm(self, cell: tuple[int, int]) -> tuple[float, float]:
        row, col = cell
        return (
            col * self.GRID_SPACING_CM + self.GRID_SPACING_CM / 2.0,
            row * self.GRID_SPACING_CM + self.GRID_SPACING_CM / 2.0,
        )

    def _load_permanent_obstacles(self) -> set[tuple[int, int]]:
        try:
            raw_cells = json.loads(self.PERMANENT_OBSTACLES_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return set()

        cells = set()
        for item in raw_cells:
            try:
                row, col = item
            except (TypeError, ValueError):
                continue
            cells.add((
                max(0, min(self.GRID_DIM_CELLS - 1, int(row))),
                max(0, min(self.GRID_DIM_CELLS - 1, int(col))),
            ))
        return cells

    def _save_permanent_obstacles(self) -> None:
        raw_cells = sorted([list(cell) for cell in self.permanent_obstacle_cells])
        self.PERMANENT_OBSTACLES_PATH.write_text(json.dumps(raw_cells, indent=2), encoding="utf-8")

    def _set_priority_goal_cell(self, row: int, col: int, source: str = "manual") -> None:
        row = max(0, min(self.GRID_DIM_CELLS - 1, int(row)))
        col = max(0, min(self.GRID_DIM_CELLS - 1, int(col)))
        self.priority_goal_cell = (row, col)
        self.priority_goal_cm = self._cell_center_cm(self.priority_goal_cell)

        if hasattr(self, "priority_goal_row_var"):
            self.priority_goal_row_var.set(str(row))
        if hasattr(self, "priority_goal_col_var"):
            self.priority_goal_col_var.set(str(col))

        if hasattr(self, "task_summary_var"):
            if source == "click":
                self.task_summary_var.set(f"Map goal selected: cell ({row}, {col}).")
            else:
                self.task_summary_var.set(f"Goal cell ({row}, {col}) selected.")

    def _on_plot_click(self, event):
        # Ignore clicks while a navigation tool (pan/zoom) is active so dragging
        # the map doesn't drop goals/obstacles.
        if getattr(self, "toolbar", None) is not None and self.toolbar.mode:
            return
        if event.button != 1:
            return
        if event.inaxes != self.ax_traj or event.xdata is None or event.ydata is None:
            return
        if not (0.0 <= event.xdata <= self.ARENA_SIZE_CM and 0.0 <= event.ydata <= self.ARENA_SIZE_CM):
            return

        col = int(event.xdata // self.GRID_SPACING_CM)
        row = int(event.ydata // self.GRID_SPACING_CM)
        if self.map_click_mode_var.get() == "obstacle":
            self._toggle_permanent_obstacle(row, col)
        else:
            self._set_priority_goal_cell(row, col, source="click")
        self._refresh_plot()

    def _toggle_permanent_obstacle(self, row: int, col: int) -> None:
        cell = (
            max(0, min(self.GRID_DIM_CELLS - 1, int(row))),
            max(0, min(self.GRID_DIM_CELLS - 1, int(col))),
        )
        if cell in self.permanent_obstacle_cells:
            self.permanent_obstacle_cells.remove(cell)
        else:
            self.permanent_obstacle_cells.add(cell)
        self._save_permanent_obstacles()

        if self.command_sender:
            self.command_sender({
                "type": "permanent_obstacle_update",
                "action": "toggle",
                "row": cell[0],
                "col": cell[1],
            })
        self.task_summary_var.set(f"Known obstacle toggled at cell ({cell[0]}, {cell[1]}).")
        self._refresh_map_status()

    def _clear_permanent_obstacles(self) -> None:
        self.permanent_obstacle_cells.clear()
        self._save_permanent_obstacles()
        if self.command_sender:
            self.command_sender({"type": "permanent_obstacle_update", "action": "clear"})
        self.task_summary_var.set("Known obstacle map cleared.")
        self._refresh_map_status()
        self._refresh_plot()

    def _refresh_map_status(self) -> None:
        mode = self.map_click_mode_var.get().strip() or "goal"
        mode_label = "Known obstacle" if mode == "obstacle" else "Goal"
        self.map_status_var.set(
            f"Mode: {mode_label} | known obstacles: {len(self.permanent_obstacle_cells)}"
        )

    # ----------------------------
    # Refresh UI
    # ----------------------------
    def _refresh_table(self):
        self.tree.delete(*self.tree.get_children())

        for robot_id, state in self.robot_states.items():
            state_name = str(state.get("state", ""))
            tag = state_name if state_name in self.STATE_COLORS else ""
            self.tree.insert(
                "",
                "end",
                tags=(tag,) if tag else (),
                values=(
                    robot_id,
                    state_name,
                    f'{float(state.get("battery_percent", 0.0)):.1f}%',
                    round(float(state.get("x_cm", 0.0)), 2),
                    round(float(state.get("y_cm", 0.0)), 2),
                    round(float(state.get("theta_deg", 0.0)), 2),
                ),
            )

    def _refresh_plot(self):
        # Preserve the user's pan/zoom across live redraws; only snap back to
        # the full 0-400 cm arena on first draw or an explicit Reset View.
        preserve = self._plot_initialized and not self._reset_view
        if preserve:
            cur_xlim = self.ax_traj.get_xlim()
            cur_ylim = self.ax_traj.get_ylim()
        self._reset_view = False
        self._plot_initialized = True

        self.ax_traj.clear()

        self.fig.set_facecolor(self.C_PANEL)
        self.ax_traj.set_facecolor(self.C_PLOT_BG)
        self.ax_traj.set_title(
            "Arena Map  ·  click to set priority goal",
            fontsize=12, fontweight="bold", color=self.C_TEXT, pad=10,
        )
        self.ax_traj.set_xlabel("x (cm)", color=self.C_TEXT_DIM)
        self.ax_traj.set_ylabel("y (cm)", color=self.C_TEXT_DIM)

        # Keep physical scale equal
        self.ax_traj.set_aspect("equal", adjustable="box")

        # Major ticks every 50 cm (labels)
        self.ax_traj.xaxis.set_major_locator(MultipleLocator(50))
        self.ax_traj.yaxis.set_major_locator(MultipleLocator(50))

        # Minor ticks every 10 cm (grid squares)
        self.ax_traj.xaxis.set_minor_locator(MultipleLocator(10))
        self.ax_traj.yaxis.set_minor_locator(MultipleLocator(10))

        # Draw grids
        self.ax_traj.grid(which="major", linewidth=0.7, color=self.C_GRID_MAJ)
        self.ax_traj.grid(which="minor", linewidth=0.3, color=self.C_GRID_MIN)
        self.ax_traj.tick_params(colors=self.C_TEXT_DIM, labelsize=8)
        for spine in self.ax_traj.spines.values():
            spine.set_edgecolor(self.C_BORDER)

        # Arena boundary outline (0-400 cm)
        self.ax_traj.add_patch(
            Rectangle(
                (0, 0), self.ARENA_SIZE_CM, self.ARENA_SIZE_CM,
                fill=False, edgecolor=self.C_RED, linewidth=1.4, alpha=0.55,
            )
        )

        for row, col in self.obstacle_cells:
            self.ax_traj.add_patch(
                Rectangle(
                    (col * self.GRID_SPACING_CM, row * self.GRID_SPACING_CM),
                    self.GRID_SPACING_CM,
                    self.GRID_SPACING_CM,
                    facecolor="#FF7A00",
                    edgecolor="none",
                    alpha=0.28,
                    label="remembered obstacle",
                )
            )

        for row, col in self.permanent_obstacle_cells:
            self.ax_traj.add_patch(
                Rectangle(
                    (col * self.GRID_SPACING_CM, row * self.GRID_SPACING_CM),
                    self.GRID_SPACING_CM,
                    self.GRID_SPACING_CM,
                    facecolor="#5b6270",
                    edgecolor="#838b99",
                    linewidth=0.8,
                    alpha=0.85,
                    label="known obstacle",
                )
            )

        goal_x, goal_y = self.priority_goal_cm
        goal_row, goal_col = self.priority_goal_cell
        self.ax_traj.add_patch(
            Rectangle(
                (goal_col * self.GRID_SPACING_CM, goal_row * self.GRID_SPACING_CM),
                self.GRID_SPACING_CM,
                self.GRID_SPACING_CM,
                facecolor=self.C_RED,
                edgecolor=self.C_RED,
                linewidth=1.5,
                alpha=0.30,
                label="priority goal",
            )
        )
        self.ax_traj.scatter(
            [goal_x],
            [goal_y],
            marker="*",
            s=220,
            color=self.C_RED,
            edgecolor="#ffffff",
            linewidth=0.8,
            label=f"goal ({goal_row}, {goal_col})",
            zorder=7,
        )

        for robot_id, hist in self.robot_history.items():
            xs = hist["x"]
            ys = hist["y"]
            color = self._get_robot_color(robot_id)

            if xs and ys:
                self.ax_traj.plot(xs, ys, marker="o", linestyle="-", label=f"{robot_id} path", color=color)

                theta_deg = hist["theta"][-1]
                theta_rad = math.radians(theta_deg)
                arrow_len = 8.0
                dx = arrow_len * math.cos(theta_rad)
                dy = arrow_len * math.sin(theta_rad)

                self.ax_traj.arrow(
                    xs[-1],
                    ys[-1],
                    dx,
                    dy,
                    head_width=3.0,
                    head_length=4.0,
                    length_includes_head=True,
                    color=color,
                    zorder=6,
                )

                # Robot center point
                self.ax_traj.scatter(
                    [xs[-1]],
                    [ys[-1]],
                    s=50,
                    label=f"{robot_id} center",
                    zorder=5,
                    color=color,
                )

                # Rotated safety box
                self._draw_robot_safety_box(
                    self.ax_traj,
                    xs[-1],
                    ys[-1],
                    theta_deg,
                    label=f"{robot_id} safety box",
                    color=color,
                )

                # Mark the two real forward ultrasonics at their mounted
                # corners and draw their latest rays. There is no physical
                # center sensor, so nothing is drawn at the robot midline.
                self._draw_sensor_marker(
                    self.ax_traj, xs[-1], ys[-1], theta_deg,
                    self.LEFT_SENSOR_FORWARD_OFFSET_CM,
                    self.LEFT_SENSOR_LATERAL_OFFSET_CM,
                    color,
                )
                self._draw_sensor_marker(
                    self.ax_traj, xs[-1], ys[-1], theta_deg,
                    self.RIGHT_SENSOR_FORWARD_OFFSET_CM,
                    self.RIGHT_SENSOR_LATERAL_OFFSET_CM,
                    color,
                )

                if hist["left_ultra"]:
                    self._draw_latest_ray(
                        self.ax_traj, xs[-1], ys[-1], theta_deg,
                        hist["left_ultra"][-1],
                        self.LEFT_SENSOR_FORWARD_OFFSET_CM,
                        self.LEFT_SENSOR_LATERAL_OFFSET_CM,
                        self.LEFT_SENSOR_ANGLE_OFFSET_DEG,
                        label=None, linestyle=":", color=color,
                    )
                if hist["right_ultra"]:
                    self._draw_latest_ray(
                        self.ax_traj, xs[-1], ys[-1], theta_deg,
                        hist["right_ultra"][-1],
                        self.RIGHT_SENSOR_FORWARD_OFFSET_CM,
                        self.RIGHT_SENSOR_LATERAL_OFFSET_CM,
                        self.RIGHT_SENSOR_ANGLE_OFFSET_DEG,
                        label=None, linestyle="--", color=color,
                    )

            hit_xs = hist["left_echo_x"] + hist["right_echo_x"]
            hit_ys = hist["left_echo_y"] + hist["right_echo_y"]
            if hit_xs and hit_ys:
                self.ax_traj.scatter(
                    hit_xs,
                    hit_ys,
                    s=18,
                    alpha=0.7,
                    label=f"{robot_id} sensor hits",
                    color=color,
                )

        handles, labels = self.ax_traj.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        if unique:
            legend = self.ax_traj.legend(
                unique.values(),
                unique.keys(),
                loc="upper left",
                fontsize=7.5,
                framealpha=0.92,
                edgecolor=self.C_BORDER,
                labelcolor=self.C_TEXT,
                ncol=2 if len(unique) > 8 else 1,
            )
            legend.get_frame().set_facecolor(self.C_PANEL_2)

        # Restore preserved view or snap to the full arena.
        if preserve:
            self.ax_traj.set_xlim(cur_xlim)
            self.ax_traj.set_ylim(cur_ylim)
        else:
            self.ax_traj.set_xlim(0, self.ARENA_SIZE_CM)
            self.ax_traj.set_ylim(0, self.ARENA_SIZE_CM)

        self.canvas.draw_idle()
    
    # ----------------------------
    # Control Board
    # ----------------------------
    def _get_selected_robot_id(self) -> str | None:
        robot_id = self.selected_robot_var.get().strip()
        if not robot_id:
            return None
        return robot_id

    def _get_selected_robot_state(self) -> dict | None:
        robot_id = self._get_selected_robot_id()
        if robot_id is None:
            return None
        return self.robot_states.get(robot_id)

    def _get_test_distance_cm(self) -> float:
        return self.DEFAULT_TEST_DISTANCE_CM

    def _get_motion_settings(self) -> MotionSettings:
        return MotionSettings(
            turn_speed_deg_per_sec=self.DEFAULT_TURN_SPEED,
            drive_speed_deg_per_sec=self.DEFAULT_DRIVE_SPEED,
        )

    def _next_test_path_id(self) -> int:
        self.test_path_counter += 1
        return self.test_path_counter

    def _send_pause(self):
        robot_id = self._get_selected_robot_id()
        if robot_id and self.command_sender:
            msg = PauseMessage(
                robot_id=robot_id,
                reason="gui_pause_button",
            )
            self.command_sender(msg)

    def _send_resume(self):
        robot_id = self._get_selected_robot_id()
        if robot_id and self.command_sender:
            msg = ResumeMessage(robot_id=robot_id)
            self.command_sender(msg)

    def _send_stop(self):
        robot_id = self._get_selected_robot_id()
        if robot_id and self.command_sender:
            msg = StopMessage(
                robot_id=robot_id,
                reason="gui_stop_button",
            )
            self.command_sender(msg)

    def _send_toggle_gripper(self):
        robot_id = self._get_selected_robot_id()
        if robot_id and self.command_sender:
            msg = ToggleGripperMessage(robot_id=robot_id)
            self.command_sender(msg)

    def _send_straight_test_path(self):
        robot_id = self._get_selected_robot_id()
        robot_state = self._get_selected_robot_state()

        if not robot_id or not self.command_sender or not robot_state:
            return

        x = float(robot_state.get("x_cm", 0.0))
        y = float(robot_state.get("y_cm", 0.0))
        theta_deg = float(robot_state.get("theta_deg", 0.0))
        theta_rad = math.radians(theta_deg)
        distance_cm = self._get_test_distance_cm()

        wp_x = min(max(x + distance_cm * math.cos(theta_rad), 0.0), 400.0)
        wp_y = min(max(y + distance_cm * math.sin(theta_rad), 0.0), 400.0)

        msg = PathAssignmentMessage(
            robot_id=robot_id,
            path_id=self._next_test_path_id(),
            replace_existing=True,
            waypoints=[Waypoint(x_cm=wp_x, y_cm=wp_y)],
            motion=self._get_motion_settings(),
        )

        self.command_sender(msg)

    def _send_continuous_drive(self):
        robot_id = self._get_selected_robot_id()
        if not robot_id or not self.command_sender:
            return
        self.command_sender({
            "type": "continuous_drive",
            "robot_id": robot_id,
            "motor_power": 35,
        })

    def _send_turnaround_test_path(self):
        robot_id = self._get_selected_robot_id()
        robot_state = self._get_selected_robot_state()

        if not robot_id or not self.command_sender or not robot_state:
            return

        x = float(robot_state.get("x_cm", 0.0))
        y = float(robot_state.get("y_cm", 0.0))
        theta_deg = float(robot_state.get("theta_deg", 0.0))
        theta_rad = math.radians(theta_deg)
        distance_cm = self._get_test_distance_cm()

        wp_x = min(max(x - distance_cm * math.cos(theta_rad), 0.0), 400.0)
        wp_y = min(max(y - distance_cm * math.sin(theta_rad), 0.0), 400.0)

        msg = PathAssignmentMessage(
            robot_id=robot_id,
            path_id=self._next_test_path_id(),
            replace_existing=True,
            waypoints=[Waypoint(x_cm=wp_x, y_cm=wp_y)],
            motion=self._get_motion_settings(),
        )

        self.command_sender(msg)

    def _send_test_path(self):
        robot_id = self._get_selected_robot_id()
        robot_state = self._get_selected_robot_state()

        if not robot_id or not self.command_sender or not robot_state:
            return

        # Start from the robot's current believed pose and send a simple
        # three-waypoint path in the global arena frame.
        x = float(robot_state.get("x_cm", 0.0))
        y = float(robot_state.get("y_cm", 0.0))

        # Simple "Γ"-shaped test path, clipped to the 0..400 cm arena
        wp1_x = min(max(x + 20.0, 0.0), 400.0)
        wp1_y = min(max(y,         0.0), 400.0)

        wp2_x = min(max(x + 40.0, 0.0), 400.0)
        wp2_y = min(max(y,         0.0), 400.0)

        wp3_x = min(max(x + 40.0, 0.0), 400.0)
        wp3_y = min(max(y + 40.0, 0.0), 400.0)

        msg = PathAssignmentMessage(
            robot_id=robot_id,
            path_id=self._next_test_path_id(),
            replace_existing=True,
            waypoints=[
                Waypoint(x_cm=wp1_x, y_cm=wp1_y),
                Waypoint(x_cm=wp2_x, y_cm=wp2_y),
                Waypoint(x_cm=wp3_x, y_cm=wp3_y),
            ],
            motion=self._get_motion_settings(),
        )

        self.command_sender(msg)

    def _parse_goal_cell(self, row_var: tk.StringVar, col_var: tk.StringVar) -> tuple[int, int] | None:
        try:
            row = int(row_var.get())
            col = int(col_var.get())
        except (TypeError, ValueError):
            return None

        if not (0 <= row < self.GRID_DIM_CELLS and 0 <= col < self.GRID_DIM_CELLS):
            return None

        return row, col

    def _send_two_robot_traverse(self):
        if not self.command_sender:
            return

        robot_one = self.grid_robot_one_var.get().strip()
        robot_two = self.grid_robot_two_var.get().strip()
        if not robot_one or not robot_two or robot_one == robot_two:
            self.grid_plan_summary_var.set("Pick two different robots before starting a coordinated traverse.")
            return

        goal_one = self._parse_goal_cell(self.grid_robot_one_row_var, self.grid_robot_one_col_var)
        goal_two = self._parse_goal_cell(self.grid_robot_two_row_var, self.grid_robot_two_col_var)
        if goal_one is None or goal_two is None:
            self.grid_plan_summary_var.set("Goal cells must be integers from 0 to 39.")
            return

        if goal_one == goal_two:
            self.grid_plan_summary_var.set("Choose different goal cells for the two robots.")
            return

        self.grid_plan_summary_var.set(
            f"Planning coordinated traverse: {robot_one} -> ({goal_one[0]}, {goal_one[1]}), "
            f"{robot_two} -> ({goal_two[0]}, {goal_two[1]})."
        )

        self.command_sender({
            "type": "coordinated_traverse",
            "robots": [
                {"robot_id": robot_one, "goal_row": goal_one[0], "goal_col": goal_one[1]},
                {"robot_id": robot_two, "goal_row": goal_two[0], "goal_col": goal_two[1]},
            ],
        })

    def _send_priority_dispatch(self):
        if not self.command_sender:
            return

        goal = self._parse_goal_cell(self.priority_goal_row_var, self.priority_goal_col_var)
        if goal is None:
            self.task_summary_var.set("Goal cells must be integers from 0 to 39.")
            return

        self._set_priority_goal_cell(goal[0], goal[1])
        self._refresh_plot()
        self.task_summary_var.set(f"Dispatching priority task to cell ({goal[0]}, {goal[1]}).")
        response = self.command_sender({
            "type": "priority_dispatch",
            "goal_row": goal[0],
            "goal_col": goal[1],
            "priority": 1,
        })
        if isinstance(response, dict):
            ok = bool(response.get("ok"))
            reason = str(response.get("reason", ""))
        else:
            ok = bool(response)
            reason = ""
        if not ok:
            self.task_summary_var.set(f"Priority dispatch failed: {reason or 'unknown_error'}.")
        else:
            self.task_summary_var.set(f"Priority task sent to cell ({goal[0]}, {goal[1]}).")
            if reason and reason != "dispatched":
                self.task_summary_var.set(
                    f"Priority task sent to cell ({goal[0]}, {goal[1]}): {reason}."
                )

    def _send_fleet_query(self):
        if self.command_sender:
            self.command_sender({"type": "fleet_query"})
    

    def _refresh_robot_selector(self):
        robot_ids = sorted(self.robot_states.keys())
        self.robot_selector["values"] = robot_ids
        self.grid_robot_one_selector["values"] = robot_ids
        self.grid_robot_two_selector["values"] = robot_ids

        current = self.selected_robot_var.get()

        # If current selection disappeared, clear it
        if current and current not in robot_ids:
            self.selected_robot_var.set("")

        # If nothing is selected and robots exist, pick the first one
        if not self.selected_robot_var.get() and robot_ids:
            self.selected_robot_var.set(robot_ids[0])

        if not self.grid_robot_one_var.get() and robot_ids:
            self.grid_robot_one_var.set(robot_ids[0])

        if not self.grid_robot_two_var.get() and len(robot_ids) > 1:
            self.grid_robot_two_var.set(robot_ids[1])
        elif not self.grid_robot_two_var.get() and robot_ids:
            self.grid_robot_two_var.set(robot_ids[0])

        selected_state = self._get_selected_robot_state()
        if selected_state is None:
            self.selected_robot_summary_var.set("Select a robot to send a test path.")
            return

        robot_id = self._get_selected_robot_id()
        state = selected_state.get("state", "unknown")
        path_id = selected_state.get("path_id", "-")
        waypoint_index = selected_state.get("waypoint_index", "-")
        x = float(selected_state.get("x_cm", 0.0))
        y = float(selected_state.get("y_cm", 0.0))
        theta = float(selected_state.get("theta_deg", 0.0))

        self.selected_robot_summary_var.set(
            f"{robot_id}: state={state}, path={path_id}, waypoint={waypoint_index}, "
            f"pose=({x:.1f}, {y:.1f}, {theta:.1f} deg)"
        )
