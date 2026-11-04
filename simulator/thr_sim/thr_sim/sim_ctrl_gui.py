"""Tk + matplotlib UI for the sim control node.

Runs in the main thread; the ROS node spins in a background thread. The UI
only reads node data under `node.lock` and sends commands via `node.request`.
"""
import logging
import math
import tkinter as tk
import traceback
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

import numpy as np
import rclpy
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.patches import Polygon

from thr_sim.path_utils import path_yaws
from thr_sim.sim_ctrl_node import PATH_SHAPES, IDLE, SETTLING, RUNNING, DONE

# Vehicle drawing dimensions [m] (reference point is the rear axle).
WHEELBASE = 2.5
REAR_OVERHANG = 0.8
FRONT_OVERHANG = 0.8
WIDTH = 1.8
WHEEL_LEN = 0.7

STATE_COLORS = {IDLE: '#777777', SETTLING: '#c98a00', RUNNING: '#1f8a3b', DONE: '#1f5fbf'}
RESULT_COLORS = {'stopped': '#1f8a3b', 'off_track': '#c0392b', 'overrun': '#c0392b',
                 'timeout': '#c98a00', 'aborted': '#777777'}
V_COLOR = '#2c7fb8'
STEER_COLOR = '#d95f02'
UPDATE_MS = 50

# The map keeps an equal aspect by stretching one axis past the limits we set;
# matplotlib logs "Ignoring fixed x/y limits..." every time. That is intended here.
logging.getLogger('matplotlib.axes._base').setLevel(logging.ERROR)
FOLLOW_SPAN = 15.0  # half-width of the view when following the vehicle [m]
TIME_WINDOW = 10.0  # the CTE / control plots show the last TIME_WINDOW seconds


class SimCtrlGui:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('THR Sim Control')
        self.root.geometry('1500x920')
        self.root.minsize(1180, 780)
        self.root.configure(background='#eef2f7')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.report_callback_exception = self._on_callback_error
        self._path_drawn = None
        self._n_results = -1
        self._closing = False

        style = ttk.Style(self.root)
        if 'clam' in style.theme_names():
            style.theme_use('clam')
        self._configure_style(style)
        header = ttk.Frame(self.root, style='Header.TFrame', padding=(20, 12))
        header.pack(side=tk.TOP, fill=tk.X)
        title = 'Evaluation studio' if node.eval_mode else 'Vehicle simulator'
        ttk.Label(header, text=title, style='Title.TLabel').pack(side=tk.LEFT)
        ttk.Label(header, text='THR  /  BICYCLE MODEL', style='Header.TLabel').pack(side=tk.RIGHT)
        self.status_text = tk.StringVar(value='Waiting for telemetry')
        ttk.Label(self.root, textvariable=self.status_text, style='Footer.TLabel',
                  padding=(16, 6)).pack(side=tk.BOTTOM, fill=tk.X)

        self._build_plot()
        self._build_panel()
        self._bind_keys()
        self._after_id = self.root.after(UPDATE_MS, self.update)

    def _configure_style(self, style):
        style.configure('.', font=('DejaVu Sans', 10), background='#eef2f7',
                        foreground='#243247')
        style.configure('TFrame', background='#eef2f7')
        style.configure('TLabel', background='#eef2f7')
        style.configure('TLabelframe', background='#eef2f7', borderwidth=1,
                        relief='solid')
        style.configure('TLabelframe.Label', foreground='#52657e',
                        font=('DejaVu Sans', 10, 'bold'))
        style.configure('TButton', padding=(10, 8), background='#e0e7f0', borderwidth=0)
        style.map('TButton', background=[('active', '#cdd8e8')])
        style.configure('Primary.TButton', background='#2563eb', foreground='white')
        style.map('Primary.TButton', background=[('active', '#1d4ed8')])
        style.configure('Big.TLabel', font=('DejaVu Sans', 14, 'bold'))
        style.configure('Header.TFrame', background='#14243a')
        style.configure('Title.TLabel', background='#14243a', foreground='white',
                        font=('DejaVu Sans', 19, 'bold'))
        style.configure('Header.TLabel', background='#14243a', foreground='#acbed6',
                        font=('DejaVu Sans', 9, 'bold'))
        style.configure('Footer.TLabel', background='#e0e7f0', foreground='#52657e')
        style.configure('Treeview', background='white', fieldbackground='white',
                        rowheight=32, borderwidth=0, font=('DejaVu Sans', 9))
        style.configure('Treeview.Heading', background='#dce5ef',
                        foreground='#34465f', padding=(6, 9),
                        font=('DejaVu Sans', 9, 'bold'))
        style.map('Treeview', background=[('selected', '#dbeafe')],
                  foreground=[('selected', '#163b75')])
        style.configure('TNotebook', background='#eef2f7', borderwidth=0)
        style.configure('TNotebook.Tab', padding=(14, 9))
        style.configure('Horizontal.TProgressbar', background='#2563eb',
                        troughcolor='#dce5ef', borderwidth=0)

    def _scroll_table(self, table):
        """Tables keep readable minimum column widths on smaller windows."""
        parent = table.master
        table.grid(row=0, column=0, sticky='nsew')
        sy = ttk.Scrollbar(parent, orient='vertical', command=table.yview)
        sx = ttk.Scrollbar(parent, orient='horizontal', command=table.xview)
        sy.grid(row=0, column=1, sticky='ns')
        sx.grid(row=1, column=0, sticky='ew')
        table.configure(yscrollcommand=sy.set, xscrollcommand=sx.set)
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)

    # ------------------------------------------------------------- layout
    def _build_plot(self):
        left = ttk.Frame(self.root, padding=(12, 12, 4, 6))
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(8, 7), dpi=100, constrained_layout=True)
        gs = self.fig.add_gridspec(5, 1)
        self.ax = self.fig.add_subplot(gs[:3, 0])
        self.ax_cte = self.fig.add_subplot(gs[3, 0])
        self.ax_v = self.fig.add_subplot(gs[4, 0], sharex=self.ax_cte)
        self.ax_steer = self.ax_v.twinx()

        ax = self.ax
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        self.corridor = None
        (self.path_line,) = ax.plot([], [], '--', color='#555555', lw=1.2, label='path')
        (self.runout_line,) = ax.plot([], [], ':', color='#999999', lw=1.5,
                                      label='after finish')
        (self.path_nodes,) = ax.plot([], [], '.', color='#999999', ms=2.5)
        (self.start_mark,) = ax.plot([], [], 'o', color='#1f8a3b', ms=8, label='start')
        (self.finish_line,) = ax.plot([], [], '-', color='#c0392b', lw=3, label='finish (stop)')
        (self.trail_line,) = ax.plot([], [], '-', color='#1f77b4', lw=2.0, label='trail')
        self.body = Polygon(np.zeros((4, 2)), closed=True, fc='#ff8c1a', ec='#7a3d00',
                            lw=1.2, zorder=5, label='vehicle')
        ax.add_patch(self.body)
        (self.front_wheel,) = ax.plot([], [], '-', color='black', lw=3, zorder=6)
        (self.rear_wheel,) = ax.plot([], [], '-', color='black', lw=3, zorder=6)
        ax.legend(loc='upper left', fontsize=8, ncol=3, framealpha=0.9)

        ac = self.ax_cte
        ac.grid(True, alpha=0.3)
        ac.tick_params(labelbottom=False)
        ac.set_ylabel('CTE [m]')
        (self.cte_line,) = ac.plot([], [], color='#1f77b4')
        lim = self.node.max_cte
        for y in (lim, -lim):
            ac.axhline(y, color='#c0392b', ls='--', lw=1)
        ac.axhline(0, color='#555555', lw=0.8)
        ac.set_ylim(-1.2 * lim, 1.2 * lim)
        # Fixed x-axis (seconds before the latest sample): the data slides, the
        # axis never changes, so no full redraw is needed while a run goes on.
        ac.set_xlim(-TIME_WINDOW, 0.0)

        # control signals: velocity (left axis) and steering angle (right axis)
        av, ast = self.ax_v, self.ax_steer
        av.grid(True, alpha=0.3)
        av.set_xlabel(f'time relative to now [s]  (last {TIME_WINDOW:.0f} s)')
        av.set_ylabel('velocity [m/s]', color=V_COLOR)
        av.tick_params(axis='y', colors=V_COLOR)
        ast.set_ylabel('steer [deg]', color=STEER_COLOR)
        ast.tick_params(axis='y', colors=STEER_COLOR)
        ast.axhline(0, color=STEER_COLOR, lw=0.6, alpha=0.5)
        (self.v_line,) = av.plot([], [], color=V_COLOR, lw=1.5, label='velocity cmd')
        (self.v_act_line,) = av.plot([], [], '--', color=V_COLOR, lw=1.5,
                                     label='velocity actual')
        (self.steer_line,) = ast.plot([], [], color=STEER_COLOR, lw=1.5, label='steer cmd')
        (self.steer_act_line,) = ast.plot([], [], '--', color=STEER_COLOR, lw=1.5,
                                          label='steer actual')
        av.legend(handles=[self.v_line, self.v_act_line, self.steer_line, self.steer_act_line],
                  loc='upper right', fontsize=7, ncol=2)
        av.set_ylim(0, 32)  # avoid a redraw when cruise commands first arrive
        # wide enough for the full steering range (0.6 rad ~ 34 deg), so the axis
        # never has to grow (each growth is a slow full redraw)
        ast.set_ylim(-40, 40)

        self.fig.set_facecolor('#eef2f7')
        for axis in (ax, ac, av, ast):
            axis.set_facecolor('#ffffff')
            axis.tick_params(labelsize=8, colors='#52657e')
            axis.xaxis.label.set_color('#52657e')
            for spine in axis.spines.values():
                spine.set_color('#dce5ef')
        ax.set_title('TRACK MAP', loc='left', fontsize=10, color='#34465f', pad=12)
        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        toolbar = NavigationToolbar2Tk(self.canvas, left, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Blitting: everything that moves is "animated" and drawn on top of a
        # cached background. A full redraw only happens when axis limits change
        # (or on zoom/pan/resize, which triggers draw_event and a new cache).
        self.animated = [self.trail_line, self.body, self.front_wheel, self.rear_wheel,
                         self.cte_line, self.v_line, self.steer_line,
                         self.v_act_line, self.steer_act_line]
        for a in self.animated:
            a.set_animated(True)
        self._bg = None
        self._need_full = True
        self.canvas.mpl_connect('draw_event', self._on_draw)

        # savefig skips animated artists; include them for the toolbar's Save button.
        orig_savefig = self.fig.savefig

        def savefig(*args, **kwargs):
            for a in self.animated:
                a.set_animated(False)
            try:
                return orig_savefig(*args, **kwargs)
            finally:
                for a in self.animated:
                    a.set_animated(True)
                self._need_full = True

        self.fig.savefig = savefig

    def _on_draw(self, _event):
        self._bg = self.canvas.copy_from_bbox(self.fig.bbox)
        self._draw_animated()

    def _draw_animated(self):
        for a in self.animated:
            a.axes.draw_artist(a)

    def _set_lim(self, ax, axis, lo, hi):
        """Set limits only if they changed; flags a full redraw when they do."""
        get, set_ = (ax.get_xlim, ax.set_xlim) if axis == 'x' else (ax.get_ylim, ax.set_ylim)
        if get() != (lo, hi):
            set_(lo, hi)
            self._need_full = True

    def _build_panel(self):
        panel = ttk.Frame(self.root, padding=(10, 12, 12, 6), width=650)
        panel.pack(side=tk.RIGHT, fill=tk.Y)
        panel.pack_propagate(False)

        if self.node.eval_mode:
            self._build_eval_controls(panel)
        else:
            self._build_sim_controls(panel)

        # --- view options
        f = ttk.LabelFrame(panel, text='View', padding=8)
        f.pack(fill=tk.X, pady=6)
        self.follow_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text='Follow vehicle', variable=self.follow_var).pack(side=tk.LEFT)
        ttk.Button(f, text='Fit path', command=self.fit_view).pack(side=tk.RIGHT)

        f = ttk.Frame(panel, padding=(4, 6))
        f.pack(fill=tk.X)
        self.conn_labels = {}
        for key, title in (('vehicle', 'Vehicle'), ('driver', 'Driver')):
            label = tk.Label(f, text=f'● {title}: offline', fg='#9b4444',
                             bg='#eef2f7', font=('DejaVu Sans', 9, 'bold'))
            label.pack(side=tk.LEFT, padx=(0, 18))
            self.conn_labels[key] = label

        tabs = ttk.Notebook(panel)
        tabs.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        live_tab, results_tab = ttk.Frame(tabs, padding=8), ttk.Frame(tabs, padding=8)
        tabs.add(live_tab, text='Live telemetry')
        tabs.add(results_tab, text='Run history')

        # --- live status
        f = ttk.LabelFrame(live_tab, text='Vehicle telemetry', padding=12)
        f.pack(fill=tk.X, pady=6)
        self.state_lbl = ttk.Label(f, text=IDLE, style='Big.TLabel')
        self.state_lbl.grid(row=0, column=0, columnspan=2, sticky='w')
        self.progress = ttk.Progressbar(f, maximum=100.0)
        self.progress.grid(row=1, column=0, columnspan=2, sticky='we', pady=4)
        self.live = {}
        rows = ('Run', 'Time', 'Progress', 'To finish', 'CTE', 'RMS CTE', 'Pose', 'Command',
                'Actual')
        for i, name in enumerate(rows, start=2):
            ttk.Label(f, text=name + ':').grid(row=i, column=0, sticky='w')
            lbl = ttk.Label(f, text='–', font=('TkFixedFont', 10))
            lbl.grid(row=i, column=1, sticky='w', padx=6)
            self.live[name] = lbl
        f.columnconfigure(1, weight=1)

        # --- results
        f = ttk.LabelFrame(results_tab, text='Individual runs', padding=8)
        f.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        cols = (('run', '#', 32), ('shape', 'Path', 88), ('result', 'Result', 84),
                ('track', 'Track', 72), ('time', 'Time', 66), ('stop', 'Stop', 62),
                ('score', 'Score', 80), ('err', 'Stop err m', 76))
        table_frame = ttk.Frame(f)
        table_frame.pack(fill=tk.BOTH, expand=True)
        self.table = ttk.Treeview(table_frame, columns=[c[0] for c in cols], show='headings', height=8)
        for key, text, cw in cols:
            self.table.heading(key, text=text)
            self.table.column(key, width=cw, minwidth=cw, anchor='center', stretch=key == 'result')
        for res, color in RESULT_COLORS.items():
            self.table.tag_configure(res, foreground=color)
        self._scroll_table(self.table)
        self.avg_lbl = ttk.Label(f, text='Average score: –', style='Big.TLabel')
        self.avg_lbl.pack(anchor='w', pady=(6, 0))

    def _build_sim_controls(self, panel):
        # --- simulation controls
        f = ttk.LabelFrame(panel, text='Simulation', padding=8)
        f.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(f, text='Path').grid(row=0, column=0, sticky='w')
        self.shape_var = tk.StringVar(value=self.node.shape)
        shape = ttk.Combobox(f, textvariable=self.shape_var, values=PATH_SHAPES,
                             state='readonly', width=14)
        shape.grid(row=0, column=1, sticky='we', padx=4)
        shape.bind('<<ComboboxSelected>>',
                   lambda _e: self.node.request('shape', shape=self.shape_var.get()))
        ttk.Label(f, text='Runs').grid(row=1, column=0, sticky='w', pady=(4, 0))
        self.runs_var = tk.IntVar(value=self.node.num_runs)
        ttk.Spinbox(f, from_=1, to=100, textvariable=self.runs_var, width=6) \
            .grid(row=1, column=1, sticky='w', padx=4, pady=(4, 0))

        btns = ttk.Frame(f)
        btns.grid(row=2, column=0, columnspan=2, sticky='we', pady=(8, 0))
        for i in range(4):
            btns.columnconfigure(i, weight=1)
        ttk.Button(btns, text='▶ Start', style='Primary.TButton', command=self.on_start).grid(row=0, column=0, sticky='we')
        ttk.Button(btns, text='Evaluate', command=self.on_evaluate) \
            .grid(row=0, column=1, sticky='we', padx=2)
        ttk.Button(btns, text='■ Stop', command=lambda: self.node.request('stop')) \
            .grid(row=0, column=2, sticky='we')
        ttk.Button(btns, text='↺ Reset', command=lambda: self.node.request('reset')) \
            .grid(row=0, column=3, sticky='we', padx=(2, 0))
        f.columnconfigure(1, weight=1)
        ttk.Label(f, text='Evaluate = every path × Runs. Keys: Enter start · S stop · R reset',
                  foreground='#777777').grid(row=3, column=0, columnspan=2, sticky='w', pady=(6, 0))
        ttk.Button(f, text='? Scoring rules', command=self.show_rules) \
            .grid(row=4, column=0, columnspan=2, sticky='w', pady=(4, 0))

    def _build_eval_controls(self, panel):
        """Evaluation mode (evaluate.sh): start button, progress and a summary
        instead of the path / runs / Start controls."""
        n = self.node
        f = ttk.LabelFrame(panel, text='Evaluation', padding=8)
        f.pack(fill=tk.X, pady=(0, 6))
        self.eval_plan_lbl = ttk.Label(f, wraplength=600)
        self.eval_plan_lbl.pack(anchor='w')

        btns = ttk.Frame(f)
        btns.pack(fill=tk.X, pady=(6, 4))
        btns.columnconfigure(0, weight=3)
        btns.columnconfigure(1, weight=1)
        btns.columnconfigure(2, weight=1)
        self.eval_btn = ttk.Button(btns, text='▶ Start evaluation', style='Primary.TButton',
                                   command=lambda: n.request('evaluate', num_runs=n.eval_runs))
        self.eval_btn.grid(row=0, column=0, sticky='we')
        ttk.Button(btns, text='■ Stop', command=lambda: n.request('stop')) \
            .grid(row=0, column=1, sticky='we', padx=(2, 0))
        ttk.Button(btns, text='? Scoring rules', command=self.show_rules) \
            .grid(row=0, column=2, sticky='we', padx=(2, 0))

        self.eval_prog = ttk.Progressbar(f, maximum=1.0)
        self.eval_prog.pack(fill=tk.X, pady=(2, 2))
        self.eval_prog_lbl = ttk.Label(f, text='–')
        self.eval_prog_lbl.pack(anchor='w')

        wt, wv, ws = self._maxima()
        cols = (('shape', 'Path', 88), ('done', 'Done', 50), ('ok', 'Stopped', 62),
                ('track', 'Track', 72), ('time', 'Time', 66), ('stop', 'Stop', 62),
                ('score', 'Score', 80), ('err', '|err| m', 62))
        eval_frame = ttk.Frame(f)
        eval_frame.pack(fill=tk.X, pady=(8, 0))
        self.eval_table = ttk.Treeview(eval_frame, columns=[c[0] for c in cols], show='headings',
                                       height=len(n.eval_shapes))
        for key, text, cw in cols:
            self.eval_table.heading(key, text=text)
            self.eval_table.column(key, width=cw, minwidth=cw, anchor='center',
                                   stretch=key == 'shape')
        for tag, color in (('good', '#1f8a3b'), ('mid', '#c98a00'), ('bad', '#c0392b'),
                           ('todo', '#999999')):
            self.eval_table.tag_configure(tag, foreground=color)
        self._scroll_table(self.eval_table)
        self.overall_lbl = tk.Label(f, text='Overall score: –', bg='#eef2f7',
                                    font=('DejaVu Sans', 20, 'bold'))
        self.overall_lbl.pack(anchor='w', pady=(6, 0))
        self.parts_lbl = ttk.Label(f, text='', wraplength=600)
        self.parts_lbl.pack(anchor='w')
        ttk.Label(f, text=f'Score = tracking ({wt} pts) + time ({wv} pts) + stop ({ws} pts) · '
                          '≥ 80 green, ≥ 50 orange, < 50 red · Key: S stop',
                  foreground='#777777', wraplength=600).pack(anchor='w')

    def rules_text(self):
        """Scoring rules, filled in from the node's current parameters."""
        n = self.node
        wt, wv, ws = (100 * n.w_track, 100 * n.w_time, 100 * n.w_stop)
        L, t = 60.0, 6.0               # worked example: path length, run time
        e = n.stop_free + 2.0          # ... and stop error (2 m beyond the free zone)
        ex_track = wt * (1 - 0.10 / n.max_cte)
        ex_time = wv * n.time_fraction(L / t)
        ex_stop = ws * n.stop_fraction(e)
        return f"""SCORING RULES

Task (every run)
  Drive the path from the start (green dot) to the finish node (red line)
  as fast as you can and come to a stop on the finish node. The dotted part
  after the finish ({n.path_kwargs['runout']:.0f} m run-out) is only there to absorb overshoot.

How a run ends
  stopped    the car has been at rest (moved < {100 * n.stop_eps:.0f} cm) for {n.stop_hold:.1f} s
             after setting off
  off_track  {f'distance from the path > {n.max_cte:.1f} m':<40} -> time and stop points = 0
  overrun    {'the car reached the end of the run-out':<40} -> time and stop points = 0
  timeout    {f'the run took longer than {n.time_limit:.0f} s':<40} -> time and stop points = 0

Score (max 100) = Tracking (max {wt:.0f}) + Time (max {wv:.0f}) + Stop (max {ws:.0f})

  Tracking = {wt:.0f} x completion x max(0, 1 - RMS error / {n.max_cte:.1f} m)
    completion  share of the path up to the finish node that was driven (0..1)
    RMS error   root-mean-square distance from the path during the run

  Time = {wv:.0f} x min(v_eq, {n.time_v_max:.0f}) / {n.time_v_max:.0f}          (only if 'stopped')
    v_eq        equivalent speed = path length to the finish node / run time
    run time    from the start until the car came to rest
    linear:     {n.time_v_max / 3:.0f} m/s -> {wv / 3:.1f},  {n.time_v_max / 2:.0f} m/s -> {wv / 2:.1f},  {n.time_v_max:.0f} m/s -> {wv:.0f} points

  Stop = {ws:.0f} points if |stop error| <= {n.stop_free:.1f} m,
         then {n.stop_penalty:.0f} points less per metre beyond {n.stop_free:.1f} m
         (0 points from {n.stop_zero_dist():.1f} m; only if the run ended 'stopped')
    stop error  distance along the path from the finish node to where the
                car stopped (- = short of it, + = past it)

  Example: {L:.0f} m path, RMS error 0.10 m, at rest after {t:.0f} s, {e:.1f} m past the finish
    Tracking = {f'{wt:.0f} x 1 x (1 - 0.10/{n.max_cte:.1f})':<26} = {ex_track:5.1f}
    Time     = {f'{wv:.0f} x {L / t:.0f}/{n.time_v_max:.0f}':<26} = {ex_time:5.1f}   (v_eq = {L:.0f} m / {t:.0f} s = {L / t:.0f} m/s)
    Stop     = {f'{ws:.0f} - {n.stop_penalty:.0f} x ({e:.1f} - {n.stop_free:.1f})':<26} = {ex_stop:5.1f}
    Score    = {'':<26}   {ex_track + ex_time + ex_stop:5.1f} / 100

Evaluation
  Every path ({', '.join(n.eval_shapes)}) is run {n.eval_runs} time(s).
  Path score = average of its runs; overall score = average of all runs.
"""

    def show_rules(self):
        """Read-only window with the scoring rules (one at a time)."""
        win = getattr(self, '_rules_win', None)
        if win is not None and win.winfo_exists():
            win.lift()
            return
        win = tk.Toplevel(self.root)
        win.title('Scoring rules')
        text = self.rules_text()
        win.geometry('880x650')
        box = ScrolledText(win, width=78, height=28, wrap='word',
                           font=('TkFixedFont', 10), padx=16, pady=12, relief='flat',
                           background='white', foreground='#243247')
        box.insert('1.0', text)
        box.configure(state='disabled')
        box.pack(fill=tk.BOTH, expand=True)
        ttk.Button(win, text='Close', command=win.destroy).pack(pady=(0, 8))
        self._rules_win = win

    def _maxima(self):
        """Maximum points of tracking, time and stop (as rounded ints)."""
        n = self.node
        return tuple(round(100 * w) for w in (n.w_track, n.w_time, n.w_stop))

    def _parts(self, tracking, timing, stop):
        """Points earned in each part, as 'got/max' strings (inputs are 0-100)."""
        n = self.node
        return tuple(f'{w * v:.1f}/{100 * w:.0f}' for w, v in
                     ((n.w_track, tracking), (n.w_time, timing), (n.w_stop, stop)))

    @staticmethod
    def _score_tag(score):
        return 'good' if score >= 80 else 'mid' if score >= 50 else 'bad'

    def _update_eval_panel(self, state, results, run_idx, run_total, shape, changed):
        n = self.node
        per = n.eval_runs
        self.eval_plan_lbl.configure(
            text=f"{len(n.eval_shapes)} paths × {per} run(s): {', '.join(n.eval_shapes)}")
        total = run_total if n.started else len(n.eval_shapes) * per
        complete = n.started and state == DONE and len(results) >= total and not n.pending_eval
        self.eval_prog['value'] = len(results) / total if total else 0.0
        if n.pending_eval:
            text = 'Waiting for vehicle_sim and a driver on /path…'
        elif not n.started:
            text = 'Press ▶ Start evaluation'
        elif state in (SETTLING, RUNNING):
            k = sum(r['shape'] == shape for r in results) + 1
            text = f'Running {run_idx}/{total}: {shape} (run {k}/{per})'
        elif complete:
            text = f'Finished: {len(results)}/{total} runs'
        else:
            text = f'Stopped after {len(results)}/{total} runs'
        self.eval_prog_lbl.configure(text=text)
        self.eval_btn.configure(text='↻ Restart evaluation' if n.started or n.pending_eval
                                else '▶ Start evaluation')

        if not changed:
            return
        summary = n.summarize(results)
        self.eval_table.delete(*self.eval_table.get_children())
        for sh in n.eval_shapes:
            row = summary.get(sh)
            if row is None:
                self.eval_table.insert('', tk.END, tags=('todo',),
                                       values=(sh, f'0/{per}', '–', '–', '–', '–', '–', '–'))
                continue
            err = '–' if row['mean_abs_stop_err'] is None else f"{row['mean_abs_stop_err']:.2f}"
            track, timing, stop = self._parts(row['tracking'], row['time_score'], row['stop'])
            self.eval_table.insert('', tk.END, tags=(self._score_tag(row['score']),), values=(
                sh, f"{row['runs']}/{per}", f"{row['stopped']}/{row['runs']}",
                track, timing, stop, f"{row['score']:.1f}/100", err))
        if results:
            k = len(results)
            avg = sum(r['score'] for r in results) / k
            track, timing, stop = self._parts(sum(r['tracking'] for r in results) / k,
                                              sum(r['time_score'] for r in results) / k,
                                              sum(r['stop'] for r in results) / k)
            color = {'good': '#1f8a3b', 'mid': '#c98a00', 'bad': '#c0392b'}[self._score_tag(avg)]
            self.overall_lbl.configure(text=f'Overall score: {avg:.1f}/100', fg=color)
            v = sum(r['v_eq'] for r in results if r['result'] == 'stopped')
            n_ok = sum(r['result'] == 'stopped' for r in results)
            v_txt = f', mean v_eq {v / n_ok:.1f} m/s' if n_ok else ''
            self.parts_lbl.configure(
                text=f'= tracking {track} + time {timing} + stop {stop}   ({k} runs{v_txt})')
        else:
            self.overall_lbl.configure(text='Overall score: –', fg='black')
            self.parts_lbl.configure(text='')

    def _bind_keys(self):
        self.root.bind('<KeyPress-s>', lambda _e: self.node.request('stop'))
        if not self.node.eval_mode:
            self.root.bind('<Return>', lambda _e: self.on_start())
            self.root.bind('<KeyPress-r>', lambda _e: self.node.request('reset'))

    # ------------------------------------------------------------ actions
    def on_start(self):
        self.node.request('start', shape=self.shape_var.get(), num_runs=self._runs())

    def _runs(self):
        try:
            return max(1, int(self.runs_var.get()))
        except (tk.TclError, ValueError):
            return 1

    def on_evaluate(self):
        self.node.request('evaluate', num_runs=self._runs())

    def fit_view(self):
        if self._path_drawn is None:
            return
        pts = self._path_drawn
        m = self.node.max_cte + 3.0
        self.ax.set_xlim(pts[:, 0].min() - m, pts[:, 0].max() + m)
        self.ax.set_ylim(pts[:, 1].min() - m, pts[:, 1].max() + m)
        self.follow_var.set(False)
        self._need_full = True

    def _on_callback_error(self, exc, val, tb):
        # Ctrl-C arrives inside a Tk callback: treat it as closing the window.
        if issubclass(exc, KeyboardInterrupt):
            self.close()
        else:
            traceback.print_exception(exc, val, tb)

    def close(self):
        self._closing = True
        self.node.finished = True
        try:
            self.root.after_cancel(self._after_id)
        except tk.TclError:
            pass
        self.root.quit()

    def run(self):
        self.root.mainloop()
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    # ------------------------------------------------------------ drawing
    def _draw_path(self, pts, fin):
        self._path_drawn = pts
        self.path_line.set_data(pts[:fin + 1, 0], pts[:fin + 1, 1])
        self.runout_line.set_data(pts[fin:, 0], pts[fin:, 1])
        self.path_nodes.set_data(pts[:, 0], pts[:, 1])
        self.start_mark.set_data([pts[0, 0]], [pts[0, 1]])
        # Off-track corridor (|cte| <= max_cte) as a shaded band.
        yaw = path_yaws(pts)
        n = np.stack([-np.sin(yaw), np.cos(yaw)], axis=1) * self.node.max_cte
        # finish line across the corridor at the finish node
        fl = np.array([pts[fin] + n[fin], pts[fin] - n[fin]])
        self.finish_line.set_data(fl[:, 0], fl[:, 1])
        band = np.vstack([pts + n, (pts - n)[::-1]])
        if self.corridor is not None:
            self.corridor.remove()
        self.corridor = Polygon(band, closed=True, fc='#2ca02c14', ec='#2ca02c55',
                                lw=0.8, zorder=0)
        self.ax.add_patch(self.corridor)
        self.fit_view()
        self._need_full = True

    def _draw_vehicle(self, x, y, yaw, steer):
        c, s = math.cos(yaw), math.sin(yaw)

        def tf(px, py):
            return x + c * px - s * py, y + s * px + c * py

        hw = WIDTH / 2
        corners = [(-REAR_OVERHANG, -hw), (WHEELBASE + FRONT_OVERHANG, -hw),
                   (WHEELBASE + FRONT_OVERHANG, hw), (-REAR_OVERHANG, hw)]
        self.body.set_xy([tf(*p) for p in corners])
        h = WHEEL_LEN / 2
        r = [tf(-h, 0), tf(h, 0)]
        self.rear_wheel.set_data([p[0] for p in r], [p[1] for p in r])
        cs, ss = math.cos(steer), math.sin(steer)
        f = [tf(WHEELBASE - cs * h, -ss * h), tf(WHEELBASE + cs * h, ss * h)]
        self.front_wheel.set_data([p[0] for p in f], [p[1] for p in f])

    def update(self):
        if self._closing:
            return
        if self.node.finished or not rclpy.ok():   # done, Ctrl-C or SIGTERM
            self.close()
            return
        node = self.node
        with node.lock:
            pts, fin = node.path_pts, node.finish_idx
            state = node.state
            veh, ctrl = node.veh, node.ctrl
            trail = np.array(node.trail) if node.trail else None
            cte_hist = np.array(node.cte_hist) if node.cte_hist else None
            ctrl_hist = np.array(node.ctrl_hist) if node.ctrl_hist else None
            act_hist = np.array(node.act_hist) if node.act_hist else None
            act = node.act
            cur = dict(node.cur)
            results = list(node.results)
            run_idx, num_runs, shape = node.run_idx, node.run_total, node.shape
        conn = node.connections()

        if pts is not self._path_drawn:
            self._draw_path(pts, fin)
            if not self.node.eval_mode and self.shape_var.get() != shape:
                self.shape_var.set(shape)

        # map
        if trail is not None:
            self.trail_line.set_data(trail[:, 0], trail[:, 1])
        else:
            self.trail_line.set_data([], [])
        if veh is not None:
            steer = act[1] if act is not None else 0.0
            self._draw_vehicle(veh[0], veh[1], veh[2], steer)
            if self.follow_var.get():
                (x0, x1), (y0, y1) = self.ax.get_xlim(), self.ax.get_ylim()
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                if (abs(veh[0] - cx) > FOLLOW_SPAN / 2 or abs(veh[1] - cy) > FOLLOW_SPAN / 2
                        or abs((x1 - x0) - 2 * FOLLOW_SPAN) > 1e-6):
                    # Recenter ahead of the car so this happens only every few seconds.
                    cx = veh[0] + FOLLOW_SPAN / 3 * math.cos(veh[2])
                    cy = veh[1] + FOLLOW_SPAN / 3 * math.sin(veh[2])
                    self._set_lim(self.ax, 'x', cx - FOLLOW_SPAN, cx + FOLLOW_SPAN)
                    self._set_lim(self.ax, 'y', cy - FOLLOW_SPAN, cy + FOLLOW_SPAN)

        # time plots: last TIME_WINDOW seconds, plotted relative to the newest sample
        ends = [h[-1, 0] for h in (cte_hist, ctrl_hist, act_hist) if h is not None]
        t_now = max(ends) if ends else 0.0
        if cte_hist is not None:
            w = cte_hist[cte_hist[:, 0] >= t_now - TIME_WINDOW]
            self.cte_line.set_data(w[:, 0] - t_now, w[:, 1])
        else:
            self.cte_line.set_data([], [])
        if ctrl_hist is not None:
            w = ctrl_hist[ctrl_hist[:, 0] >= t_now - TIME_WINDOW]
            steer_deg = np.degrees(w[:, 2])
            self.v_line.set_data(w[:, 0] - t_now, w[:, 1])
            self.steer_line.set_data(w[:, 0] - t_now, steer_deg)
            # Limits only ever grow (in coarse steps), so they rarely force a redraw.
            v_lo, v_hi = self.ax_v.get_ylim()
            vmax = max(v_hi, 2.0 * math.ceil(w[:, 1].max() * 1.1 / 2.0))
            vmin = min(v_lo, -2.0 * math.ceil(max(0.0, -w[:, 1].min()) * 1.1 / 2.0))
            self._set_lim(self.ax_v, 'y', vmin, vmax)
            lim = max(self.ax_steer.get_ylim()[1],
                      10.0 * math.ceil(np.abs(steer_deg).max() * 1.1 / 10.0))
            self._set_lim(self.ax_steer, 'y', -lim, lim)
        else:
            self.v_line.set_data([], [])
            self.steer_line.set_data([], [])
        if act_hist is not None:
            w = act_hist[act_hist[:, 0] >= t_now - TIME_WINDOW]
            self.v_act_line.set_data(w[:, 0] - t_now, w[:, 1])
            self.steer_act_line.set_data(w[:, 0] - t_now, np.degrees(w[:, 2]))
        else:
            self.v_act_line.set_data([], [])
            self.steer_act_line.set_data([], [])

        # panel
        label = f'{state}  (evaluating)' if node.evaluating and state != DONE else state
        self.state_lbl.configure(text=label, foreground=STATE_COLORS.get(state, 'black'))
        for key, title, connected in (('vehicle', 'Vehicle', conn['state_pubs']),
                                       ('driver', 'Driver', conn['ctrl_pubs'])):
            self.conn_labels[key].configure(
                text=f'● {title}: {"connected" if connected else "offline"}',
                fg='#16815d' if connected else '#b54747')
        self.status_text.set(
            f'{shape.replace("_", " ").title()}  •  {state.title()}  •  '
            'Live signals: last 10 seconds  •  Solid: command / dashed: actual')

        prog = cur.get('progress', 0.0) * 100.0
        self.progress['value'] = prog
        self.live['Run'].configure(text=f'{run_idx} / {num_runs}  ({shape})')
        self.live['Time'].configure(text=f"{cur.get('time', 0.0):6.2f} s / {node.time_limit:.0f} s")
        self.live['Progress'].configure(text=f'{prog:6.1f} %  (to finish)')
        self.live['To finish'].configure(text=f"{cur.get('to_finish', 0.0):+7.2f} m")
        self.live['CTE'].configure(text=f"{cur.get('cte', 0.0):+6.3f} m  (limit ±{node.max_cte:.1f})")
        self.live['RMS CTE'].configure(text=f"{cur.get('rms_cte', 0.0):6.3f} m")
        self.live['Pose'].configure(
            text=('–' if veh is None else
                  f'x {veh[0]:6.2f}  y {veh[1]:6.2f}  ψ {math.degrees(veh[2]):6.1f}°'))
        self.live['Command'].configure(
            text=('–' if ctrl is None else
                  f'v {ctrl[0]:5.2f} m/s  δ {math.degrees(ctrl[1]):5.1f}°'))
        self.live['Actual'].configure(
            text=('–' if act is None else
                  f'v {act[0]:5.2f} m/s  δ {math.degrees(act[1]):5.1f}°'))

        changed = len(results) != self._n_results
        if self.node.eval_mode:
            self._update_eval_panel(state, results, run_idx, num_runs, shape, changed)
        if changed:
            self._n_results = len(results)
            self.table.delete(*self.table.get_children())
            for r in results:
                err = '–' if r['stop_err'] is None else f"{r['stop_err']:+.2f}"
                track, timing, stop = self._parts(r['tracking'], r['time_score'], r['stop'])
                self.table.insert('', tk.END, tags=(r['result'],), values=(
                    r['run'], r['shape'], r['result'], track, timing, stop,
                    f"{r['score']:.1f}/100", err))
            if results:
                avg = sum(r['score'] for r in results) / len(results)
                self.avg_lbl.configure(text=f'Average score: {avg:.1f}/100')
            else:
                self.avg_lbl.configure(text='Average score: –')

        try:
            self._render()
        except tk.TclError:
            # Ctrl-C can interrupt a blit halfway; ignore it once shutting down.
            if not (self._closing or self.node.finished or not rclpy.ok()):
                raise
        self._after_id = self.root.after(UPDATE_MS, self.update)

    def _render(self):
        if self._need_full or self._bg is None:
            self._need_full = False
            self.canvas.draw()          # full redraw; _on_draw recaches background
        else:
            self.canvas.restore_region(self._bg)
            self._draw_animated()
        self.canvas.blit(self.fig.bbox)
