"""Evaluation report figure (matplotlib, no pyplot): used by the results window
in the UI and saved as PNG next to the JSON results."""
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

TRACK_COLOR = '#1f77b4'
TIME_COLOR = '#2ca02c'
STOP_COLOR = '#ff7f0e'
FAIL_COLOR = '#c0392b'


def make_report_figure(results, summary, weights, stop_free, stop_zero):
    """weights: (tracking, time, stop) shares of the 100 points;
    stop_free: full stop points within this distance [m]; stop_zero: 0 points from here."""
    """results: list of run dicts; summary: SimCtrlNode.summarize(results)."""
    shapes = list(summary)
    overall = sum(r['score'] for r in results) / len(results)
    fig = Figure(figsize=(11, 2.2 + 0.9 * len(shapes)), dpi=100, constrained_layout=True)
    fig.suptitle(f'Evaluation: overall score {overall:.1f} / 100   ({len(results)} runs)',
                 fontsize=14, fontweight='bold')
    ax, ax2 = fig.subplots(1, 2, gridspec_kw={'width_ratios': [1.15, 1]})
    y = np.arange(len(shapes))[::-1]   # first path on top

    # --- score per path, split into its tracking, time and stop parts
    wt, wv, ws = weights
    track = np.array([wt * summary[s]['tracking'] for s in shapes])
    timing = np.array([wv * summary[s]['time_score'] for s in shapes])
    stop = np.array([ws * summary[s]['stop'] for s in shapes])
    ax.barh(y, track, color=TRACK_COLOR, label=f'tracking (max {100 * wt:.0f})')
    ax.barh(y, timing, left=track, color=TIME_COLOR, label=f'time (max {100 * wv:.0f})')
    ax.barh(y, stop, left=track + timing, color=STOP_COLOR, label=f'stop (max {100 * ws:.0f})')
    for yi, s in zip(y, shapes):
        row = summary[s]
        ax.text(row['score'] + 1.5, yi, f"{row['score']:.1f}", va='center',
                fontweight='bold', fontsize=11)
    ax.axvline(overall, color='#555555', ls='--', lw=1)
    ax.set_yticks(y, [f"{s}\n{summary[s]['stopped']}/{summary[s]['runs']} stopped"
                      for s in shapes])
    ax.set_xlim(0, 112)
    ax.set_xlabel('average score (max 100)')
    ax.set_title('Score per path')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=3, fontsize=8)
    ax.grid(True, axis='x', alpha=0.3)

    # --- where each run stopped relative to the finish node
    errs = [abs(r['stop_err']) for r in results if r['result'] == 'stopped']
    lim = max(stop_zero * 1.1, max(errs, default=0.0) * 1.15)
    ax2.axvspan(-stop_zero, stop_zero, color='#2ca02c', alpha=0.07,
                label=f'some stop points (±{stop_zero:.0f} m)')
    ax2.axvspan(-stop_free, stop_free, color='#2ca02c', alpha=0.15,
                label=f'full stop points (±{stop_free:.0f} m)')
    ax2.axvline(0.0, color=FAIL_COLOR, lw=2, label='finish node')
    for yi, s in zip(y, shapes):
        rs = [r for r in results if r['shape'] == s]
        ok = [r['stop_err'] for r in rs if r['result'] == 'stopped']
        jitter = np.linspace(-0.15, 0.15, len(ok)) if len(ok) > 1 else np.zeros(len(ok))
        ax2.scatter(ok, yi + jitter, color=STOP_COLOR, edgecolor='black', zorder=3, s=40)
        failed = [r['result'] for r in rs if r['result'] != 'stopped']
        if failed:
            counts = {k: failed.count(k) for k in dict.fromkeys(failed)}
            ax2.text(lim * 0.97, yi, '✗ ' + ', '.join(f'{v} {k}' for k, v in counts.items()),
                     color=FAIL_COLOR, ha='right', va='center', fontsize=9)
    ax2.set_yticks(y, shapes)
    ax2.set_ylim(-0.6, len(shapes) - 0.4)
    ax2.set_xlim(-lim, lim)
    ax2.set_xlabel('stop position − finish node [m]   (− short, + past)')
    ax2.set_title('Where each run stopped')
    ax2.legend(loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=3, fontsize=8)
    ax2.grid(True, axis='x', alpha=0.3)
    return fig


def save_report_png(path, results, summary, weights, stop_free, stop_zero):
    fig = make_report_figure(results, summary, weights, stop_free, stop_zero)
    FigureCanvasAgg(fig)
    fig.savefig(path)
