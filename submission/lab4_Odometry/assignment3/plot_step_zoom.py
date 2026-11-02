#!/usr/bin/env python3
"""Annotated close-up of the first linear and first angular velocity step.

Reads data/step_response.zip (recorded while velocity_step.py was running),
plots the ground-truth samples around each step onset, and marks the initial
level, final level, 90% level, and the interpolated 90% crossing that define
G and tau in the report. The initial/final levels are the PlotJuggler
measurements used in the report's worked example.

Requires a sourced ROS 2 environment (rosbag2_py). Run from any directory:
    python3 plot_step_zoom.py
Writes data/assignment3_step_zoom.png.
"""

import tempfile
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from geometry_msgs.msg import TwistStamped
from rclpy.serialization import deserialize_message

DATA_DIR = Path(__file__).resolve().parent / 'data'
BAG_ZIP = DATA_DIR / 'step_response.zip'
OUTPUT = DATA_DIR / 'assignment3_step_zoom.png'

# (field, onset [s], u0, u1, initial level, final level, label, unit)
STEPS = [
    ('linear', 6153.5, 0.00, 0.03, -0.001882, 0.030312, 'v', 'm/s'),
    ('angular', 6165.5, 0.00, 0.10, -0.001047, 0.100211, r'\omega', 'rad/s'),
]
WINDOW = (-0.10, 0.20)  # seconds around the step onset

INK, MUTED = '#0b0b0b', '#52514e'
GT_COLOR, CMD_COLOR = '#2a78d6', '#eb6834'


def read_bag():
    """Return {topic: array[[stamp, linear.x, angular.z], ...]} from the bag."""
    data = {'/cmd_vel': [], '/tb3/ground_truth/twist': []}
    with tempfile.TemporaryDirectory() as tmp:
        with zipfile.ZipFile(BAG_ZIP) as archive:
            archive.extractall(tmp)
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(Path(tmp) / 'step_response'),
                                              storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic not in data:
                continue
            msg = deserialize_message(raw, TwistStamped)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            data[topic].append((stamp, msg.twist.linear.x, msg.twist.angular.z))
    return {topic: np.array(rows) for topic, rows in data.items()}


def crossing_time(t, y, onset, level):
    """Linearly interpolate the first time after onset that y reaches level."""
    k = np.flatnonzero((t >= onset) & (y >= level))[0]
    return t[k - 1] + (level - y[k - 1]) / (y[k] - y[k - 1]) * (t[k] - t[k - 1])


def plot_step(ax, bag, field, onset, u0, u1, y_i, y_f, sym, unit):
    col = 1 if field == 'linear' else 2
    gt, cmd = bag['/tb3/ground_truth/twist'], bag['/cmd_vel']
    level = y_i + 0.9 * (y_f - y_i)
    tau = crossing_time(gt[:, 0], gt[:, col], onset, level) - onset
    gain = (y_f - y_i) / (u1 - u0)

    for arr, color, label, kw in (
            (cmd, CMD_COLOR, f'command $u_{{{sym}}}$', dict(drawstyle='steps-post')),
            (gt, GT_COLOR, f'ground truth ${sym}$', dict(marker='o', markersize=4))):
        sel = (arr[:, 0] >= onset + WINDOW[0]) & (arr[:, 0] <= onset + WINDOW[1])
        ax.plot((arr[sel, 0] - onset) * 1e3, arr[sel, col], color=color,
                linewidth=2, label=label, **kw)

    # Label each level where the data is far from it: the upper levels on the
    # left (before the step), the initial level on the right (after the step).
    labels = (
        (y_f, f'${sym}_f$ = {y_f:.6f}', WINDOW[0], 'left', 'bottom', (4, 3)),
        (level, f'90% = {level:.6f}', WINDOW[0], 'left', 'top', (4, -3)),
        (y_i, f'${sym}_i$ = {y_i:.6f}', WINDOW[1], 'right', 'bottom', (-4, 3)),
    )
    for value, text, x, ha, va, offset in labels:
        ax.axhline(value, color=MUTED, linewidth=1, linestyle='--')
        ax.annotate(text, (x * 1e3, value), xytext=offset, textcoords='offset points',
                    ha=ha, va=va, color=INK, fontsize=9)

    ax.axvline(0, color=MUTED, linewidth=1, linestyle=':')
    ax.axvline(tau * 1e3, color=MUTED, linewidth=1, linestyle=':')
    ax.plot(tau * 1e3, level, marker='o', markersize=9, markerfacecolor='none',
            markeredgecolor=INK, markeredgewidth=1.5, zorder=5)
    arrow_y = y_i + 0.45 * (y_f - y_i)
    ax.annotate('', (tau * 1e3, arrow_y), (0, arrow_y),
                arrowprops=dict(arrowstyle='<->', color=INK, linewidth=1))
    ax.annotate(f'$\\tau$ = {tau * 1e3:.0f} ms', (tau * 1e3, arrow_y), xytext=(6, -3),
                textcoords='offset points', va='center', color=INK, fontsize=10)

    ax.set_title(f'${{u_{{{sym}}}}}$: {u0:g} $\\rightarrow$ {u1:g} {unit}'
                 f'   (G = {gain:.3f}, $\\tau$ = {tau:.3f} s)', fontsize=10, color=INK)
    ax.set_xlabel('time since command step (ms)', color=MUTED)
    ax.set_ylabel(f'${sym}$ ({unit})', color=MUTED)
    ax.set_xlim(WINDOW[0] * 1e3, WINDOW[1] * 1e3)
    ax.grid(alpha=0.25)
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)
    ax.legend(loc='center right', fontsize=9, frameon=False)
    return gain, tau


def main():
    bag = read_bag()
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, step in zip(axes, STEPS):
        gain, tau = plot_step(ax, bag, *step)
        print(f'{step[0]:8s} step: G = {gain:.4f}, tau = {tau:.4f} s')
    fig.tight_layout()
    fig.savefig(OUTPUT, dpi=200)
    print(f'Wrote {OUTPUT}')


if __name__ == '__main__':
    main()
