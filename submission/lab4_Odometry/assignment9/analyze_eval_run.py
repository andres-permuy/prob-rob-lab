#!/usr/bin/env python3
"""Assignments 8-9: covariance and error summary of the evaluation run.

Reads data/eval_run.zip (recorded while assignment7/drive_pattern.py was
running), writes assignment9_covariance.png next to this script, and prints
the numbers quoted in the report. Requires a sourced ROS 2 environment
(rosbag2_py). Run from any directory:
    python3 analyze_eval_run.py
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
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64

HERE = Path(__file__).resolve().parent
BAG_ZIP = HERE / 'data' / 'eval_run.zip'
OUTPUT = HERE / 'assignment9_covariance.png'
DRIVING = (5.0, 61.0)  # s after the first command (drive_pattern.py segments)

INK, MUTED = '#0b0b0b', '#52514e'
BLUE, ORANGE = '#2a78d6', '#eb6834'


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag():
    """Return covariance, command, and error arrays from the zipped bag."""
    with tempfile.TemporaryDirectory() as tmp:
        with zipfile.ZipFile(BAG_ZIP) as archive:
            archive.extractall(tmp)
        return read_bag_dir(Path(tmp) / 'eval_run')


def read_bag_dir(bag_dir):
    """Return covariance, command, and error arrays from an extracted bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    odom, cmd, err = [], [], {'/ekf_error/position': [], '/ekf_error/yaw': []}
    while reader.has_next():
        topic, raw, received = reader.read_next()
        if topic == '/ekf_odom':
            m = deserialize_message(raw, Odometry)
            p, t = m.pose.covariance, m.twist.covariance
            odom.append((stamp(m), p[0], p[7], p[35], t[0], t[35]))
        elif topic == '/cmd_vel':
            cmd.append((stamp(deserialize_message(raw, TwistStamped)), received * 1e-9))
        elif topic in err:
            err[topic].append((received * 1e-9, deserialize_message(raw, Float64).data))
    return (np.array(odom), np.array(cmd),
            np.array(err['/ekf_error/position']), np.array(err['/ekf_error/yaw']))


def value_at(times, values, t):
    return values[min(np.searchsorted(times, t), len(times) - 1)]


def main():
    odom, cmd, e_pos, e_yaw = read_bag()
    odom = odom[np.argsort(odom[:, 0])]
    t = odom[:, 0] - cmd[0, 0]  # sim time since the first drive command
    sx, sy, sth = np.sqrt(odom[:, 1]), np.sqrt(odom[:, 2]), np.sqrt(odom[:, 3])
    sv, sw = np.sqrt(odom[:, 4]), np.sqrt(odom[:, 5])

    print('time since first command (s)  sigma_x(mm)  sigma_y(mm)  sigma_theta(deg)')
    for label, tq in (('bag start', t[0]), ('drive start', DRIVING[0]),
                      ('drive end', DRIVING[1]), ('bag end', t[-1])):
        i = min(np.searchsorted(t, tq), len(t) - 1)
        print(f'{label:12s} {t[i]:7.1f}   {sx[i]*1e3:9.3f}  {sy[i]*1e3:9.3f}  '
              f'{np.degrees(sth[i]):12.4f}')
    print(f'sigma_v: median {np.median(sv)*1e3:.3f} mm/s (range '
          f'{sv.min()*1e3:.3f}-{sv.max()*1e3:.3f}); sigma_w: median '
          f'{np.degrees(np.median(sw)):.4f} deg/s (range {np.degrees(sw.min()):.4f}-'
          f'{np.degrees(sw.max()):.4f})')

    # Errors from /ekf_error at the end of driving (median of the last 1 s
    # avoids the isolated ground-truth timestamp spikes).
    end_wall = cmd[0, 1] + (cmd[-1, 1] - cmd[0, 1]) * DRIVING[1] / (cmd[-1, 0] - cmd[0, 0])
    window = lambda e: e[(e[:, 0] > end_wall - 1.0) & (e[:, 0] <= end_wall), 1]
    ep, eth = np.median(window(e_pos)), np.median(window(e_yaw))
    i = min(np.searchsorted(t, DRIVING[1]), len(t) - 1)
    s_pos = np.hypot(sx[i], sy[i])
    print(f'end of driving: e_p = {ep*1e3:.1f} mm vs sqrt(Sxx+Syy) = {s_pos*1e3:.2f} mm '
          f'(ratio {ep/s_pos:.0f}); e_theta = {eth:.3f} deg vs sigma_theta = '
          f'{np.degrees(sth[i]):.4f} deg (ratio {eth/np.degrees(sth[i]):.0f})')

    fig, axes = plt.subplots(2, 2, figsize=(11, 6), sharex=True)
    panels = [
        (axes[0, 0], [(sx * 1e3, r'$\sigma_x$', BLUE), (sy * 1e3, r'$\sigma_y$', ORANGE)],
         'position std. dev. (mm)'),
        (axes[0, 1], [(np.degrees(sth), r'$\sigma_\theta$', BLUE)], r'$\sigma_\theta$ (deg)'),
        (axes[1, 0], [(sv * 1e3, r'$\sigma_v$', BLUE)], r'$\sigma_v$ (mm/s)'),
        (axes[1, 1], [(np.degrees(sw), r'$\sigma_\omega$', BLUE)], r'$\sigma_\omega$ (deg/s)'),
    ]
    for ax, series, ylabel in panels:
        ax.axvspan(*DRIVING, color=MUTED, alpha=0.12, linewidth=0)
        for values, label, color in series:
            ax.plot(t, values, color=color, linewidth=2, label=label)
        ax.set_ylabel(ylabel, color=MUTED)
        ax.grid(alpha=0.25)
        ax.set_ylim(bottom=0)
        for spine in ('top', 'right'):
            ax.spines[spine].set_visible(False)
        if len(series) > 1:
            ax.legend(frameon=False, loc='upper left')
    axes[0, 1].annotate('driving', ((DRIVING[0] + DRIVING[1]) / 2, 0), xytext=(0, 4),
                        textcoords='offset points', ha='center', color=MUTED, fontsize=9)
    for ax in axes[1]:
        ax.set_xlabel('time since first drive command (s)', color=MUTED)
    fig.suptitle('Standard deviations published on /ekf_odom (shaded: robot driving)',
                 color=INK, fontsize=11)
    fig.tight_layout()
    fig.savefig(OUTPUT, dpi=180)
    print(f'Wrote {OUTPUT}')


if __name__ == '__main__':
    main()
