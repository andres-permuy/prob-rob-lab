#!/usr/bin/env python3
"""Analyze ground-truth velocities in zero_cmd_bag.zip.

Reads the uncompressed MCAP file in the archive and decodes
geometry_msgs/msg/TwistStamped (CDR). Only the Python standard library,
NumPy, and Matplotlib are required; ROS 2 is not required.

Run from any directory:
    python analyze_zero_cmd.py

Reads data/zero_cmd_bag.zip and writes histogram plots, a CSV of the
statistics, and a text report into data/. Standard deviations are sample
standard deviations (ddof=1).
"""

from __future__ import annotations

import csv
import math
import struct
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


EVIDENCE_DIR = Path(__file__).resolve().parent / "data"
BAG_ZIP = EVIDENCE_DIR / "zero_cmd_bag.zip"
MCAP_MEMBER = "zero_cmd_bag/zero_cmd_bag_0.mcap"
TOPIC = "/tb3/ground_truth/twist"
MCAP_MAGIC = b"\x89MCAP0\r\n"


class CdrReader:
    """Small reader for the little-endian CDR encoding used by ROS 2."""

    def __init__(self, payload: bytes):
        if payload[:2] != b"\x00\x01":
            raise ValueError(f"Unsupported CDR encapsulation: {payload[:4]!r}")
        self.data = payload
        self.pos = 4
        self.origin = 4

    def align(self, boundary: int) -> None:
        relative = self.pos - self.origin
        self.pos += (-relative) % boundary

    def unpack(self, fmt: str):
        size = struct.calcsize("<" + fmt)
        value = struct.unpack_from("<" + fmt, self.data, self.pos)
        self.pos += size
        return value[0] if len(value) == 1 else value

    def string(self) -> str:
        length = self.unpack("I")
        raw = self.data[self.pos:self.pos + length]
        if len(raw) != length:
            raise ValueError("Truncated CDR string")
        self.pos += length
        return raw[:-1].decode("utf-8")

    def twist_stamped(self) -> tuple[float, float]:
        # std_msgs/Header: builtin_interfaces/Time and frame_id string.
        self.unpack("i")  # stamp.sec
        self.unpack("I")  # stamp.nanosec
        self.string()      # frame_id
        self.align(8)
        # geometry_msgs/Twist: linear Vector3 then angular Vector3.
        linear_x = self.unpack("d")
        self.unpack("d")  # linear.y
        self.unpack("d")  # linear.z
        self.unpack("d")  # angular.x
        self.unpack("d")  # angular.y
        angular_z = self.unpack("d")
        return linear_x, angular_z


def _record_messages(data: bytes):
    """Yield messages from MCAP records, including uncompressed chunks."""
    pos = 0
    while pos < len(data):
        if data[pos:pos + len(MCAP_MAGIC)] == MCAP_MAGIC:
            break  # trailing MCAP magic after the Footer record
        if pos + 9 > len(data):
            raise ValueError("Truncated MCAP record header")
        opcode = data[pos]
        length = struct.unpack_from("<Q", data, pos + 1)[0]
        start = pos + 9
        end = start + length
        if end > len(data):
            raise ValueError("Truncated MCAP record body")
        body = data[start:end]
        if opcode == 0x05:  # Message
            if len(body) < 22:
                raise ValueError("Malformed MCAP message record")
            channel_id = struct.unpack_from("<H", body, 0)[0]
            yield channel_id, body[22:]
        elif opcode == 0x06:  # Chunk
            if len(body) < 28:
                raise ValueError("Malformed MCAP chunk record")
            offset = 28  # timestamps, uncompressed size, and CRC
            compression_length = struct.unpack_from("<I", body, offset)[0]
            offset += 4
            compression = body[offset:offset + compression_length].decode("utf-8")
            offset += compression_length
            records = body[offset:]
            if compression:
                raise ValueError(
                    f"Compressed MCAP chunks ({compression!r}) are unsupported")
            # This rosbag2 writer prefixes chunk data with its byte length.
            if len(records) >= 8 and struct.unpack_from("<Q", records, 0)[0] == len(records) - 8:
                records = records[8:]
            yield from _record_messages(records)
        pos = end


def read_messages(mcap: bytes):
    """Yield (channel_id, serialized message) from the MCAP stream."""
    if not mcap.startswith(MCAP_MAGIC):
        raise ValueError("The bag member is not an MCAP file")
    yield from _record_messages(mcap[len(MCAP_MAGIC):])


def load_velocities() -> tuple[np.ndarray, np.ndarray]:
    with zipfile.ZipFile(BAG_ZIP) as archive:
        mcap = archive.read(MCAP_MEMBER)

    channel_ids: set[int] = set()
    linear_x: list[float] = []
    angular_z: list[float] = []
    for channel_id, payload in read_messages(mcap):
        channel_ids.add(channel_id)
        vx, wz = CdrReader(payload).twist_stamped()
        if math.isfinite(vx) and math.isfinite(wz):
            linear_x.append(vx)
            angular_z.append(wz)

    if len(channel_ids) != 1:
        raise ValueError(f"Expected one topic channel, found IDs {sorted(channel_ids)}")
    if not linear_x:
        raise ValueError("No finite velocity samples found")
    return np.asarray(linear_x), np.asarray(angular_z)


def main() -> None:
    vx, wz = load_velocities()
    quantities = [
        ("linear_x", "Ground-truth linear velocity $v_x$ (m/s)", vx),
        ("angular_z", "Ground-truth angular velocity $\\omega_z$ (rad/s)", wz),
    ]
    stats = []
    for key, label, values in quantities:
        stats.append({
            "quantity": key,
            "topic": TOPIC,
            "samples": int(values.size),
            "mean": float(np.mean(values)),
            "std_sample_ddof1": float(np.std(values, ddof=1)),
            "std_population_ddof0": float(np.std(values, ddof=0)),
            "minimum": float(np.min(values)),
            "maximum": float(np.max(values)),
        })

        fig, ax = plt.subplots(figsize=(8, 5))
        ax.hist(values, bins="auto", color="#3978a8", edgecolor="white", alpha=0.9)
        ax.axvline(np.mean(values), color="#c43c39", linestyle="--", linewidth=2,
                   label=f"Mean = {np.mean(values):.6g}")
        ax.set(title=f"Nominally zero command: {key.replace('_', ' ')}",
               xlabel=label, ylabel="Sample count")
        ax.grid(axis="y", alpha=0.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(EVIDENCE_DIR / f"zero_cmd_{key}_histogram.png", dpi=180)
        plt.close(fig)

    csv_path = EVIDENCE_DIR / "zero_cmd_velocity_statistics.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=stats[0].keys())
        writer.writeheader()
        writer.writerows(stats)

    report = [
        "Ground-truth velocity statistics for nominally-zero velocity command",
        f"Source: {BAG_ZIP.name}",
        f"Topic: {TOPIC} (geometry_msgs/msg/TwistStamped)",
        f"Finite paired samples: {vx.size}",
        "Standard deviation reported as sample standard deviation (ddof=1).",
        "",
    ]
    for row in stats:
        unit = "m/s" if row["quantity"] == "linear_x" else "rad/s"
        report.extend([
            f"{row['quantity']} ({unit}):",
            f"  Mean: {row['mean']:.10g} {unit}",
            f"  Sample standard deviation: {row['std_sample_ddof1']:.10g} {unit}",
            f"  Population standard deviation: {row['std_population_ddof0']:.10g} {unit}",
            f"  Range: [{row['minimum']:.10g}, {row['maximum']:.10g}] {unit}",
            "",
        ])
    (EVIDENCE_DIR / "zero_cmd_velocity_summary.txt").write_text(
        "\n".join(report), encoding="utf-8")
    print("\n".join(report))


if __name__ == "__main__":
    main()
