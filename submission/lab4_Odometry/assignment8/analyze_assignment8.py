
import csv
import math
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64

bag_path = "assignment8_errors"

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3"
    ),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
)

position_errors = []
angular_errors = []
rows = []

while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic not in (
        "/ekf_error/position",
        "/ekf_error/yaw"
    ):
        continue

    msg = deserialize_message(data, Float64)
    t = timestamp / 1e9

    if topic == "/ekf_error/position":
        position_errors.append(msg.data)
    else:
        angular_errors.append(msg.data)

    rows.append((t, topic, msg.data))

with open("assignment8_errors.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_s", "topic", "error"])
    writer.writerows(rows)


def summarize(values):
    if not values:
        raise ValueError("No samples found")

    return {
        "Mean": statistics.mean(values),
        "Maximum": max(values),
        "RMSE": math.sqrt(
            sum(x*x for x in values) / len(values)
        ),
        "Minimum": min(values),
        "Standard deviation": statistics.pstdev(values)
    }


pos = summarize(position_errors)
ang = summarize(angular_errors)

print("\nASSIGNMENT 8 RESULTS")
print("-" * 65)
print(f"{'Metric':<22}{'Position (m)':>18}{'Angular (deg)':>20}")
print("-" * 65)

for metric in pos:
    print(
        f"{metric:<22}"
        f"{pos[metric]:>18.6f}"
        f"{ang[metric]:>20.6f}"
    )

print("-" * 65)
print(f"Position samples: {len(position_errors)}")
print(f"Angular samples:  {len(angular_errors)}")
