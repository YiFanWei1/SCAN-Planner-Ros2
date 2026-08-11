"""Generate velocity-curve PNG files from SCAN-Planner JSONL telemetry."""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .log_format import read_records


LABELS = {
    "cmd_vel_safe": "safe command",
    "cmd_vel_raw": "raw command",
    "odom_velocity": "measured odometry",
}


def plot_velocity(input_path, output_path):
    records = [
        record for record in read_records(input_path)
        if record["record_type"] in LABELS]
    if not records:
        raise ValueError("log contains no velocity records")
    start_ns = min(int(record["timestamp_ns"]) for record in records)

    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    components = (("vx", "vx [m/s]"), ("vy", "vy [m/s]"),
                  ("wz", "yaw rate [rad/s]"))
    for record_type, label in LABELS.items():
        selected = [r for r in records if r["record_type"] == record_type]
        if not selected:
            continue
        times = [(int(r["timestamp_ns"]) - start_ns) * 1e-9 for r in selected]
        for axis, (field, ylabel) in zip(axes, components):
            axis.plot(times, [float(r.get(field, 0.0)) for r in selected],
                      label=label, linewidth=1.0)
            axis.set_ylabel(ylabel)
            axis.grid(True, alpha=0.3)
    axes[0].set_title("SCAN-Planner velocity telemetry")
    axes[-1].set_xlabel("time [s]")
    for axis in axes:
        if axis.lines:
            axis.legend(loc="upper right")
    figure.tight_layout()
    output_path = os.path.abspath(os.path.expanduser(output_path))
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return output_path


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Plot SCAN-Planner velocity JSONL telemetry")
    parser.add_argument("--input", required=True, help="input .jsonl file")
    parser.add_argument("--output", default="velocity_curve.png",
                        help="output PNG path")
    parsed = parser.parse_args(args)
    output = plot_velocity(parsed.input, parsed.output)
    print(output)
