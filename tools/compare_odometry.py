#!/usr/bin/env python3
"""Compare two nav_msgs/Odometry topics in a ROS 2 bag and create plots."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, sosfiltfilt


FIELDS = ("x", "y", "z")


def load_topic(reader, topic, msg_cls):
    rows = []
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        name, raw, bag_ns = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(raw, msg_cls)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        rows.append((stamp, bag_ns * 1e-9, p.x, p.y, p.z, v.x, v.y, v.z))
    a = np.asarray(rows, dtype=float)
    if len(a) < 3:
        raise RuntimeError(f"Not enough messages on {topic}: {len(a)}")
    # Sort and remove duplicate header stamps so interpolation/differentiation is valid.
    a = a[np.argsort(a[:, 0])]
    _, idx = np.unique(a[:, 0], return_index=True)
    return a[np.sort(idx)]


def make_reader(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def resample_lowpass(values, time, cutoff_hz=3.0):
    dt = np.median(np.diff(time))
    fs = 1.0 / dt
    uniform_time = np.arange(time[0], time[-1] + 0.25 * dt, dt)
    uniform_values = np.column_stack(
        [np.interp(uniform_time, time, values[:, i]) for i in range(values.shape[1])]
    )
    cutoff = min(cutoff_hz, 0.4 * fs)
    sos = butter(4, cutoff, btype="low", fs=fs, output="sos")
    return uniform_time, sosfiltfilt(sos, uniform_values, axis=0), fs, cutoff


def style_axes(ax):
    ax.grid(True, alpha=0.28)
    ax.legend(loc="best", fontsize=8)


def component_figure(t1, y1, t2, y2, ylabel, title, output, extra_norm=False):
    count = 4 if extra_norm else 3
    fig, axes = plt.subplots(count, 1, figsize=(13, 2.8 * count), sharex=True)
    for i, axis in enumerate(FIELDS):
        axes[i].plot(t1, y1[:, i], lw=1.2, label=f"/lio_odom {axis}")
        axes[i].plot(t2, y2[:, i], lw=0.8, alpha=0.82, label=f"/lio_odom_hf {axis}")
        axes[i].set_ylabel(f"{axis} [{ylabel}]")
        style_axes(axes[i])
    if extra_norm:
        axes[3].plot(t1, np.linalg.norm(y1, axis=1), lw=1.2, label="/lio_odom norm")
        axes[3].plot(t2, np.linalg.norm(y2, axis=1), lw=0.8, alpha=0.82, label="/lio_odom_hf norm")
        axes[3].set_ylabel(f"norm [{ylabel}]")
        style_axes(axes[3])
    axes[-1].set_xlabel("time from common start [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--output", type=Path, default=Path("odometry_comparison"))
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    msg_cls = get_message("nav_msgs/msg/Odometry")
    low = load_topic(make_reader(args.bag), "/lio_odom", msg_cls)
    high = load_topic(make_reader(args.bag), "/lio_odom_hf", msg_cls)
    common_start = max(low[0, 0], high[0, 0])
    common_end = min(low[-1, 0], high[-1, 0])
    low = low[(low[:, 0] >= common_start) & (low[:, 0] <= common_end)]
    high = high[(high[:, 0] >= common_start) & (high[:, 0] <= common_end)]
    tl, th = low[:, 0] - common_start, high[:, 0] - common_start
    pl, ph = low[:, 2:5], high[:, 2:5]
    vl_raw, vh_raw = low[:, 5:8], high[:, 5:8]
    tvl, vl, fsl, cutl = resample_lowpass(vl_raw, tl)
    tvh, vh, fsh, cuth = resample_lowpass(vh_raw, th)
    al = np.gradient(vl, tvl, axis=0, edge_order=2)
    ah = np.gradient(vh, tvh, axis=0, edge_order=2)

    # Full 3D and top-down position trajectory.
    fig = plt.figure(figsize=(14, 6))
    ax = fig.add_subplot(121, projection="3d")
    ax.plot(*pl.T, lw=1.4, label="/lio_odom")
    ax.plot(*ph.T, lw=0.8, alpha=0.8, label="/lio_odom_hf")
    ax.set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]", title="3D trajectory")
    ax.legend()
    ax2 = fig.add_subplot(122)
    ax2.plot(pl[:, 0], pl[:, 1], lw=1.4, label="/lio_odom")
    ax2.plot(ph[:, 0], ph[:, 1], lw=0.8, alpha=0.8, label="/lio_odom_hf")
    ax2.set(xlabel="x [m]", ylabel="y [m]", title="Top-down trajectory")
    ax2.axis("equal")
    style_axes(ax2)
    fig.tight_layout()
    fig.savefig(args.output / "01_position_trajectory.png", dpi=180)
    plt.close(fig)
    component_figure(tl, pl, th, ph, "m", "Position components (camera_init frame)", args.output / "02_position_vs_time.png")
    component_figure(tvl, vl, tvh, vh, "m/s", "Linear velocity (base_link frame, uniform resampling + 3 Hz low-pass)", args.output / "03_velocity.png", True)
    component_figure(tvl, al, tvh, ah, "m/s²", "Derived linear acceleration (uniform resampling + 3 Hz velocity low-pass)", args.output / "04_acceleration.png", True)

    # Align high-rate series to low-rate stamps for difference metrics and plot.
    def interp(target_t, source_t, arr):
        return np.column_stack([np.interp(target_t, source_t, arr[:, i]) for i in range(3)])
    ph_i = interp(tl, th, ph)
    vh_i, ah_i = interp(tvl, tvh, vh), interp(tvl, tvh, ah)
    pl_v = interp(tvl, tl, pl)
    ph_v = interp(tvl, th, ph)
    dp, dv, da = pl - ph_i, vl - vh_i, al - ah_i
    component_figure(tl, dp, tl, np.zeros_like(dp), "m", "Position difference: /lio_odom - interpolated /lio_odom_hf", args.output / "05_position_difference.png")

    def metrics(delta):
        norm = np.linalg.norm(delta, axis=1)
        return {
            "component_rmse": dict(zip(FIELDS, np.sqrt(np.mean(delta ** 2, axis=0)).tolist())),
            "norm_mean": float(np.mean(norm)),
            "norm_rmse": float(np.sqrt(np.mean(norm ** 2))),
            "norm_p95": float(np.percentile(norm, 95)),
            "norm_max": float(np.max(norm)),
        }
    summary = {
        "bag": str(args.bag), "common_duration_s": float(common_end - common_start),
        "lio_odom": {"samples": len(low), "median_rate_hz": fsl},
        "lio_odom_hf": {"samples": len(high), "median_rate_hz": fsh},
        "filter": {"type": "4th-order zero-phase Butterworth", "requested_cutoff_hz": 3.0,
                   "effective_low_cutoff_hz": cutl, "effective_high_cutoff_hz": cuth},
        "position_difference_m": metrics(dp), "velocity_difference_mps": metrics(dv),
        "acceleration_difference_mps2": metrics(da),
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    with (args.output / "aligned_differences.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s"] + [f"position_d{q}_m" for q in FIELDS] +
                        [f"velocity_d{q}_mps" for q in FIELDS] + [f"acceleration_d{q}_mps2" for q in FIELDS])
        # Velocity/acceleration differences use the uniformly resampled low-rate grid.
        position_delta_on_velocity_grid = pl_v - ph_v
        writer.writerows(np.column_stack((tvl, position_delta_on_velocity_grid, dv, da)))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
