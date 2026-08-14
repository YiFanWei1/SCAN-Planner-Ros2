#!/usr/bin/env python3
"""Analyze controller replay telemetry and pidstat resource logs."""

import argparse
import json
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def percentile(values, q):
    return float(np.percentile(values, q)) if len(values) else 0.0


def load_telemetry(path):
    records = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("schema") == "scan_planner.telemetry.v1":
                records.append(record)
    odom = [r for r in records if r["record_type"] == "odom_velocity"]
    if not odom:
        raise RuntimeError(f"no odometry in {path}")
    start = min(r["timestamp_ns"] for r in odom)
    end = max(r["timestamp_ns"] for r in odom)
    selected = [r for r in records if start <= r["timestamp_ns"] <= end]
    return selected, start, end


def command_array(records, kind, start):
    rows = [r for r in records if r["record_type"] == kind]
    rows.sort(key=lambda r: r["timestamp_ns"])
    if not rows:
        return np.empty((0, 7))
    return np.asarray([
        [(r["timestamp_ns"] - start) * 1e-9,
         r["vx"], r["vy"], r["vz"], r["wx"], r["wy"], r["wz"]]
        for r in rows], dtype=float)


def command_metrics(raw, safe, duration, limits):
    if len(raw) < 2:
        return {}
    linear = np.hypot(raw[:, 1], raw[:, 2])
    dt = np.diff(raw[:, 0])
    delta = np.diff(raw[:, [1, 2, 6]], axis=0)
    valid = (dt >= 0.005) & (dt <= 0.2)
    accel = np.linalg.norm(delta[valid, :2], axis=1) / dt[valid]
    yaw_accel = np.abs(delta[valid, 2]) / dt[valid]
    stopped = (linear < 0.01) & (np.abs(raw[:, 6]) < 0.01)
    saturated = ((np.abs(raw[:, 1]) >= limits[0] * 0.99) |
                 (np.abs(raw[:, 2]) >= limits[1] * 0.99) |
                 (np.abs(raw[:, 6]) >= limits[2] * 0.99))

    gate_zero = np.zeros(len(safe), dtype=bool)
    gate_changed = np.zeros(len(safe), dtype=bool)
    if len(safe):
        index = np.searchsorted(raw[:, 0], safe[:, 0], side="right") - 1
        valid_safe = index >= 0
        index = np.maximum(index, 0)
        raw_near = raw[index][:, [1, 2, 6]]
        safe_values = safe[:, [1, 2, 6]]
        raw_active = np.linalg.norm(raw_near, axis=1) >= 0.01
        safe_zero = np.linalg.norm(safe_values, axis=1) < 0.01
        gate_zero = valid_safe & raw_active & safe_zero
        gate_changed = valid_safe & (np.linalg.norm(raw_near - safe_values, axis=1) > 0.02)

    return {
        "duration_s": duration,
        "raw_samples": int(len(raw)),
        "safe_samples": int(len(safe)),
        "raw_rate_hz": float((len(raw) - 1) / max(raw[-1, 0] - raw[0, 0], 1e-9)),
        "linear_speed_mean": float(np.mean(linear)),
        "linear_speed_rms": float(np.sqrt(np.mean(linear ** 2))),
        "linear_speed_p95": percentile(linear, 95),
        "linear_speed_max": float(np.max(linear)),
        "abs_yaw_rate_mean": float(np.mean(np.abs(raw[:, 6]))),
        "abs_yaw_rate_p95": percentile(np.abs(raw[:, 6]), 95),
        "abs_yaw_rate_max": float(np.max(np.abs(raw[:, 6]))),
        "linear_accel_p95": percentile(accel, 95),
        "linear_accel_max": float(np.max(accel)) if len(accel) else 0.0,
        "yaw_accel_p95": percentile(yaw_accel, 95),
        "yaw_accel_max": float(np.max(yaw_accel)) if len(yaw_accel) else 0.0,
        "stop_fraction": float(np.mean(stopped)),
        "saturation_fraction": float(np.mean(saturated)),
        "gate_forced_zero_fraction": float(np.mean(gate_zero)) if len(safe) else 0.0,
        "gate_changed_fraction": float(np.mean(gate_changed)) if len(safe) else 0.0,
        "command_total_variation": float(np.sum(np.linalg.norm(delta, axis=1))),
        "command_variation_per_second": float(
            np.sum(np.linalg.norm(delta, axis=1)) / max(duration, 1e-9)),
        "command_mean_step_change": float(np.mean(np.linalg.norm(delta, axis=1))),
    }


def launch_pids(path):
    mapping = {}
    pattern = re.compile(r"\[([^\]]+)-\d+\]: process started with pid \[(\d+)\]")
    text = path.read_text(encoding="utf-8", errors="replace")
    for name, pid in pattern.findall(text):
        mapping[int(pid)] = name
    return mapping, text


def load_pidstat(path, pid_names):
    rows = []
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 19 or fields[0].startswith("#") or fields[0] == "Linux":
                continue
            try:
                pid = int(fields[2])
                cpu = float(fields[7])
                rss_kb = float(fields[12])
                read_kbs = float(fields[14])
                write_kbs = float(fields[15])
            except (ValueError, IndexError):
                continue
            if pid in pid_names:
                rows.append({"time": fields[0], "pid": pid, "name": pid_names[pid],
                             "cpu": cpu, "rss_mb": rss_kb / 1024.0,
                             "read_kbs": read_kbs, "write_kbs": write_kbs})
    return rows


def resource_metrics(rows, controller_name):
    result = {"processes": {}}
    for name in sorted(set(r["name"] for r in rows)):
        selected = [r for r in rows if r["name"] == name]
        cpu = np.asarray([r["cpu"] for r in selected])
        rss = np.asarray([r["rss_mb"] for r in selected])
        result["processes"][name] = {
            "samples": len(selected), "cpu_mean": float(np.mean(cpu)),
            "cpu_p95": percentile(cpu, 95), "cpu_max": float(np.max(cpu)),
            "rss_mean_mb": float(np.mean(rss)), "rss_max_mb": float(np.max(rss)),
        }
    stack_names = {"scan_planner_node", controller_name, "cmd_vel_safety_gate",
                   "real_go2_input_adapter", "terrain_path_visualizer"}
    by_time = {}
    for row in rows:
        if row["name"] in stack_names:
            entry = by_time.setdefault(row["time"], [0.0, 0.0])
            entry[0] += row["cpu"]
            entry[1] += row["rss_mb"]
    stack = np.asarray(list(by_time.values())) if by_time else np.empty((0, 2))
    result["stack"] = {
        "samples": len(stack),
        "cpu_mean": float(np.mean(stack[:, 0])) if len(stack) else 0.0,
        "cpu_p95": percentile(stack[:, 0], 95) if len(stack) else 0.0,
        "cpu_max": float(np.max(stack[:, 0])) if len(stack) else 0.0,
        "rss_mean_mb": float(np.mean(stack[:, 1])) if len(stack) else 0.0,
        "rss_max_mb": float(np.max(stack[:, 1])) if len(stack) else 0.0,
    }
    return result


def event_metrics(text, is_mppi):
    result = {
        "local_replan_failed": len(re.findall(r"Local replan failed", text)),
        "trajectory_rejected": len(re.findall(r"Trajectory rejected", text)),
        "astar_failed_summaries": len(re.findall(r"AStarSummary.*plan=failed", text)),
    }
    if is_mppi:
        states = re.findall(r"MPPI state: ([^\s]+)", text)
        result["mppi_state_transitions"] = {s: states.count(s) for s in sorted(set(states))}
    return result


def time_series(rows, process_name):
    selected = [r for r in rows if r["name"] == process_name]
    return (np.arange(len(selected)), np.asarray([r["cpu"] for r in selected]),
            np.asarray([r["rss_mb"] for r in selected]))


def plot(results, arrays, resource_rows, output):
    colors = {"closed_loop": "#2878B5", "mppi": "#D95319"}
    fig, axes = plt.subplots(3, 2, figsize=(15, 12), constrained_layout=True)
    for name in ("closed_loop", "mppi"):
        raw = arrays[name]
        stride = max(1, len(raw) // 4000)
        axes[0, 0].plot(raw[::stride, 0], np.hypot(raw[::stride, 1], raw[::stride, 2]),
                        color=colors[name], linewidth=0.8, label=name)
        axes[0, 1].plot(raw[::stride, 0], raw[::stride, 6], color=colors[name],
                        linewidth=0.8, label=name)
    axes[0, 0].set(title="Commanded planar speed", ylabel="m/s", xlabel="Replay time (s)")
    axes[0, 1].set(title="Commanded yaw rate", ylabel="rad/s", xlabel="Replay time (s)")
    axes[0, 0].legend(); axes[0, 1].legend()

    metric_names = ["linear_speed_rms", "linear_accel_p95", "stop_fraction",
                    "command_total_variation"]
    labels = ["Speed RMS\n(m/s)", "Accel p95\n(m/s²)", "Stop fraction", "Total variation"]
    x = np.arange(len(labels)); width = 0.36
    for index, name in enumerate(("closed_loop", "mppi")):
        values = [results[name]["commands"][key] for key in metric_names]
        axes[1, 0].bar(x + (index - 0.5) * width, values, width,
                       label=name, color=colors[name])
    axes[1, 0].set_xticks(x, labels); axes[1, 0].set_title("Command quality metrics")
    axes[1, 0].legend()

    controller_names = {"closed_loop": "closed_loop_controller",
                        "mppi": "scan_mppi_controller_node"}
    for name in ("closed_loop", "mppi"):
        t, cpu, rss = time_series(resource_rows[name], controller_names[name])
        axes[1, 1].plot(t, cpu, color=colors[name], linewidth=1.0, label=name)
        axes[2, 0].plot(t, rss, color=colors[name], linewidth=1.0, label=name)
    axes[1, 1].set(title="Controller process CPU", ylabel="% of one CPU core", xlabel="Sample (1 Hz)")
    axes[2, 0].set(title="Controller process RSS", ylabel="MiB", xlabel="Sample (1 Hz)")
    axes[1, 1].legend(); axes[2, 0].legend()

    stack_cpu = [results[n]["resources"]["stack"]["cpu_mean"] for n in ("closed_loop", "mppi")]
    stack_rss = [results[n]["resources"]["stack"]["rss_mean_mb"] for n in ("closed_loop", "mppi")]
    axis = axes[2, 1]
    axis.bar([0, 1], stack_cpu, color=[colors["closed_loop"], colors["mppi"]])
    axis.set_xticks([0, 1], ["closed_loop", "mppi"])
    axis.set_ylabel("Mean CPU (% one core)")
    axis.set_title("Navigation stack resource cost")
    axis2 = axis.twinx()
    axis2.plot([0, 1], stack_rss, color="black", marker="o", linewidth=2)
    axis2.set_ylabel("Mean RSS sum (MiB)")
    fig.suptitle("Closed-loop controller vs MPPI — identical rosbag replay", fontsize=16)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("benchmark_results"))
    args = parser.parse_args()
    configs = {
        # real_go2_livox.yaml overrides controllers.yaml at launch time.
        "closed_loop": ("closed_loop_controller", (0.50, 0.50, 0.80)),
        "mppi": ("scan_mppi_controller_node", (0.50, 0.35, 0.80)),
    }
    results, arrays, rows_by_run = {}, {}, {}
    for name, (controller, limits) in configs.items():
        directory = args.root / name
        telemetry = directory / "telemetry_resource_run.jsonl"
        records, start, end = load_telemetry(telemetry)
        raw = command_array(records, "cmd_vel_raw", start)
        safe = command_array(records, "cmd_vel_safe", start)
        pids, launch_text = launch_pids(directory / "launch_resource_run.log")
        rows = load_pidstat(directory / "resources_all.txt", pids)
        results[name] = {
            "commands": command_metrics(raw, safe, (end - start) * 1e-9, limits),
            "resources": resource_metrics(rows, controller),
            "events": event_metrics(launch_text, name == "mppi"),
            "odometry_samples": sum(r["record_type"] == "odom_velocity" for r in records),
        }
        arrays[name] = raw
        rows_by_run[name] = rows
    args.root.mkdir(parents=True, exist_ok=True)
    with (args.root / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(results, stream, indent=2, sort_keys=True)
    plot(results, arrays, rows_by_run, args.root / "controller_comparison.png")
    print(json.dumps(results, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
