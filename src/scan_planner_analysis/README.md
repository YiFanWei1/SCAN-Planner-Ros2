# SCAN-Planner Analysis

The recorder writes one JSON object per line using schema
`scan_planner.telemetry.v1`. It records safe commands, raw commands and
measured odometry velocity without participating in the control chain.

The real Go2 launch starts the recorder by default. Its output path is printed
at startup and defaults to the package's `output` directory:

```text
~/github_code/SCAN-Planner-Ros2/src/scan_planner_analysis/output/velocity_YYYYmmdd_HHMMSS.jsonl
```

Generate a PNG after stopping the run:

```bash
ros2 run scan_planner_analysis plot_velocity \
  --input ~/github_code/SCAN-Planner-Ros2/src/scan_planner_analysis/output/velocity_YYYYmmdd_HHMMSS.jsonl \
  --output ~/github_code/SCAN-Planner-Ros2/src/scan_planner_analysis/output/velocity_curve.png
```

Disable recording with `analysis:=false`, or select a fixed path with
`analysis_output:=/tmp/run.jsonl` on `real_go2_livox.launch.py`. Override the
default directory with `analysis_output_directory:=/path/to/output`.
# Independent high-frequency odometry timing probe

Run this node without SCAN to compare the real callback interval with the
source `header.stamp` interval:

```bash
ros2 run scan_planner_analysis hf_odom_probe --ros-args \
  -p topic:=/lio_odom_hf \
  -p reliability:=best_effort \
  -p qos_depth:=1 \
  -p gap_threshold:=0.15 \
  -p header_gap_threshold:=0.03 \
  -p output_file:=/tmp/hf_odom_probe.jsonl
```

`CALLBACK_STALL_BEGIN` is printed as soon as no callback has arrived for the
configured duration. After reception resumes, `CALLBACK_STALL_CONFIRMED` means
the callback interval was abnormal while the source header interval remained
normal. `SOURCE_TIME_GAP` means the source timestamp itself also jumped.
