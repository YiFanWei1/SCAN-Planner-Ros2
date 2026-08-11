# SCAN-Planner Analysis

The recorder writes one JSON object per line using schema
`scan_planner.telemetry.v1`. It records safe commands, raw commands and
measured odometry velocity without participating in the control chain.

The real Go2 launch starts the recorder by default. Its output path is printed
at startup and defaults to:

```text
~/.ros/scan_planner_analysis/velocity_YYYYmmdd_HHMMSS.jsonl
```

Generate a PNG after stopping the run:

```bash
ros2 run scan_planner_analysis plot_velocity \
  --input ~/.ros/scan_planner_analysis/velocity_YYYYmmdd_HHMMSS.jsonl \
  --output ~/velocity_curve.png
```

Disable recording with `analysis:=false`, or select a fixed path with
`analysis_output:=/tmp/run.jsonl` on `real_go2_livox.launch.py`.
