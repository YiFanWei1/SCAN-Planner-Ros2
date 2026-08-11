from pathlib import Path

from scan_planner_analysis.telemetry_recorder import resolve_output_path


def test_explicit_output_file_takes_precedence(tmp_path):
    output = tmp_path / "fixed.jsonl"
    assert resolve_output_path(str(output), "/ignored", "stamp") == str(output)


def test_output_directory_gets_timestamped_filename(tmp_path):
    output = resolve_output_path("", str(tmp_path), "20260811_102000")
    assert Path(output) == tmp_path / "velocity_20260811_102000.jsonl"
