import json

from scan_planner_analysis.log_format import encode_record, read_records


def test_round_trip_jsonl(tmp_path):
    path = tmp_path / "telemetry.jsonl"
    path.write_text(
        encode_record("cmd_vel_safe", 123, "/cmd", vx=0.2, vy=0.0, wz=0.1)
        + "\n", encoding="utf-8")
    records = read_records(path)
    assert records[0]["timestamp_ns"] == 123
    assert records[0]["vx"] == 0.2


def test_unrelated_schema_is_ignored(tmp_path):
    path = tmp_path / "telemetry.jsonl"
    path.write_text(json.dumps({"schema": "other"}) + "\n", encoding="utf-8")
    assert read_records(path) == []
