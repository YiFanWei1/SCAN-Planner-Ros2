"""Stable JSONL telemetry schema shared by the recorder and analysis tools."""

import json


SCHEMA = "scan_planner.telemetry.v1"


def encode_record(record_type, timestamp_ns, source_topic, **values):
    """Return one compact, deterministic JSONL record."""
    record = {
        "schema": SCHEMA,
        "timestamp_ns": int(timestamp_ns),
        "record_type": str(record_type),
        "source_topic": str(source_topic),
    }
    record.update(values)
    return json.dumps(record, sort_keys=True, separators=(",", ":"))


def read_records(path):
    """Read valid telemetry records, ignoring blank and unrelated lines."""
    records = []
    with open(path, "r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(
                    f"invalid JSON at line {line_number}: {error.msg}") from error
            if record.get("schema") != SCHEMA:
                continue
            if "timestamp_ns" not in record or "record_type" not in record:
                raise ValueError(f"incomplete telemetry record at line {line_number}")
            records.append(record)
    return records
