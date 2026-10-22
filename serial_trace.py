"""Opt-in serial evidence recorder. Never opens a serial port or sends commands."""

import argparse
import json
import os
import queue
import threading
import time
import uuid
from collections import Counter
from pathlib import Path


class SerialTrace:
    def __init__(self, directory, *, label="", capacity=32768):
        directory = Path(directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        name = f"serial-{time.time_ns()}-{os.getpid()}-{uuid.uuid4().hex[:8]}.jsonl"
        self.path = directory / name
        self._file = self.path.open("x", encoding="utf-8")
        self._queue = queue.Queue(maxsize=capacity)
        self._stop = threading.Event()
        self.dropped = 0
        self.error = None
        self._thread = threading.Thread(target=self._write, daemon=True)
        self._thread.start()
        self.emit("metadata", label=label, pid=os.getpid(), wall_time_ns=time.time_ns())
        print(f"[serial-trace] {self.path}", flush=True)

    def emit(self, event, **fields):
        if self._stop.is_set() or self.error is not None:
            return
        record = {"event": event, "monotonic_ns": time.monotonic_ns(), **fields}
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        size = 0
        last_flush = time.monotonic()
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    record = self._queue.get(timeout=0.1)
                except queue.Empty:
                    self._file.flush()
                    continue
                line = json.dumps(record, ensure_ascii=True) + "\n"
                size += len(line)
                if size > 512 * 1024 * 1024:
                    raise RuntimeError("trace exceeds 512 MiB; start a new capture")
                self._file.write(line)
                if time.monotonic() - last_flush >= 0.5:
                    self._file.flush()
                    last_flush = time.monotonic()
        except Exception as exc:
            self.error = repr(exc)
            print(f"[serial-trace] recording failed: {self.error}", flush=True)
        finally:
            try:
                self._file.write(json.dumps({
                    "event": "trace_end", "monotonic_ns": time.monotonic_ns(),
                    "dropped": self.dropped, "error": self.error,
                }) + "\n")
            finally:
                self._file.close()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive() or self.dropped or self.error:
            print(
                f"[serial-trace] incomplete: dropped={self.dropped}, "
                f"error={self.error}, writer_alive={self._thread.is_alive()}",
                flush=True,
            )


def summarize(path, motor_id=3):
    counts = Counter()
    statuses = []
    timeouts = []
    end = None
    label = None
    with Path(path).open(encoding="utf-8") as log:
        for line in log:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                counts["invalid_lines"] += 1
                continue
            event = row["event"]
            counts[event] += 1
            if event == "metadata":
                label = row.get("label")
            if event == "trace_end":
                end = row
            if event == "tx_written":
                frame = bytes.fromhex(row["hex"])
                if len(frame) == 20 and frame[:4] == b"\x55\xaa\x00\x14":
                    can_id = int.from_bytes(frame[4:8], "little")
                    if can_id == 0xA0 and frame[8] == motor_id:
                        counts["motor_queries_written"] += 1
                    elif can_id != 0xA0:
                        counts["non_query_writes"] += 1
                else:
                    counts["non_query_writes"] += 1
            if event == "rx_frame" and not row["crc_valid"]:
                counts["crc_rejections_all_motors"] += 1
            if event == "status" and row["motor_id"] == motor_id:
                statuses.append(row)
            if event == "query_result" and row["motor_id"] == motor_id:
                counts["motor_query_timeouts" if row["timeout"] else
                       "motor_query_successes"] += 1
                if row["timeout"]:
                    timeouts.append(row["monotonic_ns"])
    stamps = sorted(row["monotonic_ns"] for row in statuses)
    gaps = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
    return {
        "path": str(path), "label": label, "motor_id": motor_id,
        "counts": dict(counts), "parsed_statuses": len(statuses),
        "max_status_gap_ms": max(gaps, default=None),
        "first_timeout_monotonic_ns": next(iter(timeouts), None),
        "angle_range_deg": (
            [min(r["angle"] for r in statuses), max(r["angle"] for r in statuses)]
            if statuses else None
        ),
        "capture_complete": bool(
            end and not end["dropped"] and not end["error"]
            and not counts["invalid_lines"]
        ),
        "trace_end": end,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--motor", type=int, default=3)
    args = parser.parse_args()
    print(json.dumps([summarize(p, args.motor) for p in args.paths], indent=2))
