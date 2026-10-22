import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import motor
from serial_trace import SerialTrace, summarize


class SerialTraceTest(unittest.TestCase):
    def test_raw_write_and_crc_rejection_are_recorded_without_changing_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            transport = motor.Motor()
            transport.connected = True
            transport.serial = Mock()
            transport.serial.write.return_value = 20
            transport._trace = SerialTrace(directory, label="test")
            request = transport._build_frame(0xA0, bytes([3, 1]) + b"\0" * 6)
            transport._send_raw(request, post_write_delay_s=0)
            good = transport._build_frame(3 << 8, b"\0" * 8)
            bad = bytearray(good)
            bad[12] ^= 1
            transport.receive_buffer.extend(bytes(bad) + good)
            transport._process_receive_buffer()
            transport._trace.close()
            transport.serial.write.assert_called_once_with(request)
            result = summarize(transport._trace.path)
            self.assertTrue(result["capture_complete"])
            self.assertEqual(result["counts"]["motor_queries_written"], 1)
            self.assertEqual(result["counts"]["crc_rejections_all_motors"], 1)
            self.assertEqual(result["parsed_statuses"], 1)
            self.assertEqual(result["label"], "test")

    def test_incomplete_capture_cannot_claim_complete_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "partial.jsonl"
            path.write_text('{"event":"metadata"}\n{"event":', encoding="utf-8")
            result = summarize(path)
            self.assertFalse(result["capture_complete"])
            self.assertEqual(result["counts"]["invalid_lines"], 1)

    def test_dropped_records_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            trace = SerialTrace(directory)
            trace.dropped = 1
            trace.close()
            self.assertFalse(summarize(trace.path)["capture_complete"])

    def test_disabled_trace_does_not_create_a_writer(self):
        transport = motor.Motor()
        self.assertIsNone(transport._trace)
        transport._trace_event("test")


if __name__ == "__main__":
    unittest.main()
