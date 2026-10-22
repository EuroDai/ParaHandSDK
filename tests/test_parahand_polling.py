import threading
import unittest
from unittest.mock import patch

from parahand import JointDefinition, ParaHand


class _Motor:
    connected = True
    timeout_s = 0.05

    def __init__(self):
        self.requested = []
        self.batches = []

    @staticmethod
    def _status(motor_id):
        return {
            "angle": float(motor_id),
            "online": True,
            "fsm_state": 1,
            "error_code": 0,
            "temp": 30.0,
            "bus_voltage": 12.0,
            "received_at_monotonic_s": 123.0 + motor_id,
        }

    def get_motor_status(self, motor_id, *, sync, timeout):
        self.requested.append((motor_id, sync, timeout))
        return self._status(motor_id)

    def get_motor_statuses_sync(self, motor_ids, *, timeout):
        motor_ids = tuple(motor_ids)
        self.batches.append((motor_ids, timeout))
        return {
            motor_id: self._status(motor_id)
            for motor_id in motor_ids
        }


def _make_hand():
    hand = ParaHand.__new__(ParaHand)
    hand.motor = _Motor()
    hand.ctrl_frequency = 50
    hand.joint_to_motor = {
        "first": JointDefinition(1, -90.0, 90.0, False, True),
        "second": JointDefinition(2, -90.0, 90.0, False, True),
    }
    hand.motor_to_joint = {1: "first", 2: "second"}
    hand._feedback_lock = threading.Lock()
    hand._motor_feedback = {}
    hand._poll_failure_counts = {}
    hand._poll_next_allowed_at = {}
    hand._poll_bus_pause_until = 0.0
    hand._poll_cursor = 0
    return hand


class ParaHandPollingTest(unittest.TestCase):
    def test_failed_motor_does_not_pause_healthy_motor_and_can_recover(self):
        hand = _make_hand()
        with patch("parahand.time.monotonic", return_value=10.0):
            hand._store_motor_status(
                1, {"timeout": True, "cached": False, "online": False},
            )
            feedback = hand.poll_once()
        self.assertEqual(set(feedback), {2})
        self.assertEqual(hand._poll_bus_pause_until, 0.0)
        with patch("parahand.time.monotonic", return_value=10.051):
            feedback = hand.poll_once()
        self.assertEqual(set(feedback), {1, 2})
        self.assertNotIn(1, hand._poll_failure_counts)
        self.assertNotIn(1, hand._poll_next_allowed_at)

    def test_repeated_timeout_has_bounded_retry_and_preserves_feedback_age(self):
        hand = _make_hand()
        hand._store_motor_status(1, hand.motor._status(1))
        with patch("parahand.time.monotonic", return_value=200.0):
            for _ in range(1100):
                hand._store_motor_status(
                    1,
                    {"timeout": True, "cached": True, "angle": 1.0},
                )
                self.assertLessEqual(hand._poll_next_allowed_at[1], 200.25)
                self.assertGreaterEqual(hand._poll_next_allowed_at[1], 200.05)
        self.assertEqual(
            hand._motor_feedback[1]["position_updated_at_monotonic_s"], 124.0,
        )

    def test_background_exception_is_logged_and_polling_continues(self):
        hand = _make_hand()
        hand._poll_stop_event = threading.Event()
        hand._poll_error_count = 0
        hand._last_poll_error = None
        hand._last_poll_warning_at = float("-inf")
        calls = []

        def poll():
            calls.append(1)
            if len(calls) == 1:
                raise TimeoutError("send lock busy")
            hand._poll_stop_event.set()

        hand.poll_once = poll
        with self.assertLogs("parahand", level="WARNING") as captured:
            hand._poll_loop()
        self.assertEqual(len(calls), 2)
        self.assertEqual(hand._poll_error_count, 1)
        self.assertIn("send lock busy", captured.output[0])

    def test_poll_next_spends_one_status_transaction_per_tick(self):
        hand = _make_hand()

        first = hand.poll_next()
        second = hand.poll_next()

        self.assertEqual([item[0] for item in hand.motor.requested], [1, 2])
        self.assertEqual(set(first), {1})
        self.assertEqual(set(second), {2})
        self.assertIsNotNone(
            first[1]["position_updated_at_monotonic_s"]
        )

    def test_poll_once_refreshes_every_motor_in_one_pipelined_batch(self):
        hand = _make_hand()

        feedback = hand.poll_once()

        self.assertEqual(hand.motor.requested, [])
        self.assertEqual(hand.motor.batches, [((1, 2), 0.05)])
        self.assertEqual(set(feedback), {1, 2})
        self.assertEqual(
            feedback[1]["position_updated_at_monotonic_s"],
            124.0,
        )


if __name__ == "__main__":
    unittest.main()
