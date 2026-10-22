import unittest
from unittest.mock import patch

import motor


class _Clock:
    now = 0.0

    def monotonic(self):
        return self.now


class _DelayedLock:
    def __init__(self, clock, delay):
        self.clock = clock
        self.delay = delay
        self.released = False

    def acquire(self, *, timeout):
        self.clock.now += min(self.delay, timeout)
        return self.delay <= timeout

    def release(self):
        self.released = True


class _Replies:
    def __init__(self, transport, clock):
        self.transport = transport
        self.clock = clock
        self.pending = []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def wait(self, seconds):
        until = self.clock.now + seconds
        due = [item for item in self.pending if item[0] <= until]
        self.clock.now = max(
            self.clock.now, min((item[0] for item in due), default=until),
        )
        for received_at, motor_id in due:
            if received_at > self.clock.now:
                continue
            self.transport._status_response_generation[motor_id] = 1
            self.transport._latest_status_response[motor_id] = {
                "angle": float(motor_id),
                "online": True,
                "received_at_monotonic_s": received_at,
            }
            self.pending.remove((received_at, motor_id))


class MotorTimingTest(unittest.TestCase):
    def make_transport(self, *, lock_delay=0.06, send_delay=0.0):
        clock = _Clock()
        transport = motor.Motor()
        transport.connected = True
        transport._status_transaction_lock = _DelayedLock(clock, lock_delay)
        replies = _Replies(transport, clock)
        transport._status_condition = replies

        def send(frame, **kwargs):
            clock.now += send_delay
            replies.pending.append((clock.now + 0.01, frame[8]))

        transport._send_raw = send
        return transport, clock, replies

    def test_send_lock_wait_does_not_consume_response_budget(self):
        transport, clock, _ = self.make_transport()
        with patch.object(motor.time, "monotonic", clock.monotonic):
            result = transport.get_motor_statuses_sync([3], timeout=0.05)
        self.assertFalse(result[3]["timeout"])
        self.assertAlmostEqual(clock.now, 0.07)
        self.assertTrue(transport._status_transaction_lock.released)

    def test_last_query_gets_full_budget_after_slow_batch_writes(self):
        transport, clock, _ = self.make_transport(
            lock_delay=0.0, send_delay=0.03,
        )
        with patch.object(motor.time, "monotonic", clock.monotonic):
            result = transport.get_motor_statuses_sync([1, 2, 3], timeout=0.05)
        self.assertTrue(all(not status["timeout"] for status in result.values()))

    def test_missing_reply_remains_timeout_after_lock_contention(self):
        transport, clock, _ = self.make_transport()
        transport._send_raw = lambda *args, **kwargs: None
        with patch.object(motor.time, "monotonic", clock.monotonic):
            result = transport.get_motor_statuses_sync([3], timeout=0.05)
        self.assertTrue(result[3]["timeout"])
        self.assertFalse(result[3]["cached"])
        self.assertAlmostEqual(clock.now, 0.11)

    def test_lock_starvation_is_explicit_and_does_not_send(self):
        transport, clock, replies = self.make_transport(lock_delay=1.0)
        with patch.object(motor.time, "monotonic", clock.monotonic):
            with self.assertRaisesRegex(TimeoutError, "send lock busy"):
                transport.get_motor_statuses_sync([3], timeout=0.05)
        self.assertEqual(replies.pending, [])
        self.assertEqual(transport.get_queue_status()["status_lock_timeout_count"], 1)
        self.assertTrue(transport._status_batch_lock.acquire(blocking=False))
        transport._status_batch_lock.release()

    def test_late_reply_cannot_satisfy_an_earlier_query(self):
        transport, clock, replies = self.make_transport(lock_delay=0.0)

        def send(frame, **kwargs):
            motor_id = frame[8]
            replies.pending.append((0.06 if motor_id == 1 else 0.07, motor_id))
            if motor_id == 2:
                clock.now += 0.06

        transport._send_raw = send
        with patch.object(motor.time, "monotonic", clock.monotonic):
            result = transport.get_motor_statuses_sync([1, 2], timeout=0.05)
        self.assertTrue(result[1]["timeout"])
        self.assertFalse(result[2]["timeout"])


if __name__ == "__main__":
    unittest.main()
