import threading
import time
import unittest

import motor


class _ImmediateStatusMotor(motor.Motor):
    def __init__(self):
        super().__init__()
        self.connected = True
        self.sent = []

    def _send_raw(self, frame, *, post_write_delay_s=0.003):
        motor_id = frame[8]
        self.sent.append((motor_id, post_write_delay_s))
        self._record_status_response(
            motor.Response(
                type=motor.ResponseType.STATUS_RESPONSE,
                data={
                    "motor_id": motor_id,
                    "angle": float(motor_id),
                    "online": True,
                },
                timestamp=time.time(),
                raw_data=b"",
            )
        )


class _NoResponseStatusMotor(motor.Motor):
    def __init__(self):
        super().__init__()
        self.connected = True
        self.query_frames_sent = threading.Event()
        self.sent_count = 0

    def _send_raw(self, frame, *, post_write_delay_s=0.003):
        self.sent_count += 1
        if self.sent_count == 3:
            self.query_frames_sent.set()


class MotorBackpressureTest(unittest.TestCase):
    def test_status_batch_pipelines_requests_and_matches_responses(self):
        transport = _ImmediateStatusMotor()

        statuses = transport.get_motor_statuses_sync((1, 8, 16))

        self.assertEqual(
            transport.sent,
            [
                (1, transport.STATUS_QUERY_INTERVAL_S),
                (8, transport.STATUS_QUERY_INTERVAL_S),
                (16, transport.STATUS_QUERY_INTERVAL_S),
            ],
        )
        self.assertEqual(set(statuses), {1, 8, 16})
        self.assertTrue(all(not item["timeout"] for item in statuses.values()))
        self.assertEqual(statuses[16]["angle"], 16.0)

    def test_status_batch_releases_send_lock_while_waiting_for_replies(self):
        transport = _NoResponseStatusMotor()
        finished = threading.Event()

        def read_batch():
            transport.get_motor_statuses_sync((1, 2, 3), timeout=0.2)
            finished.set()

        worker = threading.Thread(target=read_batch)
        worker.start()
        self.assertTrue(transport.query_frames_sent.wait(0.1))

        acquired = transport._status_transaction_lock.acquire(timeout=0.05)
        try:
            self.assertTrue(acquired)
            self.assertFalse(finished.is_set())
        finally:
            if acquired:
                transport._status_transaction_lock.release()
        worker.join(timeout=0.5)
        self.assertFalse(worker.is_alive())

    def test_response_queue_drops_oldest_when_full(self):
        transport = motor.Motor(response_queue_size=2)

        for value in range(5):
            transport._enqueue_response(
                motor.Response(
                    type=motor.ResponseType.ANGLE_BROADCAST,
                    data={"value": value},
                    timestamp=time.time(),
                    raw_data=b"",
                )
            )

        self.assertEqual(transport.response_queue.qsize(), 2)
        self.assertEqual(transport.get_response().data["value"], 3)
        self.assertEqual(transport.get_response().data["value"], 4)
        self.assertEqual(
            transport.get_queue_status()["dropped_response_count"],
            3,
        )

    def test_per_motor_position_commands_keep_only_latest_target(self):
        transport = motor.Motor(command_queue_size=4)
        transport.connected = True

        for angle in range(100):
            transport.set_motor_angle(3, float(angle))

        self.assertEqual(transport.command_queue.qsize(), 1)
        token = transport.command_queue.get_nowait()
        latest = transport._resolve_latest_position_command(token)
        transport.command_queue.task_done()
        self.assertEqual(latest.data["angle"], 99.0)
        self.assertEqual(
            transport.get_queue_status()["coalesced_motor_angle_count"],
            99,
        )

    def test_whole_hand_commands_keep_only_latest_target(self):
        transport = motor.Motor(command_queue_size=4)
        transport.connected = True

        for angle in range(100):
            transport.set_motor_angles_broadcast({1: float(angle)})

        self.assertEqual(transport.command_queue.qsize(), 1)
        token = transport.command_queue.get_nowait()
        latest = transport._resolve_latest_position_command(token)
        transport.command_queue.task_done()
        self.assertEqual(latest.data["angles"], {1: 99.0})
        self.assertEqual(
            transport.get_queue_status()["coalesced_angle_broadcast_count"],
            99,
        )

    def test_event_commands_fail_when_bounded_queue_is_full(self):
        transport = motor.Motor(
            command_queue_size=1,
            write_timeout_s=0.001,
        )
        transport.connected = True
        transport.enable_motor(1)

        with self.assertRaisesRegex(RuntimeError, "command queue is full"):
            transport.enable_motor(2)

        self.assertEqual(transport.command_queue.qsize(), 1)

    def test_wait_for_commands_times_out_on_stalled_transport(self):
        transport = motor.Motor(
            command_queue_size=1,
            write_timeout_s=0.001,
        )
        transport.connected = True
        transport.enable_motor(1)

        with self.assertRaisesRegex(TimeoutError, "queue to drain"):
            transport.wait_for_commands(0.001)


if __name__ == "__main__":
    unittest.main()
