import unittest

import motor


class MotorStreamTest(unittest.TestCase):
    def setUp(self):
        self.transport = motor.Motor()

    def status(self, motor_id, payload=b"\0" * 8):
        return self.transport._build_frame(motor_id << 8, payload)

    def broadcast(self, header, payload=b"\0" * 16):
        crc = self.transport._calculate_crc16_ccitt_false(payload)
        return header + payload + crc.to_bytes(2, "little")

    def feed(self, data):
        self.transport.receive_buffer.extend(data)
        self.transport._process_receive_buffer()

    def test_noise_between_status_frames_does_not_stall_receiver(self):
        self.feed(self.status(1) + b"\xff" + self.status(2))
        self.assertEqual(set(self.transport._latest_status_response), {1, 2})
        self.assertEqual(len(self.transport.receive_buffer), 0)

    def test_partial_headers_and_frames_survive_each_possible_split(self):
        frame = self.status(9)
        for split in range(1, len(frame)):
            with self.subTest(split=split):
                self.transport = motor.Motor()
                self.feed(b"noise" + frame[:split])
                self.assertEqual(self.transport.response_queue.qsize(), 0)
                self.feed(frame[split:])
                self.assertEqual(self.transport.response_queue.qsize(), 1)
                self.assertIn(9, self.transport._latest_status_response)

    def test_payload_header_is_not_a_frame_boundary(self):
        self.feed(self.status(1, motor.Motor.FRAME_HEAD + b"\0" * 4))
        self.assertEqual(self.transport.response_queue.qsize(), 1)
        self.assertEqual(len(self.transport.receive_buffer), 0)

    def test_broadcast_and_status_can_share_the_same_serial_read(self):
        seen = []

        def callback(response):
            # This callback also verifies it is not called under buffer_lock.
            seen.append(self.transport.get_buffer_status())

        self.transport.add_response_callback(
            motor.ResponseType.ANGLE_BROADCAST, callback,
        )
        self.feed(
            self.status(1) + self.broadcast(motor.Motor.ANGLE_BROADCAST_HEAD1)
            + self.status(16)
        )
        self.assertEqual(len(seen), 1)
        self.assertEqual(self.transport.response_queue.qsize(), 3)
        self.assertEqual(set(self.transport._latest_status_response), {1, 16})

    def test_unrecognized_bytes_cannot_grow_buffer_forever(self):
        self.feed(b"\xff" * 4096 + b"\x55\xaa")
        self.assertEqual(self.transport.receive_buffer, b"\x55\xaa")

    def test_recorded_ring_status_passes_crc_and_preserves_angle(self):
        # Captured ID 7 status; independent of the SDK frame builder.
        frame = bytes.fromhex("55aa00140007010000008f62fdffcd0578d70000")
        self.feed(frame)
        status = self.transport._latest_status_response[7]
        self.assertEqual(status["angle"], -0.0234375)
        self.assertEqual(self.transport.get_queue_status()["crc_error_count"], 0)

    def test_bad_zero_cannot_replace_a_valid_nonzero_status(self):
        frame = self.status(7, bytes([1, 0, 1, 75, 128, 1, 0, 12]))
        self.feed(frame)
        before = dict(self.transport._latest_status_response[7])
        generation = self.transport._status_response_generation[7]
        corrupt = bytearray(frame)
        corrupt[12:14] = b"\0\0"  # 3 degrees -> zero without updating CRC.
        self.feed(corrupt)
        self.assertEqual(self.transport._latest_status_response[7], before)
        self.assertEqual(self.transport._status_response_generation[7], generation)
        self.assertEqual(self.transport.response_queue.qsize(), 1)
        self.assertEqual(self.transport.get_queue_status()["crc_error_count"], 1)
        self.assertEqual(self.transport._parse_single_frame(corrupt), [])

    def test_truncated_bad_packet_recovers_next_valid_frame(self):
        self.feed(self.status(7)[:13] + self.status(8))
        self.assertEqual(set(self.transport._latest_status_response), {8})
        self.assertEqual(self.transport.get_queue_status()["crc_error_count"], 1)
        self.assertEqual(len(self.transport.receive_buffer), 0)

    def test_all_broadcast_types_reject_corrupt_payload_and_checksum(self):
        headers = (
            motor.Motor.ANGLE_BROADCAST_HEAD1, motor.Motor.ANGLE_BROADCAST_HEAD2,
            motor.Motor.SPEED_BROADCAST_HEAD1, motor.Motor.SPEED_BROADCAST_HEAD2,
            motor.Motor.CURRENT_BROADCAST_HEAD1, motor.Motor.CURRENT_BROADCAST_HEAD2,
            motor.Motor.ENABLE_BROADCAST_HEAD1, motor.Motor.ENABLE_BROADCAST_HEAD2,
        )
        for header in headers:
            with self.subTest(header=header):
                self.transport = motor.Motor()
                frame = self.broadcast(header)
                seen = []
                for response_type in motor.ResponseType:
                    self.transport.add_response_callback(response_type, seen.append)
                for index in (2, 18):
                    corrupt = bytearray(frame)
                    corrupt[index] ^= 1
                    self.feed(corrupt)
                self.assertEqual(seen, [])
                self.feed(frame)
                self.assertEqual(len(seen), 1)
                self.assertEqual(
                    self.transport.get_queue_status()["crc_error_count"], 2,
                )


if __name__ == "__main__":
    unittest.main()
