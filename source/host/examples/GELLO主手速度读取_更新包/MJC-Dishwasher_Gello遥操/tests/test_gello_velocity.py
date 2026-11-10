from __future__ import annotations

import math
from pathlib import Path
import tempfile
import unittest

from gello.config import AxisConfig, config_from_dict, load_config, save_config
from gello.protocol import (
    ChecksumError, ProtocolError, ServoStatusError,
    build_position_velocity_request, parse_position_velocity_response,
)
from gello.reader import ConnectionState, GelloReader, SERVO_SIGNS
from test_gello import FakeServoSerial, position_response, position_velocity_response


class VelocityTests(unittest.TestCase):
    def reader(self, **kwargs):
        reader = GelloReader("fake", group_mask=1, timeout_s=0.001, **kwargs)
        port = FakeServoSerial(max_read_chunk=1)
        reader._serial = port
        self.addCleanup(port.close)
        return reader, port

    def test_wire_format_signed_speed_and_position(self):
        self.assertEqual(build_position_velocity_request(1).hex(), "ffff0104023804bc")
        for speed in (0, 1, -1, 32767, -32767):
            response = parse_position_velocity_response(
                position_velocity_response(17, 0x8001, speed), expected_id=17
            )
            self.assertEqual(response.raw_position, -1)
            self.assertEqual(response.raw_velocity, speed)

    def test_rejects_corruption_wrong_id_length_and_status(self):
        packet = position_velocity_response(1, 1000, -123)
        with self.assertRaises(ChecksumError):
            parse_position_velocity_response(packet[:-1] + bytes([packet[-1] ^ 1]))
        with self.assertRaises(ProtocolError):
            parse_position_velocity_response(packet, expected_id=2)
        for malformed in (packet[:-1], position_response(1, 1000), b"invalid"):
            with self.assertRaises(ProtocolError):
                parse_position_velocity_response(malformed)
        with self.assertRaises(ServoStatusError):
            parse_position_velocity_response(position_velocity_response(1, 1000, 0, status=4))

    def test_unit_direction_timestamps_and_zero_do_not_change_speed(self):
        for unit in (0.732, 0.0146):
            reader, port = self.reader(velocity_rpm_per_unit=unit)
            port.velocities.update({i: (-1 if i % 2 else 1) * i for i in range(1, 8)})
            reader._scan_once()
            group = reader.latest_snapshot().group(0)
            self.assertTrue(group.velocity_valid)
            for i, speed in enumerate(group.arm_velocities_rad_s):
                self.assertAlmostEqual(speed, port.velocities[i + 1] * unit * math.tau / 60 * SERVO_SIGNS[i])
            self.assertAlmostEqual(group.gripper_servo_velocity_rad_s, -7 * unit * math.tau / 60)
            self.assertTrue(all(0 < stamp <= group.captured_at for stamp in group.servo_captured_at))
            reader.capture_zero(0)
            reader._scan_once()
            self.assertEqual(reader.latest_snapshot().group(0).arm_velocities_rad_s, group.arm_velocities_rad_s)
            self.assertEqual(AxisConfig(sign=-1, scale=2, offset_rad=50).apply_velocity(3), -6)

    def test_missing_warning_and_rejected_jump_never_reuse_speed(self):
        reader, port = self.reader()
        reader._scan_once()
        self.assertTrue(reader.latest_snapshot().group(0).velocity_valid)
        port.missing_ids.add(1)
        port.statuses[2] = 4
        port.queue_positions(3, [2000, 3000])
        reader._scan_once()
        group = reader.latest_snapshot().group(0)
        self.assertFalse(group.velocity_valid)
        self.assertEqual(group.raw_velocities[:3], (None, None, None))
        self.assertEqual(group.arm_velocities_rad_s[:3], (None, None, None))
        self.assertIsNone(group.servo_captured_at[0])
        self.assertIsNone(group.servo_captured_at[2])
        port.missing_ids.clear()
        port.statuses.clear()
        reader._scan_once()
        self.assertTrue(reader.latest_snapshot().group(0).velocity_valid)

    def test_rejected_frames_do_not_publish_old_speed(self):
        reader, port = self.reader()
        reader._scan_once()
        # A valid position-only response is insufficient for the combined read.
        port.missing_ids.add(1)
        port.buffer.extend(position_response(1, 1234))
        reader._scan_once()
        self.assertIsNone(reader.latest_snapshot().group(0).raw_velocities[0])
        self.assertGreater(reader.stats.frame_errors, 0)

    def test_disabled_read_keeps_position_working(self):
        reader, port = self.reader(read_velocity=False)
        reader._scan_once()
        group = reader.latest_snapshot().group(0)
        self.assertTrue(group.data_valid)
        self.assertFalse(group.velocity_valid)
        self.assertEqual(group.raw_velocities, (None,) * 7)

    def test_disconnect_invalidates_velocity_until_new_samples(self):
        reader, port = self.reader()
        reader._scan_once()
        original = reader.latest_snapshot().group(0)
        reader._set_connection(ConnectionState.RECONNECT_WAIT)
        group = reader.latest_snapshot().group(0)
        self.assertFalse(group.velocity_valid)
        self.assertEqual(group.raw_velocities, (None,) * 7)
        self.assertEqual(group.arm_radians, original.arm_radians)
        reader._set_connection(ConnectionState.CONNECTED)
        self.assertFalse(reader.latest_snapshot().group(0).velocity_valid)
        reader._scan_once()
        self.assertTrue(reader.latest_snapshot().group(0).velocity_valid)

    def test_config_round_trip_and_invalid_units(self):
        config = config_from_dict({"serial": {"read_velocity": False, "velocity_rpm_per_unit": 0.0146}})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            save_config(config, path)
            self.assertEqual(load_config(path), config)
        for unit in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                config_from_dict({"serial": {"velocity_rpm_per_unit": unit}})
            with self.assertRaises(ValueError):
                GelloReader("fake", velocity_rpm_per_unit=unit)
