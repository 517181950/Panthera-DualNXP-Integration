from __future__ import annotations

from collections import defaultdict, deque
import errno
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest

import serial

from gello.config import GelloConfig, config_from_dict, load_config, save_config
from gello.protocol import (
    ChecksumError,
    ServoStatusError,
    build_position_request,
    build_position_velocity_request,
    checksum,
    decode_signed_magnitude,
    parse_position_response,
    pop_status_packet,
)
from gello.reader import (
    ConnectionState,
    GelloReader,
    RADIANS_PER_TICK,
    wrapped_tick_delta,
)


def position_response(servo_id: int, raw: int, *, status: int = 0) -> bytes:
    packet = bytearray(
        (0xFF, 0xFF, servo_id, 0x04, status, raw & 0xFF, (raw >> 8) & 0xFF, 0)
    )
    packet[-1] = checksum(packet[2:-1])
    return bytes(packet)


def position_velocity_response(servo_id: int, raw: int, speed: int, *, status: int = 0) -> bytes:
    word = abs(speed) | (0x8000 if speed < 0 else 0)
    packet = bytearray((0xFF, 0xFF, servo_id, 6, status,
                        raw & 0xFF, (raw >> 8) & 0xFF,
                        word & 0xFF, word >> 8, 0))
    packet[-1] = checksum(packet[2:-1])
    return bytes(packet)


class FakeServoSerial:
    def __init__(
        self,
        *,
        missing_ids: tuple[int, ...] = (),
        noise_once: bytes = b"",
        max_read_chunk: int = 64,
        timeout: float = 0.001,
        **_kwargs,
    ) -> None:
        self.timeout = timeout
        self.missing_ids = set(missing_ids)
        self.noise_once = noise_once
        self.max_read_chunk = max_read_chunk
        self.positions = {servo_id: 1000 + servo_id for servo_id in range(1, 18)}
        self.velocities = {servo_id: 100 for servo_id in range(1, 18)}
        self.statuses: dict[int, int] = {}
        self.queued_positions: dict[int, deque[int]] = defaultdict(deque)
        self.buffer = bytearray()
        self.closed = False
        self.requested_ids: list[int] = []
        self._lock = threading.Lock()

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self.buffer)

    def set_position(self, servo_id: int, raw: int) -> None:
        with self._lock:
            self.positions[servo_id] = raw

    def set_status(self, servo_id: int, status: int) -> None:
        with self._lock:
            self.statuses[servo_id] = status

    def queue_positions(self, servo_id: int, values: list[int]) -> None:
        with self._lock:
            self.queued_positions[servo_id].extend(values)

    def write(self, data: bytes) -> int:
        servo_id = data[2]
        if data not in (build_position_request(servo_id), build_position_velocity_request(servo_id)):
            raise AssertionError(f"unexpected request: {data.hex()}")
        with self._lock:
            self.requested_ids.append(servo_id)
            if servo_id not in self.missing_ids:
                queued = self.queued_positions[servo_id]
                raw = queued.popleft() if queued else self.positions[servo_id]
                if self.noise_once:
                    self.buffer.extend(self.noise_once)
                    self.noise_once = b""
                self.buffer.extend(
                    position_velocity_response(
                        servo_id, raw, self.velocities[servo_id],
                        status=self.statuses.get(servo_id, 0),
                    ) if data[6] == 4 else position_response(
                        servo_id, raw, status=self.statuses.get(servo_id, 0)
                    )
                )
        return len(data)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            count = min(size, self.max_read_chunk, len(self.buffer))
            result = bytes(self.buffer[:count])
            del self.buffer[:count]
        if not result:
            time.sleep(self.timeout)
        return result

    def reset_input_buffer(self) -> None:
        with self._lock:
            self.buffer.clear()

    def reset_output_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class FailAfterWritesSerial(FakeServoSerial):
    def __init__(self, fail_after: int, **kwargs) -> None:
        super().__init__(**kwargs)
        self.fail_after = fail_after
        self.write_count = 0

    def write(self, data: bytes) -> int:
        if self.write_count >= self.fail_after:
            raise serial.SerialException(errno.EIO, "USB device disconnected")
        self.write_count += 1
        return super().write(data)


class SerialFactorySequence:
    def __init__(self, *results: FakeServoSerial | BaseException) -> None:
        self.results = deque(results)
        self.calls = 0

    def __call__(self, **_kwargs) -> FakeServoSerial:
        self.calls += 1
        if not self.results:
            raise AssertionError("serial factory called more times than expected")
        result = self.results.popleft()
        if isinstance(result, BaseException):
            raise result
        return result


def wait_until(predicate, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise TimeoutError("condition was not reached")


class ProtocolTests(unittest.TestCase):
    def test_position_request_matches_ragtel_protocol(self) -> None:
        self.assertEqual(
            build_position_request(1).hex(" ").upper(),
            "FF FF 01 04 02 38 02 BE",
        )

    def test_position_response_validation(self) -> None:
        response = parse_position_response(position_response(17, 0xABC), expected_id=17)
        self.assertEqual(response.servo_id, 17)
        self.assertEqual(response.raw_position, 0xABC)

        corrupted = bytearray(position_response(1, 1234))
        corrupted[-1] ^= 0x01
        with self.assertRaises(ChecksumError):
            parse_position_response(bytes(corrupted))
        with self.assertRaises(ServoStatusError):
            parse_position_response(position_response(1, 1234, status=4))

        warning = parse_position_response(
            position_response(1, 1234, status=4),
            reject_error_status=False,
        )
        self.assertEqual(warning.raw_position, 1234)
        self.assertEqual(warning.status, 4)

    def test_every_12_bit_position_value_decodes(self) -> None:
        for raw in range(4096):
            response = parse_position_response(position_response(1, raw))
            self.assertEqual(response.raw_position, raw)

    def test_sms_sts_position_direction_bit_is_sign_magnitude(self) -> None:
        self.assertEqual(decode_signed_magnitude(0x0001, 15), 1)
        self.assertEqual(decode_signed_magnitude(0x8001, 15), -1)
        self.assertEqual(
            parse_position_response(position_response(4, 0x8001)).raw_position,
            -1,
        )

    def test_rolling_buffer_recovers_from_noise_and_bad_length(self) -> None:
        expected = position_response(3, 2222)
        buffer = bytearray(b"\x12\xff\x01\xff\xff\x02\xffjunk" + expected)
        packet = None
        discarded = 0
        while packet is None:
            packet, count = pop_status_packet(buffer)
            discarded += count
        self.assertEqual(packet, expected)
        self.assertGreater(discarded, 0)


class RolloverTests(unittest.TestCase):
    def test_forward_and_reverse_rollover(self) -> None:
        self.assertEqual(wrapped_tick_delta(5, 4090), 11)
        self.assertEqual(wrapped_tick_delta(4090, 5), -11)
        self.assertEqual(wrapped_tick_delta(110, 100), 10)


class GripperConfigTests(unittest.TestCase):
    def test_unwrapped_calibration_range_may_cross_4095(self) -> None:
        config = config_from_dict(
            {
                "left": {
                    "gripper": {
                        "min_raw": 4000,
                        "max_raw": 4200,
                    }
                }
            }
        )

        self.assertEqual(config.left.gripper.min_raw, 4000)
        self.assertEqual(config.left.gripper.max_raw, 4200)
        self.assertAlmostEqual(
            config.left.gripper.apply(4100),
            (config.left.gripper.output_min + config.left.gripper.output_max) / 2,
        )


class ReaderTests(unittest.TestCase):
    def make_reader(
        self,
        fake: FakeServoSerial,
        *,
        group_mask: int = 0x3,
        scan_interval_s: float = 0.01,
    ) -> GelloReader:
        return GelloReader(
            "fake",
            group_mask=group_mask,
            timeout_s=0.001,
            scan_interval_s=scan_interval_s,
            serial_factory=lambda **_kwargs: fake,
        )

    def test_one_port_scans_all_fourteen_ids_and_publishes_two_groups(self) -> None:
        fake = FakeServoSerial(max_read_chunk=2)
        reader = self.make_reader(fake)
        reader.start()
        try:
            snapshot = reader.wait_for_snapshot(0.5, require_complete=True)
        finally:
            reader.stop()

        self.assertEqual(tuple(group.group for group in snapshot.groups), (0, 1))
        self.assertEqual(snapshot.group(0).servo_ids, tuple(range(1, 8)))
        self.assertEqual(snapshot.group(1).servo_ids, tuple(range(11, 18)))
        self.assertEqual(fake.requested_ids[:14], list(range(1, 8)) + list(range(11, 18)))
        self.assertTrue(all(value == 0.0 for value in snapshot.group(0).arm_radians))
        self.assertTrue(fake.closed)

    def test_reader_recovers_frame_alignment_without_resetting_input(self) -> None:
        fake = FakeServoSerial(noise_once=b"\x00\x44\xff", max_read_chunk=1)
        reader = self.make_reader(fake, group_mask=0x1)
        reader.start()
        try:
            snapshot = reader.wait_for_snapshot(0.5, require_complete=True)
            stats = reader.stats
        finally:
            reader.stop()
        self.assertTrue(snapshot.group(0).data_valid)
        self.assertGreater(stats.frame_errors, 0)

    def test_servo_status_warning_keeps_valid_position_data(self) -> None:
        fake = FakeServoSerial()
        fake.set_status(4, 0x04)
        reader = self.make_reader(fake, group_mask=0x1)
        reader.start()
        try:
            snapshot = reader.wait_for_snapshot(0.5, require_complete=True)
            stats = reader.stats
        finally:
            reader.stop()

        self.assertTrue(snapshot.group(0).data_valid)
        self.assertEqual(snapshot.group(0).raw_positions[3], 1004)
        self.assertGreater(stats.status_errors, 0)

    def test_forward_rollover_is_unwrapped(self) -> None:
        fake = FakeServoSerial()
        fake.set_position(1, 4090)
        reader = self.make_reader(fake, group_mask=0x1, scan_interval_s=0.03)
        reader.start()
        try:
            first = reader.wait_for_snapshot(0.5, require_complete=True)
            fake.set_position(1, 5)
            second = reader.wait_for_snapshot(
                0.5, after_sequence=first.sequence, require_complete=True
            )
        finally:
            reader.stop()

        group = second.group(0)
        self.assertEqual(group.unwrapped_ticks[0], 4101)
        self.assertAlmostEqual(group.arm_radians[0], -11 * RADIANS_PER_TICK)

    def test_signed_position_crosses_zero_without_frame_error(self) -> None:
        fake = FakeServoSerial()
        fake.set_position(4, 0x8001)  # SMS/STS sign-magnitude encoding for -1.
        reader = self.make_reader(fake, group_mask=0x1, scan_interval_s=0.03)
        reader.start()
        try:
            first = reader.wait_for_snapshot(0.5, require_complete=True)
            fake.set_position(4, 0x0000)
            second = reader.wait_for_snapshot(
                0.5, after_sequence=first.sequence, require_complete=True
            )
            stats = reader.stats
        finally:
            reader.stop()

        self.assertEqual(first.group(0).raw_positions[3], 4095)
        self.assertEqual(second.group(0).raw_positions[3], 0)
        self.assertEqual(second.group(0).unwrapped_ticks[3], 4096)
        self.assertAlmostEqual(
            second.group(0).arm_radians[3], -RADIANS_PER_TICK
        )
        self.assertEqual(stats.frame_errors, 0)

    def test_gripper_calibration_crosses_rollover_continuously(self) -> None:
        fake = FakeServoSerial()
        fake.set_position(7, 4090)
        reader = self.make_reader(fake, group_mask=0x1, scan_interval_s=0.03)
        reader.start()
        try:
            first = reader.wait_for_snapshot(0.5, require_complete=True)
            reader.toggle_gripper_calibration(0)
            calibrating = reader.wait_for_snapshot(
                0.5, after_sequence=first.sequence, require_complete=True
            )
            fake.set_position(7, 5)
            crossed = reader.wait_for_snapshot(
                0.5, after_sequence=calibrating.sequence, require_complete=True
            )
            reader.toggle_gripper_calibration(0)
            calibrated = reader.wait_for_snapshot(
                0.5, after_sequence=crossed.sequence, require_complete=True
            )
        finally:
            reader.stop()

        group = calibrated.group(0)
        self.assertEqual(group.raw_positions[-1], 5)
        self.assertEqual(group.unwrapped_ticks[-1], 4101)
        self.assertEqual(group.gripper_calib_min_raw, 4090)
        self.assertEqual(group.gripper_calib_max_raw, 4101)
        self.assertAlmostEqual(group.gripper_normalized, 1.0)
        self.assertAlmostEqual(group.gripper_radians, 0.6)

    def test_saved_gripper_range_is_aligned_to_current_turn(self) -> None:
        fake = FakeServoSerial()
        fake.set_position(7, 20)
        reader = self.make_reader(fake, group_mask=0x1, scan_interval_s=0.03)
        reader.start()
        try:
            first = reader.wait_for_snapshot(0.5, require_complete=True)
            reader.set_gripper_calibration(0, 3890, 4140)
            calibrated = reader.wait_for_snapshot(
                0.5, after_sequence=first.sequence, require_complete=True
            )
        finally:
            reader.stop()

        group = calibrated.group(0)
        self.assertEqual(group.gripper_calib_min_raw, -206)
        self.assertEqual(group.gripper_calib_max_raw, 44)
        self.assertAlmostEqual(group.gripper_normalized, 226 / 250)
        self.assertAlmostEqual(group.gripper_radians, 0.6 * 226 / 250)

    def test_suspicious_jump_uses_consistent_reread(self) -> None:
        fake = FakeServoSerial()
        fake.set_position(1, 1000)
        reader = self.make_reader(fake, group_mask=0x1, scan_interval_s=0.03)
        reader.start()
        try:
            first = reader.wait_for_snapshot(0.5, require_complete=True)
            fake.queue_positions(1, [3000, 1005])
            second = reader.wait_for_snapshot(
                0.5, after_sequence=first.sequence, require_complete=True
            )
            stats = reader.stats
        finally:
            reader.stop()

        self.assertEqual(second.group(0).raw_positions[0], 1005)
        self.assertAlmostEqual(second.group(0).arm_radians[0], -5 * RADIANS_PER_TICK)
        self.assertEqual(stats.jump_retries, 1)
        self.assertEqual(stats.jump_rejects, 0)

    def test_missing_node_is_visible_in_snapshot_and_statistics(self) -> None:
        fake = FakeServoSerial(missing_ids=(17,))
        reader = self.make_reader(fake)
        reader.start()
        try:
            snapshot = reader.wait_for_snapshot(0.5)
            stats = reader.stats
        finally:
            reader.stop()

        right = snapshot.group(1)
        self.assertFalse(right.data_valid)
        self.assertIsNone(right.raw_positions[-1])
        self.assertGreaterEqual(dict(stats.node_timeouts)[17], 1)
        self.assertGreaterEqual(stats.incomplete_scans, 1)

    def test_explicit_zero_and_gripper_calibration_are_worker_commands(self) -> None:
        fake = FakeServoSerial()
        reader = self.make_reader(fake, group_mask=0x1, scan_interval_s=0.03)
        reader.start()
        try:
            first = reader.wait_for_snapshot(0.5, require_complete=True)
            fake.set_position(1, 1011)
            moved = reader.wait_for_snapshot(
                0.5, after_sequence=first.sequence, require_complete=True
            )
            self.assertNotEqual(moved.group(0).arm_radians[0], 0.0)

            reader.capture_zero(0)
            reader.set_gripper_calibration(0, 900, 1100)
            zeroed = reader.wait_for_snapshot(
                0.5, after_sequence=moved.sequence, require_complete=True
            )
        finally:
            reader.stop()

        self.assertEqual(zeroed.group(0).arm_radians[0], 0.0)
        self.assertAlmostEqual(zeroed.group(0).gripper_normalized, 0.535)

    def test_transport_error_reconnects_and_preserves_zero_and_unwrap_state(self) -> None:
        first_port = FailAfterWritesSerial(fail_after=7)
        second_port = FakeServoSerial()
        second_port.set_position(1, 1011)
        factory = SerialFactorySequence(first_port, second_port)
        reader = GelloReader(
            "fake",
            group_mask=0x1,
            timeout_s=0.001,
            reconnect_initial_s=0.03,
            reconnect_max_s=0.03,
            serial_factory=factory,
        )
        reader.start()
        try:
            first = reader.wait_for_snapshot(0.5, require_complete=True)
            recovered = reader.wait_for_snapshot(
                1.0, after_sequence=first.sequence, require_complete=True
            )
            stats = reader.stats
        finally:
            reader.stop()

        self.assertEqual(factory.calls, 2)
        self.assertTrue(first_port.closed)
        self.assertEqual(recovered.group(0).raw_positions[0], 1011)
        self.assertAlmostEqual(
            recovered.group(0).arm_radians[0], -10 * RADIANS_PER_TICK
        )
        self.assertEqual(stats.reconnect_attempts, 1)
        self.assertEqual(stats.successful_reconnects, 1)
        self.assertIsNone(reader.fatal_error)

    def test_total_bus_silence_reopens_the_port(self) -> None:
        silent_port = FakeServoSerial(missing_ids=tuple(range(1, 8)))
        working_port = FakeServoSerial()
        factory = SerialFactorySequence(silent_port, working_port)
        reader = GelloReader(
            "fake",
            group_mask=0x1,
            timeout_s=0.001,
            link_loss_s=0.015,
            reconnect_initial_s=0.003,
            reconnect_max_s=0.003,
            serial_factory=factory,
        )
        reader.start()
        try:
            snapshot = reader.wait_for_snapshot(1.0, require_complete=True)
            stats = reader.stats
        finally:
            reader.stop()

        self.assertTrue(snapshot.group(0).data_valid)
        self.assertTrue(silent_port.closed)
        self.assertEqual(factory.calls, 2)
        self.assertEqual(stats.reconnect_attempts, 1)
        self.assertEqual(stats.successful_reconnects, 1)

    def test_reconnect_wait_reports_error_attempt_and_countdown(self) -> None:
        factory = SerialFactorySequence(
            serial.SerialException(errno.ENOENT, "device missing"),
            FakeServoSerial(),
        )
        reader = GelloReader(
            "fake",
            group_mask=0x1,
            reconnect_initial_s=0.05,
            reconnect_max_s=0.05,
            serial_factory=factory,
        )
        reader.start()
        try:
            wait_until(
                lambda: reader.connection_status.state
                == ConnectionState.RECONNECT_WAIT
            )
            status = reader.connection_status
            self.assertEqual(status.reconnect_attempt, 1)
            self.assertIn("device missing", status.last_error)
            self.assertGreater(status.retry_in_s, 0.0)
            reader.wait_for_snapshot(1.0, require_complete=True)
        finally:
            reader.stop()

        self.assertEqual(factory.calls, 2)
        self.assertEqual(reader.stats.successful_reconnects, 1)

    def test_permission_error_is_terminal_and_is_not_retried(self) -> None:
        factory = SerialFactorySequence(
            PermissionError(errno.EACCES, "permission denied")
        )
        reader = GelloReader(
            "fake",
            group_mask=0x1,
            reconnect_initial_s=0.001,
            reconnect_max_s=0.001,
            serial_factory=factory,
        )
        reader.start()
        try:
            wait_until(lambda: reader.fatal_error is not None)
            self.assertEqual(
                reader.connection_status.state, ConnectionState.FAILED
            )
        finally:
            reader.stop()

        self.assertEqual(factory.calls, 1)
        self.assertIsInstance(reader.fatal_error, PermissionError)

    def test_disabling_auto_reconnect_makes_transport_error_terminal(self) -> None:
        factory = SerialFactorySequence(serial.SerialException("port failed"))
        reader = GelloReader(
            "fake",
            group_mask=0x1,
            auto_reconnect=False,
            serial_factory=factory,
        )
        reader.start()
        try:
            wait_until(lambda: reader.fatal_error is not None)
        finally:
            reader.stop()

        self.assertEqual(factory.calls, 1)
        self.assertIsInstance(reader.fatal_error, serial.SerialException)


class ConfigTests(unittest.TestCase):
    def test_round_trip_and_mapping(self) -> None:
        config = GelloConfig()
        config.left.axes[0].sign = -1
        config.left.axes[0].scale = 2.0
        config.left.axes[0].offset_rad = 0.25
        config.swap_arms = True
        config.right.gripper.min_raw = 1000
        config.right.gripper.max_raw = 2000
        config.right.gripper.inverted = True
        config.serial.port = "/dev/serial/by-id/example"
        config.serial.link_loss_ms = 750
        config.serial.reconnect_initial_ms = 100
        config.serial.reconnect_max_ms = 1500

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gello.json"
            save_config(config, path)
            loaded = load_config(path)
            raw_json = json.loads(path.read_text(encoding="utf-8"))

        self.assertEqual(loaded, config)
        self.assertEqual(raw_json["serial"]["group_mask"], 3)
        self.assertTrue(raw_json["swap_arms"])
        self.assertTrue(loaded.swap_arms)
        self.assertAlmostEqual(loaded.left.axes[0].apply(0.5), -0.75)
        self.assertAlmostEqual(loaded.right.gripper.apply(1000), 0.04)

    def test_invalid_config_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            config_from_dict({"serial": {"group_mask": 7}})
        with self.assertRaises(ValueError):
            config_from_dict(
                {"left": {"axes": [{"sign": 1, "scale": 1}] * 5}}
            )

    def test_missing_swap_arms_defaults_to_false(self) -> None:
        self.assertFalse(config_from_dict({}).swap_arms)


class TkModuleTests(unittest.TestCase):
    def test_tk_module_imports_without_opening_a_window(self) -> None:
        from gello.tk_app import GelloTunerApp

        self.assertEqual(GelloTunerApp.REFRESH_MS, 50)


if __name__ == "__main__":
    unittest.main()
