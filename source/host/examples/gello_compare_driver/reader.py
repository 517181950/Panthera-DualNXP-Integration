"""Threaded reader for two seven-servo Gello groups on one serial bus."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from enum import Enum
import errno
import math
import queue
import threading
import time
from typing import Callable, Protocol

import serial

from .protocol import (
    ChecksumError,
    ProtocolError,
    build_position_request,
    build_position_velocity_request,
    parse_position_response,
    parse_position_velocity_response,
    pop_status_packet,
)


TICKS_PER_REVOLUTION = 4096
HALF_REVOLUTION_TICKS = TICKS_PER_REVOLUTION // 2
RADIANS_PER_TICK = 2.0 * 3.141592653589793 / TICKS_PER_REVOLUTION
GRIPPER_TRAVEL_RADIANS = 0.6
SERVOS_PER_GROUP = 7
ARM_SERVOS_PER_GROUP = 6
GROUP_SERVO_IDS = (
    (1, 2, 3, 4, 5, 6, 7),
    (11, 12, 13, 14, 15, 16, 17),
)
SERVO_SIGNS = (-1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0)
SERVO_GEARS = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 4.0)


class BusSilenceError(OSError):
    """No selected servo has returned a valid frame for the configured period."""


class ConnectionState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECT_WAIT = "reconnect_wait"
    FAILED = "failed"


class SerialPort(Protocol):
    @property
    def in_waiting(self) -> int: ...

    def write(self, data: bytes) -> int: ...
    def read(self, size: int = 1) -> bytes: ...
    def reset_input_buffer(self) -> None: ...
    def reset_output_buffer(self) -> None: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class GroupSnapshot:
    group: int
    servo_ids: tuple[int, ...]
    raw_positions: tuple[int | None, ...]
    unwrapped_ticks: tuple[int | None, ...]
    arm_radians: tuple[float | None, ...]
    gripper_radians: float | None
    gripper_normalized: float | None
    gripper_calib_min_raw: int | None
    gripper_calib_max_raw: int | None
    gripper_calib_active: bool
    gripper_calib_done: bool
    zeroed: bool
    data_valid: bool
    captured_at: float
    # Signed register values, before any direction/gear mapping. Missing is None.
    raw_velocities: tuple[int | None, ...] = (None,) * SERVOS_PER_GROUP
    arm_velocities_rad_s: tuple[float | None, ...] = (None,) * ARM_SERVOS_PER_GROUP
    # Actual gripper servo shaft speed, not calibrated finger travel speed.
    gripper_servo_velocity_rad_s: float | None = None
    velocity_valid: bool = False
    servo_captured_at: tuple[float | None, ...] = (None,) * SERVOS_PER_GROUP


@dataclass(frozen=True)
class ReaderSnapshot:
    groups: tuple[GroupSnapshot, ...]
    captured_at: float
    sequence: int
    scan_duration_s: float

    def group(self, index: int) -> GroupSnapshot | None:
        return next((group for group in self.groups if group.group == index), None)


@dataclass(frozen=True)
class ReaderStats:
    scans: int
    complete_scans: int
    incomplete_scans: int
    servo_timeouts: int
    checksum_errors: int
    frame_errors: int
    status_errors: int
    unexpected_ids: int
    jump_retries: int
    jump_rejects: int
    scan_hz: float
    last_scan_s: float
    p95_scan_s: float
    p99_scan_s: float
    reconnect_attempts: int
    successful_reconnects: int
    node_timeouts: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class ConnectionStatus:
    state: ConnectionState
    reconnect_attempt: int
    last_error: str | None
    retry_at: float | None

    @property
    def retry_in_s(self) -> float:
        if self.retry_at is None:
            return 0.0
        return max(0.0, self.retry_at - time.perf_counter())


@dataclass
class _Tracker:
    initialized: bool = False
    previous_raw: int = 0
    unwrapped: int = 0
    zero_offset: int = 0


@dataclass
class _GripperCalibration:
    minimum: int | None = None
    maximum: int | None = None
    active: bool = False
    done: bool = False
    aligned: bool = False


@dataclass(frozen=True)
class _Command:
    kind: str
    group: int
    minimum: int = 0
    maximum: int = 0


def wrapped_tick_delta(current: int, previous: int) -> int:
    """Return the shortest signed 12-bit delta, including 4095/0 rollover."""
    delta = current - previous
    if delta > HALF_REVOLUTION_TICKS:
        delta -= TICKS_PER_REVOLUTION
    elif delta < -HALF_REVOLUTION_TICKS:
        delta += TICKS_PER_REVOLUTION
    return delta


def _default_serial_factory(**kwargs) -> SerialPort:
    return serial.Serial(**kwargs)


def _is_recoverable_transport_error(error: Exception) -> bool:
    """Return whether reopening the serial device can reasonably recover."""
    if isinstance(error, BusSilenceError):
        return True
    error_number = getattr(error, "errno", None)
    if isinstance(error, PermissionError) or error_number in (
        errno.EACCES,
        errno.EPERM,
        errno.EINVAL,
    ):
        return False
    if isinstance(error, serial.SerialException):
        return True
    return isinstance(error, OSError)


class GelloReader:
    """Continuously scan one or both seven-servo groups in a daemon thread.

    Group 0 uses IDs 1..7 and group 1 uses IDs 11..17. The worker owns the
    serial port and all unwrap/calibration state. Other threads exchange only
    immutable snapshots and discrete commands with it.
    """

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 1_000_000,
        group_mask: int = 0x3,
        timeout_s: float = 0.005,
        scan_interval_s: float = 0.0,
        max_delta_ticks: int = 100,
        auto_reconnect: bool = True,
        link_loss_s: float = 0.5,
        reconnect_initial_s: float = 0.25,
        reconnect_max_s: float = 2.0,
        read_velocity: bool = True,
        velocity_rpm_per_unit: float = 0.732,
        serial_factory: Callable[..., SerialPort] = _default_serial_factory,
    ) -> None:
        if not port:
            raise ValueError("port must not be empty")
        if baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if group_mask not in (0x1, 0x2, 0x3):
            raise ValueError("group_mask must be 1, 2, or 3")
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if scan_interval_s < 0:
            raise ValueError("scan_interval_s must be non-negative")
        if not 0 < max_delta_ticks < HALF_REVOLUTION_TICKS:
            raise ValueError("max_delta_ticks must be in the range 1..2047")
        if link_loss_s <= 0:
            raise ValueError("link_loss_s must be positive")
        if reconnect_initial_s <= 0:
            raise ValueError("reconnect_initial_s must be positive")
        if reconnect_max_s < reconnect_initial_s:
            raise ValueError("reconnect_max_s must be at least reconnect_initial_s")
        if not isinstance(read_velocity, bool):
            raise ValueError("read_velocity must be a boolean")
        if not math.isfinite(velocity_rpm_per_unit) or velocity_rpm_per_unit <= 0:
            raise ValueError("velocity_rpm_per_unit must be positive and finite")

        self.port = port
        self.baudrate = baudrate
        self.group_mask = group_mask
        self.timeout_s = timeout_s
        self.scan_interval_s = scan_interval_s
        self.max_delta_ticks = max_delta_ticks
        self.auto_reconnect = auto_reconnect
        self.link_loss_s = link_loss_s
        self.reconnect_initial_s = reconnect_initial_s
        self.reconnect_max_s = reconnect_max_s
        self.read_velocity = read_velocity
        self.velocity_rpm_per_unit = velocity_rpm_per_unit
        self._serial_factory = serial_factory

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial: SerialPort | None = None
        self._rx_buffer = bytearray()
        self._velocity_samples: dict[int, int | None] = {}
        self._sample_times: dict[int, float] = {}
        self._commands: queue.SimpleQueue[_Command] = queue.SimpleQueue()
        self._lock = threading.Lock()
        self._snapshot: ReaderSnapshot | None = None
        self._fatal_error: BaseException | None = None
        self._connected = False
        self._connection_status = ConnectionStatus(
            state=ConnectionState.DISCONNECTED,
            reconnect_attempt=0,
            last_error=None,
            retry_at=None,
        )

        self._trackers = [
            [_Tracker() for _ in range(SERVOS_PER_GROUP)] for _ in range(2)
        ]
        self._zeroed = [False, False]
        self._gripper_calibration = [_GripperCalibration(), _GripperCalibration()]

        self._scans = 0
        self._complete_scans = 0
        self._incomplete_scans = 0
        self._servo_timeouts = 0
        self._checksum_errors = 0
        self._frame_errors = 0
        self._status_errors = 0
        self._unexpected_ids = 0
        self._jump_retries = 0
        self._jump_rejects = 0
        self._node_timeouts = {
            servo_id: 0 for ids in GROUP_SERVO_IDS for servo_id in ids
        }
        self._scan_times: deque[float] = deque(maxlen=256)
        self._scan_durations: deque[float] = deque(maxlen=256)
        self._last_scan_s = 0.0
        self._reconnect_attempts = 0
        self._successful_reconnects = 0

    def start(self) -> None:
        if self._thread is not None:
            if self._thread.is_alive():
                return
            raise RuntimeError("a stopped GelloReader cannot be restarted; create a new one")
        self._stop.clear()
        self._rx_buffer.clear()
        with self._lock:
            self._fatal_error = None
            self._set_connection_locked(ConnectionState.CONNECTING)
        self._thread = threading.Thread(
            target=self._run,
            name=f"gello-reader-{self.port}",
            daemon=True,
        )
        self._thread.start()

    def _open_port(self) -> SerialPort:
        port = self._serial_factory(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout_s,
            write_timeout=self.timeout_s,
        )
        try:
            port.reset_input_buffer()
            port.reset_output_buffer()
        except Exception:
            port.close()
            raise
        return port

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=max(1.0, self.timeout_s * 32))
        port = self._serial
        self._serial = None
        if port is not None:
            port.close()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        with self._lock:
            self._set_connection_locked(ConnectionState.DISCONNECTED)

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def fatal_error(self) -> BaseException | None:
        with self._lock:
            return self._fatal_error

    @property
    def connection_status(self) -> ConnectionStatus:
        with self._lock:
            return self._connection_status

    def latest_snapshot(self) -> ReaderSnapshot | None:
        with self._lock:
            return self._snapshot

    def latest(self, group: int) -> GroupSnapshot | None:
        snapshot = self.latest_snapshot()
        return snapshot.group(group) if snapshot is not None else None

    @property
    def stats(self) -> ReaderStats:
        with self._lock:
            scan_hz = 0.0
            if len(self._scan_times) >= 2:
                elapsed = self._scan_times[-1] - self._scan_times[0]
                if elapsed > 0:
                    scan_hz = (len(self._scan_times) - 1) / elapsed
            durations = sorted(self._scan_durations)

            def percentile(fraction: float) -> float:
                if not durations:
                    return 0.0
                index = math.ceil(fraction * len(durations)) - 1
                return durations[max(0, index)]

            return ReaderStats(
                scans=self._scans,
                complete_scans=self._complete_scans,
                incomplete_scans=self._incomplete_scans,
                servo_timeouts=self._servo_timeouts,
                checksum_errors=self._checksum_errors,
                frame_errors=self._frame_errors,
                status_errors=self._status_errors,
                unexpected_ids=self._unexpected_ids,
                jump_retries=self._jump_retries,
                jump_rejects=self._jump_rejects,
                scan_hz=scan_hz,
                last_scan_s=self._last_scan_s,
                p95_scan_s=percentile(0.95),
                p99_scan_s=percentile(0.99),
                reconnect_attempts=self._reconnect_attempts,
                successful_reconnects=self._successful_reconnects,
                node_timeouts=tuple(sorted(self._node_timeouts.items())),
            )

    def _set_connection_locked(
        self,
        state: ConnectionState,
        *,
        reconnect_attempt: int = 0,
        last_error: str | None = None,
        retry_at: float | None = None,
    ) -> None:
        self._connected = state == ConnectionState.CONNECTED
        if not self._connected and self._snapshot is not None:
            # Preserve the existing position snapshot, but never present speed
            # from a previous connection as current feedback.
            self._snapshot = replace(self._snapshot, groups=tuple(
                replace(group, raw_velocities=(None,) * SERVOS_PER_GROUP,
                        arm_velocities_rad_s=(None,) * ARM_SERVOS_PER_GROUP,
                        gripper_servo_velocity_rad_s=None, velocity_valid=False)
                for group in self._snapshot.groups
            ))
        self._connection_status = ConnectionStatus(
            state=state,
            reconnect_attempt=reconnect_attempt,
            last_error=last_error,
            retry_at=retry_at,
        )

    def _set_connection(
        self,
        state: ConnectionState,
        *,
        reconnect_attempt: int = 0,
        last_error: str | None = None,
        retry_at: float | None = None,
    ) -> None:
        with self._lock:
            self._set_connection_locked(
                state,
                reconnect_attempt=reconnect_attempt,
                last_error=last_error,
                retry_at=retry_at,
            )

    def wait_for_snapshot(
        self,
        timeout_s: float,
        *,
        after_sequence: int | None = None,
        require_complete: bool = False,
    ) -> ReaderSnapshot:
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            snapshot = self.latest_snapshot()
            if snapshot is not None and (
                after_sequence is None or snapshot.sequence > after_sequence
            ):
                if not require_complete or all(
                    group.data_valid for group in snapshot.groups
                ):
                    return snapshot
            error = self.fatal_error
            if error is not None:
                raise RuntimeError(f"Gello reader stopped: {error}") from error
            self._stop.wait(0.005)
        raise TimeoutError(f"no matching Gello snapshot within {timeout_s:.3f}s")

    def capture_zero(self, group: int = -1) -> None:
        self._validate_group(group)
        self._commands.put(_Command("capture_zero", group))

    def toggle_gripper_calibration(self, group: int) -> None:
        self._validate_group(group, allow_all=False)
        self._commands.put(_Command("toggle_gripper_calibration", group))

    def set_gripper_calibration(self, group: int, minimum: int, maximum: int) -> None:
        self._validate_group(group, allow_all=False)
        if minimum >= maximum:
            raise ValueError("gripper calibration must satisfy min < max")
        self._commands.put(
            _Command("set_gripper_calibration", group, minimum, maximum)
        )

    @staticmethod
    def _validate_group(group: int, *, allow_all: bool = True) -> None:
        valid = (-1, 0, 1) if allow_all else (0, 1)
        if group not in valid:
            raise ValueError(f"group must be one of {valid}")

    def _read_position(self, servo_id: int) -> int | None:
        # Clear before every attempt, including jump retries and reconnects.
        self._velocity_samples.pop(servo_id, None)
        self._sample_times.pop(servo_id, None)
        port = self._serial
        if port is None:
            raise RuntimeError("serial port is not open")
        request = (
            build_position_velocity_request(servo_id)
            if self.read_velocity else build_position_request(servo_id)
        )
        written = port.write(request)
        if written != len(request):
            raise OSError(f"wrote {written} of {len(request)} request bytes")

        deadline = time.perf_counter() + self.timeout_s
        while not self._stop.is_set() and time.perf_counter() < deadline:
            packet, discarded = pop_status_packet(self._rx_buffer)
            self._frame_errors += discarded
            if packet is not None:
                try:
                    parser = (
                        parse_position_velocity_response
                        if self.read_velocity else parse_position_response
                    )
                    response = parser(
                        packet, reject_error_status=False
                    )
                except ChecksumError:
                    self._checksum_errors += 1
                    continue
                except ProtocolError:
                    self._frame_errors += 1
                    continue
                if response.servo_id != servo_id:
                    self._unexpected_ids += 1
                    continue
                if response.status:
                    self._status_errors += 1
                self._sample_times[servo_id] = time.perf_counter()
                if self.read_velocity and not response.status:
                    self._velocity_samples[servo_id] = response.raw_velocity
                # SMS/STS present position uses bit 15 as a sign bit.  Map
                # signed and multi-turn values onto the circular 12-bit
                # encoder range before rollover tracking.
                return response.raw_position % TICKS_PER_REVOLUTION

            waiting = port.in_waiting
            chunk = port.read(min(max(waiting, 1), 256))
            if chunk:
                self._rx_buffer.extend(chunk)

        if not self._stop.is_set():
            self._servo_timeouts += 1
            self._node_timeouts[servo_id] += 1
        return None

    def _read_stable_position(self, group: int, index: int) -> int | None:
        servo_id = GROUP_SERVO_IDS[group][index]
        raw = self._read_position(servo_id)
        if raw is None:
            return None
        tracker = self._trackers[group][index]
        if not tracker.initialized:
            tracker.initialized = True
            tracker.previous_raw = raw
            tracker.unwrapped = raw
            return raw

        delta = wrapped_tick_delta(raw, tracker.previous_raw)
        if abs(delta) > self.max_delta_ticks:
            self._jump_retries += 1
            retry = self._read_position(servo_id)
            if retry is None:
                self._jump_rejects += 1
                return None
            retry_delta = wrapped_tick_delta(retry, tracker.previous_raw)
            readings_agree = abs(wrapped_tick_delta(retry, raw)) <= self.max_delta_ticks
            if abs(retry_delta) <= self.max_delta_ticks:
                raw = retry
                delta = retry_delta
            elif readings_agree:
                raw = retry
                delta = retry_delta
            else:
                self._jump_rejects += 1
                return None

        tracker.unwrapped += delta
        tracker.previous_raw = raw
        return raw

    def _drain_commands(self) -> list[_Command]:
        commands: list[_Command] = []
        while True:
            try:
                commands.append(self._commands.get_nowait())
            except queue.Empty:
                return commands

    def _apply_commands(
        self,
        commands: list[_Command],
        raw_by_group: dict[int, list[int | None]],
    ) -> None:
        for command in commands:
            groups = range(2) if command.group == -1 else (command.group,)
            for group in groups:
                if command.kind == "capture_zero":
                    current = raw_by_group.get(group)
                    if current is not None and all(raw is not None for raw in current):
                        for tracker in self._trackers[group]:
                            tracker.zero_offset = tracker.unwrapped
                        self._zeroed[group] = True
                elif command.kind == "toggle_gripper_calibration":
                    calibration = self._gripper_calibration[group]
                    if calibration.active:
                        calibration.active = False
                        calibration.done = (
                            calibration.minimum is not None
                            and calibration.maximum is not None
                            and calibration.minimum < calibration.maximum
                        )
                        calibration.aligned = calibration.done
                    else:
                        calibration.minimum = None
                        calibration.maximum = None
                        calibration.active = True
                        calibration.done = False
                        calibration.aligned = True
                elif command.kind == "set_gripper_calibration":
                    calibration = self._gripper_calibration[group]
                    calibration.minimum = command.minimum
                    calibration.maximum = command.maximum
                    calibration.active = False
                    calibration.done = True
                    calibration.aligned = False
                    tracker = self._trackers[group][-1]
                    if tracker.initialized:
                        self._align_gripper_calibration(
                            calibration, tracker.unwrapped
                        )

    @staticmethod
    def _align_gripper_calibration(
        calibration: _GripperCalibration, current: int
    ) -> None:
        if calibration.minimum is None or calibration.maximum is None:
            return
        midpoint = (calibration.minimum + calibration.maximum) / 2.0
        turns = round((current - midpoint) / TICKS_PER_REVOLUTION)
        shift = turns * TICKS_PER_REVOLUTION
        calibration.minimum += shift
        calibration.maximum += shift
        calibration.aligned = True

    def _capture_group(
        self, group: int, raw_positions: list[int | None], captured_at: float
    ) -> GroupSnapshot:
        trackers = self._trackers[group]
        current_valid = [raw is not None for raw in raw_positions]
        if not self._zeroed[group] and all(current_valid):
            for tracker in trackers:
                tracker.zero_offset = tracker.unwrapped
            self._zeroed[group] = True

        calibration = self._gripper_calibration[group]
        gripper_raw = raw_positions[-1]
        gripper_unwrapped = trackers[-1].unwrapped if gripper_raw is not None else None
        if calibration.active and gripper_unwrapped is not None:
            calibration.minimum = (
                gripper_unwrapped
                if calibration.minimum is None
                else min(calibration.minimum, gripper_unwrapped)
            )
            calibration.maximum = (
                gripper_unwrapped
                if calibration.maximum is None
                else max(calibration.maximum, gripper_unwrapped)
            )
        elif (
            calibration.done
            and not calibration.aligned
            and gripper_unwrapped is not None
        ):
            self._align_gripper_calibration(calibration, gripper_unwrapped)

        unwrapped: list[int | None] = []
        radians: list[float | None] = []
        for index, raw in enumerate(raw_positions):
            if raw is None:
                unwrapped.append(None)
                radians.append(None)
                continue
            tracker = trackers[index]
            unwrapped.append(tracker.unwrapped)
            zeroed_ticks = tracker.unwrapped - tracker.zero_offset
            radians.append(
                SERVO_SIGNS[index]
                * SERVO_GEARS[index]
                * zeroed_ticks
                * RADIANS_PER_TICK
            )

        normalized: float | None = None
        if (
            gripper_unwrapped is not None
            and (calibration.done or calibration.active)
            and calibration.minimum is not None
            and calibration.maximum is not None
        ):
            if calibration.maximum == calibration.minimum:
                normalized = 0.0
            else:
                normalized = (gripper_unwrapped - calibration.minimum) / (
                    calibration.maximum - calibration.minimum
                )
                normalized = min(max(normalized, 0.0), 1.0)

        raw_velocities = tuple(
            self._velocity_samples.get(servo_id) if valid else None
            for servo_id, valid in zip(GROUP_SERVO_IDS[group], current_valid)
        )
        radians_per_unit = self.velocity_rpm_per_unit * 2.0 * math.pi / 60.0
        arm_velocities = tuple(
            None if value is None else
            value * radians_per_unit * SERVO_SIGNS[index] * SERVO_GEARS[index]
            for index, value in enumerate(raw_velocities[:ARM_SERVOS_PER_GROUP])
        )
        return GroupSnapshot(
            group=group,
            servo_ids=GROUP_SERVO_IDS[group],
            raw_positions=tuple(raw_positions),
            unwrapped_ticks=tuple(unwrapped),
            arm_radians=tuple(radians[:ARM_SERVOS_PER_GROUP]),
            gripper_radians=(
                None
                if normalized is None
                else normalized * GRIPPER_TRAVEL_RADIANS
            ),
            gripper_normalized=normalized,
            gripper_calib_min_raw=calibration.minimum,
            gripper_calib_max_raw=calibration.maximum,
            gripper_calib_active=calibration.active,
            gripper_calib_done=calibration.done,
            zeroed=self._zeroed[group],
            data_valid=all(current_valid),
            captured_at=captured_at,
            raw_velocities=raw_velocities,
            arm_velocities_rad_s=arm_velocities,
            gripper_servo_velocity_rad_s=(
                None if raw_velocities[-1] is None
                else raw_velocities[-1] * radians_per_unit
            ),
            velocity_valid=all(value is not None for value in raw_velocities),
            servo_captured_at=tuple(
                self._sample_times.get(servo_id) if valid else None
                for servo_id, valid in zip(GROUP_SERVO_IDS[group], current_valid)
            ),
        )

    def _scan_once(self) -> tuple[bool, bool]:
        started = time.perf_counter()
        commands = self._drain_commands()
        raw_by_group: dict[int, list[int | None]] = {}
        for group in range(2):
            if not (self.group_mask & (1 << group)):
                continue
            values: list[int | None] = []
            for index in range(SERVOS_PER_GROUP):
                if self._stop.is_set():
                    break
                values.append(self._read_stable_position(group, index))
            if len(values) == SERVOS_PER_GROUP:
                raw_by_group[group] = values

        if self._stop.is_set():
            return False, False
        self._apply_commands(commands, raw_by_group)
        captured_at = time.perf_counter()
        groups = tuple(
            self._capture_group(group, raw_by_group[group], captured_at)
            for group in sorted(raw_by_group)
        )
        duration = captured_at - started
        complete = bool(groups) and all(group.data_valid for group in groups)
        any_response = any(
            raw is not None for group in groups for raw in group.raw_positions
        )

        with self._lock:
            self._scans += 1
            if complete:
                self._complete_scans += 1
            else:
                self._incomplete_scans += 1
            self._last_scan_s = duration
            self._scan_times.append(captured_at)
            self._scan_durations.append(duration)
            self._snapshot = ReaderSnapshot(
                groups=groups,
                captured_at=captured_at,
                sequence=self._scans,
                scan_duration_s=duration,
            )

        remaining = self.scan_interval_s - (time.perf_counter() - started)
        if remaining > 0:
            self._stop.wait(remaining)
        return complete, any_response

    def _close_worker_port(self) -> None:
        port = self._serial
        self._serial = None
        if port is not None:
            port.close()

    def _fail(self, error: Exception) -> None:
        with self._lock:
            self._fatal_error = error
            self._set_connection_locked(
                ConnectionState.FAILED,
                last_error=str(error),
            )

    def _wait_to_reconnect(
        self, error: Exception, delay_s: float, attempt: int
    ) -> bool:
        self._set_connection(
            ConnectionState.RECONNECT_WAIT,
            reconnect_attempt=attempt,
            last_error=str(error),
            retry_at=time.perf_counter() + delay_s,
        )
        return self._stop.wait(delay_s)

    def _run(self) -> None:
        backoff_s = self.reconnect_initial_s
        retry_streak = 0
        reconnecting = False
        try:
            while not self._stop.is_set():
                if reconnecting:
                    with self._lock:
                        self._reconnect_attempts += 1
                self._set_connection(
                    ConnectionState.CONNECTING,
                    reconnect_attempt=retry_streak,
                )
                try:
                    self._serial = self._open_port()
                    self._rx_buffer.clear()
                    self._set_connection(
                        ConnectionState.CONNECTED,
                        reconnect_attempt=retry_streak,
                    )
                    last_valid_response_at = time.perf_counter()
                    connection_confirmed = False

                    while not self._stop.is_set():
                        complete, any_response = self._scan_once()
                        now = time.perf_counter()
                        if any_response:
                            last_valid_response_at = now
                        elif now - last_valid_response_at >= self.link_loss_s:
                            raise BusSilenceError(
                                f"no valid servo response for {self.link_loss_s:.3f}s"
                            )

                        if complete and not connection_confirmed:
                            connection_confirmed = True
                            if reconnecting:
                                with self._lock:
                                    self._successful_reconnects += 1
                            reconnecting = False
                            retry_streak = 0
                            backoff_s = self.reconnect_initial_s
                            self._set_connection(ConnectionState.CONNECTED)
                except Exception as error:
                    self._close_worker_port()
                    if self._stop.is_set():
                        break
                    if (
                        not self.auto_reconnect
                        or not _is_recoverable_transport_error(error)
                    ):
                        self._fail(error)
                        break

                    reconnecting = True
                    retry_streak += 1
                    if self._wait_to_reconnect(error, backoff_s, retry_streak):
                        break
                    backoff_s = min(backoff_s * 2.0, self.reconnect_max_s)
                else:
                    self._close_worker_port()
        finally:
            self._close_worker_port()
            if self.connection_status.state != ConnectionState.FAILED:
                self._set_connection(ConnectionState.DISCONNECTED)
