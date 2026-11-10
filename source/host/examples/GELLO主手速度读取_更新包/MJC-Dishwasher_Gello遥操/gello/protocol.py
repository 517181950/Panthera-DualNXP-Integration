"""Protocol helpers for the serial-bus servos used by the Gello arms."""

from __future__ import annotations

from dataclasses import dataclass


HEADER = b"\xff\xff"
READ_INSTRUCTION = 0x02
PRESENT_POSITION_ADDRESS = 56
POSITION_BYTES = 2
MIN_STATUS_LENGTH = 2
MAX_STATUS_LENGTH = 32


class ProtocolError(ValueError):
    """Base class for malformed servo packets."""


class ChecksumError(ProtocolError):
    """The status packet checksum does not match its contents."""


class ServoStatusError(ProtocolError):
    """The servo returned a nonzero error status."""

    def __init__(self, servo_id: int, status: int) -> None:
        super().__init__(f"servo {servo_id} returned status 0x{status:02x}")
        self.servo_id = servo_id
        self.status = status


@dataclass(frozen=True)
class PositionResponse:
    servo_id: int
    raw_position: int
    status: int


@dataclass(frozen=True)
class PositionVelocityResponse:
    servo_id: int
    raw_position: int
    raw_velocity: int
    status: int


def checksum(payload: bytes | bytearray | memoryview) -> int:
    """Return the protocol-1 one's-complement checksum for *payload*."""
    return (~sum(payload)) & 0xFF


def decode_signed_magnitude(value: int, sign_bit: int) -> int:
    """Decode the sign-magnitude words used by SMS/STS servo registers."""
    if not 0 <= value <= 0xFFFF:
        raise ValueError("servo word must be in the range 0..65535")
    if not 0 <= sign_bit <= 15:
        raise ValueError("sign_bit must be in the range 0..15")
    sign_mask = 1 << sign_bit
    magnitude = value & (sign_mask - 1)
    return -magnitude if value & sign_mask else magnitude


def build_position_request(servo_id: int) -> bytes:
    """Build a two-byte present-position read request."""
    return _build_read_request(servo_id, POSITION_BYTES)


def build_position_velocity_request(servo_id: int) -> bytes:
    """Read position (56) and velocity (58) together, without writing registers."""
    return _build_read_request(servo_id, 4)


def _build_read_request(servo_id: int, count: int) -> bytes:
    if not 1 <= servo_id <= 0xFD:
        raise ValueError("servo_id must be in the range 1..253")
    packet = bytearray(
        (
            0xFF,
            0xFF,
            servo_id,
            0x04,
            READ_INSTRUCTION,
            PRESENT_POSITION_ADDRESS,
            count,
            0x00,
        )
    )
    packet[-1] = checksum(packet[2:-1])
    return bytes(packet)


def parse_position_response(
    packet: bytes,
    *,
    expected_id: int | None = None,
    reject_error_status: bool = True,
) -> PositionResponse:
    """Validate and decode one two-byte present-position status packet."""
    servo_id, status = _validate_response(
        packet, POSITION_BYTES, expected_id, reject_error_status
    )
    return PositionResponse(
        servo_id, decode_signed_magnitude(packet[5] | (packet[6] << 8), 15), status
    )


def parse_position_velocity_response(
    packet: bytes,
    *,
    expected_id: int | None = None,
    reject_error_status: bool = True,
) -> PositionVelocityResponse:
    """Decode little-endian SMS/STS position and speed sign-magnitude words."""
    servo_id, status = _validate_response(packet, 4, expected_id, reject_error_status)
    return PositionVelocityResponse(
        servo_id=servo_id,
        raw_position=decode_signed_magnitude(packet[5] | (packet[6] << 8), 15),
        raw_velocity=decode_signed_magnitude(packet[7] | (packet[8] << 8), 15),
        status=status,
    )


def _validate_response(
    packet: bytes, parameter_bytes: int, expected_id: int | None,
    reject_error_status: bool,
) -> tuple[int, int]:
    if len(packet) < 6 or packet[:2] != HEADER:
        raise ProtocolError("invalid status packet header")
    length = packet[3]
    if not MIN_STATUS_LENGTH <= length <= MAX_STATUS_LENGTH:
        raise ProtocolError(f"invalid status packet length {length}")
    if len(packet) != length + 4:
        raise ProtocolError(
            f"status packet has {len(packet)} bytes, expected {length + 4}"
        )
    if checksum(packet[2:-1]) != packet[-1]:
        raise ChecksumError("status packet checksum mismatch")

    servo_id = packet[2]
    if expected_id is not None and servo_id != expected_id:
        raise ProtocolError(
            f"received servo {servo_id}, expected servo {expected_id}"
        )
    status = packet[4]
    if status and reject_error_status:
        raise ServoStatusError(servo_id, status)
    if length != parameter_bytes + 2:
        raise ProtocolError(
            f"response has length field {length}, expected {parameter_bytes + 2}"
        )
    return servo_id, status


def pop_status_packet(buffer: bytearray) -> tuple[bytes | None, int]:
    """Pop one framed packet from a rolling receive buffer.

    Returns ``(packet, discarded_bytes)``. A ``None`` packet means more input
    is required. Invalid length bytes are discarded here so the next valid
    header can be found without resetting the serial input buffer.
    """
    discarded = 0
    while True:
        header_at = buffer.find(HEADER)
        if header_at < 0:
            keep = 1 if buffer.endswith(b"\xff") else 0
            discarded += len(buffer) - keep
            if keep:
                buffer[:] = b"\xff"
            else:
                buffer.clear()
            return None, discarded
        if header_at:
            del buffer[:header_at]
            discarded += header_at
        if len(buffer) < 4:
            return None, discarded

        length = buffer[3]
        if not MIN_STATUS_LENGTH <= length <= MAX_STATUS_LENGTH:
            del buffer[0]
            discarded += 1
            continue
        packet_size = length + 4
        if len(buffer) < packet_size:
            return None, discarded
        packet = bytes(buffer[:packet_size])
        del buffer[:packet_size]
        return packet, discarded
