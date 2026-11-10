"""Schema revision 7 codec; wire-compatible with the generated WLC C codec."""
from pathlib import Path
import re
import struct

SCHEMA = {}
IDS = {}
for name, ident, body in re.findall(r"message (\w+) @id\((\d+)\) \{(.*?)\}", (Path(__file__).parents[1] / "protocol/robot_messages.wl").read_text(), re.S):
    SCHEMA[name] = [(t, k, int(n or 0), int(i)) for t, k, n, i in re.findall(r"required (?:packed )?(\w+) (\w+)(?:\[(\d+)\])? @id\((\d+)\);", body)]
    IDS[name] = int(ident)
NAMES = {v: k for k, v in IDS.items()}

class ProtocolError(ValueError):
    pass

TABLE = []
for i in range(256):
    c = i
    for _ in range(8):
        c = (c >> 1) ^ (0x82F63B78 if c & 1 else 0)
    TABLE.append(c)

def crc(data):
    value = 0xFFFFFFFF
    for byte in data:
        value = TABLE[(value ^ byte) & 255] ^ (value >> 8)
    return value ^ 0xFFFFFFFF

def vi(value):
    if not isinstance(value, int) or not 0 <= value <= 0xFFFFFFFFFFFFFFFF:
        raise ProtocolError("varint outside unsigned 64-bit representation")
    result = bytearray()
    while value > 127:
        result.append((value & 127) | 128)
        value >>= 7
    result.append(value)
    return bytes(result)

def rv(data, offset):
    result = 0
    for count in range(10):
        if offset >= len(data):
            raise ProtocolError("truncated varint")
        byte = data[offset]
        offset += 1
        if count == 9 and byte > 1:
            raise ProtocolError("overflowing varint")
        result |= (byte & 127) << (7 * count)
        if not byte & 128:
            if count and byte == 0:
                raise ProtocolError("noncanonical varint")
            return result, offset
    raise ProtocolError("unterminated varint")

def zero(name):
    return {key: [0] * count if count else zero(typ) if typ in SCHEMA else 0
            for typ, key, count, _ in SCHEMA[name]}

def _wire(typ, count=0):
    return 2 if count or typ in SCHEMA else 1 if typ == "fixed64" else 5 if typ in ("float32", "fixed32") else 0

def _scalar(typ, value):
    if typ == "float32":
        return struct.pack(">f", value)
    if typ in ("fixed32", "fixed64"):
        return struct.pack(">I" if typ == "fixed32" else ">Q", value)
    if typ == "int32":
        if not isinstance(value, int) or not -(1 << 31) <= value < (1 << 31):
            raise ProtocolError("int32 representation")
        return vi((value << 1) ^ (value >> 31))
    maximum = {"uint8": 255, "uint16": 65535, "uint32": 0xFFFFFFFF, "bool": 1}[typ]
    if not isinstance(value, int) or not 0 <= value <= maximum:
        raise ProtocolError("integer representation")
    return vi(value)

def encode(name, message):
    result = bytearray()
    for typ, key, count, ident in SCHEMA[name]:
        if key not in message:
            raise ProtocolError("missing " + key)
        value = message[key]
        if count:
            if len(value) != count:
                raise ProtocolError("fixed array " + key)
            raw = b"".join(_scalar(typ, item) for item in value)
        elif typ in SCHEMA:
            raw = encode(typ, value)
        else:
            raw = _scalar(typ, value)
        wire = _wire(typ, count)
        result += vi((ident << 3) | wire)
        if wire == 2:
            result += vi(len(raw))
        result += raw
    return bytes(result)

def _value(typ, raw):
    if typ in ("fixed32", "fixed64"):
        return int.from_bytes(raw, "big")
    if typ == "float32":
        return struct.unpack(">f", raw)[0]
    if typ == "int32":
        value = (raw >> 1) ^ -(raw & 1)
        if not -(1 << 31) <= value < (1 << 31):
            raise ProtocolError("signed overflow")
        return value
    maximum = {"uint8": 255, "uint16": 65535, "uint32": 0xFFFFFFFF, "bool": 1}[typ]
    if raw > maximum:
        raise ProtocolError("integer overflow")
    return bool(raw) if typ == "bool" else raw

def decode(name, data):
    offset = 0
    raw_fields = {}
    while offset < len(data):
        tag, offset = rv(data, offset)
        ident, wire = tag >> 3, tag & 7
        if not ident or ident in raw_fields:
            raise ProtocolError("zero/duplicate field")
        if wire == 0:
            value, offset = rv(data, offset)
        elif wire in (1, 2, 5):
            if wire == 2:
                size, offset = rv(data, offset)
            else:
                size = 8 if wire == 1 else 4
            if size > len(data) - offset:
                raise ProtocolError("truncated field")
            value = data[offset:offset + size]
            offset += size
        else:
            raise ProtocolError("unknown wire type")
        raw_fields[ident] = (wire, value)
    result = {}
    for typ, key, count, ident in SCHEMA[name]:
        if ident not in raw_fields:
            raise ProtocolError("missing " + key)
        wire, raw = raw_fields[ident]
        if wire != _wire(typ, count):
            raise ProtocolError("wrong wire type for " + key)
        if count:
            size = 8 if typ == "fixed64" else 4
            if len(raw) != count * size:
                raise ProtocolError("array size for " + key)
            value = [_value(typ, raw[i * size:(i + 1) * size]) for i in range(count)]
        elif typ in SCHEMA:
            value = decode(typ, raw)
        else:
            value = _value(typ, raw)
        result[key] = value
    return result

def frame(mid, payload, session=0, seq=0, kind=0):
    if kind not in (0, 1, 2) or len(payload) > 512 or (kind == 2 and payload):
        raise ProtocolError("frame shape")
    result = struct.pack(">BBH", 0xA4 | kind, 0, mid)
    if kind:
        result += struct.pack(">QI", session, seq)
    result += payload
    return result + struct.pack(">I", crc(result))

def unframe(data):
    if not 8 <= len(data) <= 532 or crc(data[:-4]) != int.from_bytes(data[-4:], "big"):
        raise ProtocolError("frame length/CRC32C")
    if data[0] & 0xFC != 0xA4 or data[1] or data[0] & 3 == 3:
        raise ProtocolError("frame version/kind")
    kind, mid = data[0] & 3, int.from_bytes(data[2:4], "big")
    if kind and len(data) < 20:
        raise ProtocolError("short reliable header")
    session, seq = struct.unpack(">QI", data[4:16]) if kind else (0, 0)
    payload = data[16 if kind else 4:-4]
    if kind == 2 and payload:
        raise ProtocolError("ACK payload")
    return kind, mid, session, seq, payload
