"""Source SDK 0.2: nonfatal transport, typed target APIs and passive state subscription.

No trajectory, speed, position, torque or slew limiter is implemented here.
An active device protection is reported; it is not silently cleared or bypassed.
"""
from collections import Counter, OrderedDict, deque
from dataclasses import dataclass
import copy
import errno
import math
import secrets
import select
import socket
import struct
import threading
import time
from robot_protocol import IDS, NAMES, ProtocolError, zero, encode, decode, frame, unframe
from robot_faults import describe

__version__ = "0.3.0"

@dataclass(frozen=True)
class SendResult:
    ok: bool
    sequence: int = 0
    error: str = ""

class Robot:
    def __init__(self, local="192.168.10.1", board="192.168.10.11", *, ports=(5005, 5006), peer_ports=None, host_node=10, board_node=11):
        if any(type(n) is not int or not 1 <= n <= 65535 for n in (host_node, board_node)) or host_node == board_node:
            raise ValueError("Host and board node IDs must be distinct integers in 1..65535")
        self.host_node, self.board_node = host_node, board_node
        socket.inet_aton(local)
        socket.inet_aton(board)
        self.local, self.board = local, board
        self.ports = tuple(ports)
        self.peer_ports = tuple(peer_ports if peer_ports is not None else ports)
        self.boot = secrets.randbits(64) or 1
        self.board_boot = self.lease = self.sequence = 0
        self.policy = 1
        self.socks = [None, None]
        self.count, self.stats, self.gaps = Counter(), Counter(), Counter()
        self.last, self.seen_seq = {}, {}
        self.errors, self.events = deque(maxlen=512), deque(maxlen=512)
        self._wire_seen, self._responses = OrderedDict(), OrderedDict()
        self._lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._closed = False
        self._next_bind = 0.0
        self.packet_recorder = None
        self.config = zero("ConfigureRequest")
        self.config.update(hand_mode=2, arm_state_period_us=2000, hand_state_period_us=10000,
                           force_feedback_gain=[1.0] * 6, hand_position_current_limit_ma=[1000]*6)
        self._ensure_sockets()

    def _note(self, kind, detail):
        with self._lock:
            self.stats[kind] += 1
            self.errors.append({"time": time.monotonic(), "kind": kind, "detail": str(detail)[:240]})

    def _ensure_sockets(self):
        if self._closed or time.monotonic() < self._next_bind:
            return
        self._next_bind = time.monotonic() + 1.0
        for index, port in enumerate(self.ports):
            if self.socks[index] is not None:
                continue
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
                sock.bind((self.local, port))
                sock.setblocking(False)
                self.socks[index] = sock
            except OSError as exc:
                if sock is not None:
                    sock.close()
                self._note("bind_error", exc)

    def header(self):
        with self._lock:
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            if not self.sequence:
                self.sequence = 1
            return dict(schema_revision=7, source_node=self.host_node, target_node=self.board_node, source_boot_id=self.boot,
                        target_boot_id=self.board_boot, sequence=self.sequence,
                        tx_monotonic_us=time.perf_counter_ns() // 1000)

    def _send(self, index, data):
        self._ensure_sockets()
        sock = self.socks[index]
        if sock is None:
            self._note("send_unavailable", "socket unavailable; other SDK work continues")
            return False
        try:
            if sock.sendto(data, (self.board, self.peer_ports[index])) != len(data):
                raise OSError("short UDP send")
            if self.packet_recorder:
                self.packet_recorder(0, self.peer_ports[index], data)
            self.stats["sent"] += 1
            return True
        except OSError as exc:
            self._note("send_error", exc)
            return False

    def receive(self, timeout=0):
        """Receive pushed packets; malformed packets never terminate the loop."""
        packets = []
        with self._io_lock:
            self._ensure_sockets()
            sockets = [sock for sock in self.socks if sock is not None]
            if not sockets:
                if timeout:
                    self._stop.wait(min(timeout, 0.05))
                return packets
            try:
                readable = select.select(sockets, [], [], timeout)[0]
            except (OSError, ValueError) as exc:
                self._note("select_error", exc)
                return packets
            for sock in readable:
                index = self.socks.index(sock)
                for _ in range(2048):
                    try:
                        data, peer = sock.recvfrom(2048)
                    except BlockingIOError:
                        break
                    except OSError as exc:
                        self._note("receive_error", exc)
                        if exc.errno in (errno.EBADF, errno.ENOTSOCK, 10038):
                            sock.close()
                            self.socks[index] = None
                            self._next_bind = 0
                        break
                    if self.packet_recorder:
                        self.packet_recorder(1, self.peer_ports[index], data)
                    if peer != (self.board, self.peer_ports[index]):
                        self._note("foreign_peer", peer)
                        continue
                    try:
                        kind, mid, session, wire_seq, payload = unframe(data)
                        if kind == 2:
                            self.stats["ack_received"] += 1
                            continue
                        if kind == 1:
                            self._send(index, frame(0, b"", session, wire_seq, 2))
                            key = (session, wire_seq, mid)
                            if key in self._wire_seen:
                                self.stats["reliable_duplicate"] += 1
                                continue
                            self._wire_seen[key] = None
                            if len(self._wire_seen) > 256:
                                self._wire_seen.popitem(last=False)
                        name = NAMES.get(mid)
                        if name is None:
                            self._note("unknown_message", mid)
                            continue
                        message = decode(name, payload)
                        h = message["header"]
                        if h["schema_revision"] != 7 or h["source_node"] != self.board_node or h["target_node"] != self.host_node or not h["source_boot_id"]:
                            self._note("invalid_identity", h)
                            continue
                        with self._lock:
                            new_boot = h["source_boot_id"] != self.board_boot
                            if new_boot:
                                if self.board_boot:
                                    self._note("board_restarted", "lease invalidated; no automatic activate")
                                self.board_boot = h["source_boot_id"]
                                self.lease = 0
                                self.seen_seq.clear()
                            if self.lease and h["target_boot_id"] not in (0, self.boot):
                                self._note("foreign_session", h["target_boot_id"])
                                continue
                            if name in ("ArmState", "HandState", "FaultSnapshot", "Diagnostics", "FaultEvent", "PeerArmState"):
                                old = self.seen_seq.get(name)
                                if old is not None:
                                    delta = (h["sequence"] - old) & 0xFFFFFFFF
                                    if not delta:
                                        self.stats["stream_duplicate"] += 1
                                        continue
                                    if delta >= 0x80000000:
                                        self.stats["stream_old"] += 1
                                        continue
                                    if delta > 1:
                                        self.gaps[name] += delta - 1
                                self.seen_seq[name] = h["sequence"]
                            if name in ("ArmState", "HandState"):
                                # Local receipt time is separate from MCU feedback
                                # ages, which otherwise freeze when UDP stops.
                                message["host_received_monotonic_s"] = time.perf_counter()
                            self.last[name] = message
                            self.count[name] += 1
                            if name == "ArmState":
                                self.policy = message["policy_revision"] if new_boot else max(self.policy, message["policy_revision"])
                            if name in ("OperationResult", "ConfigureResult"):
                                self._responses[(name, message["operation_id"])] = message
                                if len(self._responses) > 64:
                                    self._responses.popitem(last=False)
                            if name == "FaultEvent":
                                if len(self.events) == self.events.maxlen:
                                    self.stats["local_event_overflow"] += 1
                                self.events.append(message)
                            packets.append((name, message))
                    except (ProtocolError, ValueError, KeyError, IndexError, OverflowError) as exc:
                        self._note("malformed_packet", exc)
                        continue
                    except Exception as exc:
                        # Record an unexpected decoder problem; preserve subsequent packets.
                        self._note("decoder_internal_error", repr(exc))
                        continue
        return packets

    def discover(self, timeout=8):
        end = time.monotonic() + timeout
        while not self.board_boot and time.monotonic() < end and not self._closed:
            if self._thread and self._thread.is_alive():
                self._stop.wait(0.01)
            else:
                self.receive(min(0.05, max(0, end - time.monotonic())))
        if not self.board_boot:
            self._note("discover_timeout", "no telemetry; receiver may keep retrying")
        return bool(self.board_boot)

    def _rpc(self, request_name, response_name, request, timeout=3):
        ident = request["operation_id"]
        try:
            packet = frame(IDS[request_name], encode(request_name, request), self.boot, request["header"]["sequence"], 1)
        except (ValueError, KeyError, OverflowError, TypeError, struct.error) as exc:
            return {"operation_id": ident, "status": -errno.EINVAL, "reason": str(exc), "outcome_unknown": False}
        end, next_send, sent = time.monotonic() + timeout, 0.0, False
        while time.monotonic() < end and not self._closed:
            now = time.monotonic()
            if now >= next_send:
                sent = self._send(1, packet) or sent
                next_send = now + 0.1
            if self._thread and self._thread.is_alive():
                self._stop.wait(0.002)
            else:
                self.receive(0.01)
            with self._lock:
                result = self._responses.pop((response_name, ident), None)
            if result is not None:
                if result["status"]:
                    self._note("operation_rejected", result)
                return result
        self._note("rpc_timeout", (request_name, ident))
        return {"operation_id": ident, "status": -errno.ETIMEDOUT, "reason": "response timeout", "outcome_unknown": sent}

    def operation(self, op, *, timeout=3, operation_id=None, target_mask=0):
        request = zero("OperationRequest")
        request.update(header=self.header(), operation=op, target_mask=target_mask, control_lease=self.lease, expected_policy_revision=self.policy)
        request["operation_id"] = self.sequence if operation_id is None else operation_id
        result = self._rpc("OperationRequest", "OperationResult", request, timeout)
        if result["status"] == 0:
            with self._lock:
                self.lease = result["control_lease"]
                self.policy = result["applied_policy_revision"]
        return result

    def start_state_stream(self, *, trace_can=False, hand_query_hz=500, motor_query_hz=500, hand_angles_only=False, hand_current_fast=False, hand_velocity_fast=False, hand_full_state=False, legacy_full_checksum=False, hand_reply_driven=False, timeout=3):
        """500Hz state publication only. No lease, active, mode or target changes."""
        if hand_query_hz not in (50,100,200,500,1000,2000) or motor_query_hz not in (0,500,1000):
            return {"status": -errno.EINVAL, "reason": "hand_query_hz must be100/200/500/1000/2000 and motor_query_hz500/1000"}
        request = zero("OperationRequest")
        request.update(header=self.header(), operation=10, operation_id=self.sequence,
                       target_mask=int(trace_can)|(2 if hand_query_hz==100 else 0)|(4 if motor_query_hz==1000 else 0)|(hand_query_hz<<16)|(8 if motor_query_hz==0 else 0)|(32 if hand_angles_only else 0)|(64 if hand_current_fast else 0)|(128 if hand_velocity_fast else 0)|(256 if hand_full_state else 0)|(512 if hand_full_state and legacy_full_checksum else 0)|(1024 if hand_reply_driven else 0), expected_policy_revision=self.policy)
        return self._rpc("OperationRequest", "OperationResult", request, timeout)

    def stop_state_stream(self, *, timeout=1):
        """Restore low-rate hand queries only; no actuator or mode changes."""
        request = zero("OperationRequest")
        request.update(header=self.header(), operation=11, operation_id=self.sequence,
                       expected_policy_revision=self.policy)
        return self._rpc("OperationRequest", "OperationResult", request, timeout)

    def send_state_probe(self):
        """A StreamHeader-only diagnostic message; cannot encode joint targets."""
        header = self.header()
        if not self.board_boot:
            return SendResult(False, header["sequence"], "waiting for board identity")
        ok = self._send(0, frame(IDS["StreamHeader"], encode("StreamHeader", header)))
        return SendResult(ok, header["sequence"], "" if ok else "UDP send failed")

    def disable_joints(self, joint_mask, *, timeout=3):
        """Explicit operator request: disable selected drives, no automatic policy."""
        return self.operation(9, target_mask=joint_mask, timeout=timeout)

    def set_hand_mode(self, mode, *, timeout=3, allow_unconfirmed_hand=False, keep_j4_disabled=False):
        """Explicit physical mode request; acceptance is not an ACK or readback."""
        if mode not in (1, 2, 3):
            return {"status": -errno.EINVAL, "reason": "mode must be1..3"}
        return self.operation(8, target_mask=mode|(0x100 if allow_unconfirmed_hand else 0)|(0x200 if keep_j4_disabled else 0), timeout=timeout)

    def prepare_hand_mode(self, mode=2, *, timeout=3):
        """Explicit startup step after claim: explicit matching fresh mode readback; negative ACK remains a failure.
        The monitor never calls this. It sends no actuator target or arm command.
        """
        result = self.set_hand_mode(mode, timeout=timeout, allow_unconfirmed_hand=True)
        if result.get("status"):
            return result
        token = result["operation_id"]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.snapshot()
            diag = state.get("Diagnostics", {})
            if diag.get("hand_mode_guard_token") == token:
                if diag.get("hand_mode_guard_phase") == 8:
                    return dict(status=diag.get("hand_mode_guard_status", -errno.EIO), operation_id=token, reason="hand mode negative ACK or TX failure")
                if diag.get("hand_mode_guard_phase") == 7 and state.get("HandState", {}).get("mode_raw") == mode:
                    return dict(status=0, operation_id=token, confirmed_mode=mode, elapsed_us=diag.get("hand_mode_guard_elapsed_us"))
            time.sleep(.005)
        return dict(status=-errno.ETIMEDOUT, operation_id=token, reason="fresh mode readback not confirmed")

    def configure(self, *, timeout=3, **settings):
        allowed = set(self.config) - {"header", "operation_id", "expected_policy_revision", "new_policy_revision"}
        if set(settings) - allowed:
            return {"status": -errno.EINVAL, "reason": "unknown configuration fields", "outcome_unknown": False}
        request = copy.deepcopy(self.config)
        request.update(settings)
        request.update(header=self.header(), operation_id=self.sequence, expected_policy_revision=self.policy, new_policy_revision=self.policy + 1)
        result = self._rpc("ConfigureRequest", "ConfigureResult", request, timeout)
        if result["status"] == 0:
            self.config = request
            self.policy = result["applied_policy_revision"]
        return result

    @staticmethod
    def _six(values):
        values = [float(value) for value in values]
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("six finite numeric fields required")
        return values

    def _target(self, name, fields):
        try:
            request = zero(name)
            request.update(header=self.header(), control_lease=self.lease, policy_revision=self.policy, valid_for_us=0, **fields)
            if not self.lease:
                return SendResult(False, request["header"]["sequence"], "claim required")
            data = frame(IDS[name], encode(name, request))
            ok = self._send(0, data)
            return SendResult(ok, request["header"]["sequence"], "" if ok else "UDP send failed")
        except (ValueError, TypeError, KeyError, OverflowError, struct.error) as exc:
            self._note("target_encoding_error", exc)
            return SendResult(False, self.sequence, str(exc))

    def send_arm(self, q, dq, kp, kd, tau_ff, *, joint_mask=63):
        """SI units; all five vectors are explicit; no clipping or gain defaults."""
        try:
            vectors = [self._six(v) for v in (q, dq, kp, kd, tau_ff)]
            return self._target("ArmCommand", dict(joint_mask=joint_mask, mit=[x for joint in zip(*vectors) for x in joint]))
        except (ValueError, TypeError) as exc:
            self._note("target_encoding_error", exc)
            return SendResult(False, self.sequence, str(exc))

    def send_hand(self, q, duration_ms):
        """Six active joint angles; physical mode is preset by the user."""
        try:
            return self._target("HandCommand", dict(target_q_rad=self._six(q), duration_ms=duration_ms))
        except (ValueError, TypeError) as exc:
            self._note("target_encoding_error", exc)
            return SendResult(False, self.sequence, str(exc))

    def arm_packet(self):
        request = zero("ArmCommand")
        request.update(header=self.header(), control_lease=self.lease, policy_revision=self.policy, joint_mask=63, valid_for_us=0)
        return frame(IDS["ArmCommand"], encode("ArmCommand", request))

    def snapshot(self, *names):
        """Copy requested state types, or all types when no names are given."""
        with self._lock:
            return copy.deepcopy({name:self.last[name] for name in names if name in self.last} if names else self.last)

    def poll_fault_events(self):
        with self._lock:
            result = list(self.events)
            self.events.clear()
        return [(event, describe(event["device_class"], event["device_id"], event["device_channel"], event["raw_code"])) for event in result]

    def _run(self):
        while not self._stop.is_set():
            try:
                self.receive(0.02)
            except Exception as exc:
                self._note("worker_error", repr(exc))
                self._stop.wait(0.01)

    def start(self):
        if not self._closed and not (self._thread and self._thread.is_alive()):
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="robot-udp-receiver", daemon=True)
            self._thread.start()
        return self

    def close(self):
        """Explicitly close local resources; never implicitly brake/clear a device."""
        self._closed = True
        self._stop.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2)
        for sock in self.socks:
            if sock is not None:
                sock.close()
        self.socks = [None, None]
