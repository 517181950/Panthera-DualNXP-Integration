from __future__ import annotations

import errno
import os
import pty
import select
import threading
import time
import unittest

from gello.protocol import build_position_velocity_request, checksum
from gello.reader import GelloReader


@unittest.skipUnless(os.name == "posix", "pseudo terminals require POSIX")
class PySerialIntegrationTests(unittest.TestCase):
    def test_reader_scans_a_real_pyserial_pseudo_terminal(self) -> None:
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        os.close(slave_fd)
        stop = threading.Event()
        requests: list[int] = []
        packets: list[bytes] = []

        def emulator() -> None:
            buffer = bytearray()
            while not stop.is_set():
                ready, _, _ = select.select((master_fd,), (), (), 0.02)
                if not ready:
                    continue
                try:
                    chunk = os.read(master_fd, 256)
                except OSError as error:
                    if error.errno == errno.EIO:
                        time.sleep(0.001)
                        continue
                    raise
                buffer.extend(chunk)
                while len(buffer) >= 8:
                    header_at = buffer.find(b"\xff\xff")
                    if header_at < 0:
                        buffer.clear()
                        break
                    if header_at:
                        del buffer[:header_at]
                    if len(buffer) < 8:
                        break
                    request = bytes(buffer[:8])
                    del buffer[:8]
                    servo_id = request[2]
                    requests.append(servo_id)
                    packets.append(request)
                    raw = 1200 + servo_id
                    response = bytearray(
                        (
                            0xFF,
                            0xFF,
                            servo_id,
                            0x06,
                            0x00,
                            raw & 0xFF,
                            (raw >> 8) & 0xFF,
                            servo_id,
                            0x80,
                            0x00,
                        )
                    )
                    response[-1] = checksum(response[2:-1])
                    os.write(master_fd, response[:3])
                    os.write(master_fd, response[3:])

        emulator_thread = threading.Thread(target=emulator, daemon=True)
        emulator_thread.start()
        reader = GelloReader(
            slave_name,
            group_mask=0x1,
            timeout_s=0.02,
            scan_interval_s=0.01,
        )
        try:
            reader.start()
            snapshot = reader.wait_for_snapshot(1.0, require_complete=True)
        finally:
            reader.stop()
            stop.set()
            emulator_thread.join(timeout=1.0)
            os.close(master_fd)

        self.assertEqual(requests[:7], list(range(1, 8)))
        self.assertEqual(snapshot.group(0).raw_positions, tuple(range(1201, 1208)))
        self.assertEqual(packets[:7], [build_position_velocity_request(i) for i in range(1, 8)])
        self.assertEqual(snapshot.group(0).raw_velocities, tuple(-i for i in range(1, 8)))
        self.assertTrue(snapshot.group(0).velocity_valid)
        self.assertIsNone(reader.fatal_error)
