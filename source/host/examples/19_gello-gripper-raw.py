"""Read only GELLO ID17 for manual open/closed endpoint calibration."""
import argparse
import time

import serial

from gello_compare_driver.protocol import (
    ProtocolError, build_position_velocity_request,
    parse_position_velocity_response, pop_status_packet,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM5')
    parser.add_argument('--baudrate', type=int, default=1000000)
    parser.add_argument('--seconds', type=float, default=0, help='0: run until Ctrl+C')
    args = parser.parse_args()
    if args.seconds < 0:
        parser.error('--seconds must be nonnegative')
    start = time.perf_counter()
    minimum = maximum = None
    valid = errors = timeouts = 0
    print(f'{args.port} @ {args.baudrate} | ID17 only | READ ONLY | Ctrl+C to stop', flush=True)
    print('Move gripper to OPEN and CLOSED; note raw_ticks at each endpoint.', flush=True)
    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.03, write_timeout=0.1) as port:
            while not args.seconds or time.perf_counter() - start < args.seconds:
                cycle = time.perf_counter()
                port.reset_input_buffer()
                port.write(build_position_velocity_request(17))
                buffer = bytearray()
                response = None
                deadline = cycle + 0.10
                while time.perf_counter() < deadline and response is None:
                    buffer.extend(port.read(max(1, port.in_waiting)))
                    while True:
                        packet, _ = pop_status_packet(buffer)
                        if packet is None:
                            break
                        try:
                            response = parse_position_velocity_response(packet, expected_id=17)
                            break
                        except ProtocolError:
                            errors += 1
                if response is None:
                    timeouts += 1
                    print(f'ID17 NO FRESH RESPONSE | timeouts={timeouts} errors={errors}', flush=True)
                else:
                    raw = response.raw_position
                    minimum = raw if minimum is None else min(minimum, raw)
                    maximum = raw if maximum is None else max(maximum, raw)
                    valid += 1
                    print(f'ID17 raw_ticks={raw:5d} | raw_speed={response.raw_velocity:6d}'
                          f' | observed_min={minimum} max={maximum}', flush=True)
                time.sleep(max(0, 0.1 - (time.perf_counter() - cycle)))
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError) as exc:
        print(f'Serial error: {exc}\nClose other GELLO programs using this port.', flush=True)
        return 1
    print(f'Done: valid={valid} timeouts={timeouts} errors={errors}; '
          f'observed_min={minimum} max={maximum}. Endpoints have NOT been changed.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
