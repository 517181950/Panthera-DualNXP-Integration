"""Replay the recorded stationary arm through both C and Python settling checks."""
import ctypes as C
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'source/host'))
import numpy as np
from home_path import Still, UPRIGHT


class State(C.Structure):
    _fields_ = [('velocity', C.c_float * 6), ('low', C.c_float * 6),
                ('high', C.c_float * 6), ('updated', C.c_uint64),
                ('since', C.c_uint64), ('initialized', C.c_bool)]


library = ROOT / 'evidence/home-still.so'
subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared',
                '-fPIC', str(ROOT / 'source/src/homing/home_still.c'),
                '-lm', '-o', str(library)], check=True)
lib = C.CDLL(str(library))
lib.home_still_update.argtypes = [C.POINTER(State), C.POINTER(C.c_float),
                                C.POINTER(C.c_float), C.c_uint64, C.c_bool]
lib.home_still_update.restype = C.c_bool
rows = [json.loads(line) for line in
        (ROOT / 'evidence/upright-right-passive-500hz.jsonl').read_text().splitlines()]
state, python = State(), Still()
start = rows[0]['header']['tx_monotonic_us']
first = None
for row in rows:
    now = row['header']['tx_monotonic_us']
    q, dq = row['q_rad'], row['dq_rad_s']
    position_ok = bool(np.max(np.abs(np.array(q) - UPRIGHT)) <= np.deg2rad(2))
    actual = lib.home_still_update(C.byref(state), (C.c_float * 6)(*q),
                                  (C.c_float * 6)(*dq), now, position_ok)
    expected = python.update(q, dq, now / 1e6, position_ok)
    assert actual == expected, (now, actual, expected)
    if actual and first is None:
        first = (now - start) / 1e6
assert first is not None and first < 1
report = dict(recorded_samples=len(rows), C_Python_all_decisions_match=True,
              first_stationary_after_seconds=first,
              position_span_limit_deg=.5, velocity_filter_ms=20)
(ROOT / 'evidence/upright-still-replay.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
