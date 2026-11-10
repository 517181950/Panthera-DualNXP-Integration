"""Compare compiled MCU float gravity against the actual Pinocchio host model."""
from pathlib import Path
import ctypes as C
import json
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'source/host'))
from arm_dynamics import Gravity
model=Gravity(identified=ROOT/'source/host/examples/profiles/gravity_selected.json')
f=C.CDLL(str(ROOT/'evidence/home-gravity.so')).home_gravity
A=C.c_float*6;f.argtypes=[A,A];f.restype=None
rng=np.random.default_rng(947);poses=rng.uniform(-np.pi,np.pi,(10000,6)).astype(np.float32)
poses=np.vstack((np.zeros(6,dtype=np.float32),poses))
maximum=np.zeros(6)
for q in poses:
    out=A();f(A(*q),out);error=np.abs(np.array(out)-model(q));maximum=np.maximum(maximum,error)
assert maximum.max()<1e-5,maximum
report=dict(poses=len(poses),max_abs_error_nm=maximum.tolist(),passed=True,reference=model.identity)
(ROOT/'evidence/local-home-gravity-comparison.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
