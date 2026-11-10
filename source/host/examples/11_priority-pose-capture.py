"""Three priority poses, three rounds, slow high-gain static capture.

Defaults: 8s move / 4s settle / 3s sample / 2s hold = 153s total.
Uses selected 393.63g gravity model, no friction feedforward during capture.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arm_lab import main

HERE=Path(__file__).resolve().parent
DEFAULTS=dict(plan=str(HERE/'plans'/'priority_three_poses.json'),
              identified=str(HERE/'profiles'/'gravity_link6_full284.json'),
              transition=8.,settle=4.,sample=3.,post_hold=2.,
              kp=[24.48,61.2,61.2,12.24,12.24,12.24],kd=[1.2,1.92,1.92,.48,.48,.48])

if __name__=='__main__':raise SystemExit(main('gravity_capture',defaults=DEFAULTS))
