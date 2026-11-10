"""50 poses from the historical 285; 1s move/settle/sample/post-hold.

Fit only link6 with total hand mass 624 +/- 30 g. J1 torque excluded.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arm_lab import main

HERE=Path(__file__).resolve().parent
DEFAULTS=dict(plan=str(HERE/'plans/random50_legacy285_seed20261030.json'),
    identified=str(HERE/'profiles/gravity_link6_fixed624_seed293.json'),
    transition=1.,settle=1.,sample=1.,post_hold=1.,
    kp=[24.48,61.2,61.2,12.24,12.24,12.24],kd=[1.2,1.92,1.92,.48,.48,.48],
    fit_scope='link6',link6_mass=.624,link6_mass_tolerance=.030)

if __name__=='__main__':raise SystemExit(main('gravity_capture',defaults=DEFAULTS))
