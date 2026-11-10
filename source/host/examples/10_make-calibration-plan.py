"""Generate an inspectable pose plan offline; never opens the robot connection."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import argparse
import itertools
from arm_dynamics import write_json
from arm_calibration_plan import legacy285


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--preset',choices=['grid243','legacy285'],default='grid243')
    p.add_argument('--center-deg',type=float,nargs=6,default=[0,45,45,0,0,0])
    p.add_argument('--span-deg',type=float,default=15)
    p.add_argument('--output',required=True)
    a=p.parse_args()
    if Path(a.output).exists():p.error('Choose a new output file; existing plan is preserved')
    if a.preset=='legacy285':poses=legacy285()
    else:poses=[[a.center_deg[0]]+[v+d for v,d in zip(a.center_deg[1:],offset)]
                for offset in itertools.product((0,a.span_deg,-a.span_deg),repeat=5)]
    write_json(a.output,dict(preset=a.preset,poses_deg=poses,
               source='legacy285 reproduces 2026-10-29 USB dual-arm pose list; current tool geometry and zero mapping must be evaluated separately'))
    print(f'{len(poses)} poses: {Path(a.output).resolve()}')
    return 0


if __name__=='__main__':raise SystemExit(main())
