"""Five-finger percentage grasp, with selected arm gravity + 60% friction.

Uncheck Individual finger control for the grasp slider. All stages linear:
0-20%: thumb rotation 0 -> 100 deg, all five bends 0 deg.
20-95%: five bends -> [13, 53, 56, 55, 52] deg; rotation stays 100 deg.
95-100%: each bend increases another 5 deg; rotation stays 100 deg.
Without --execute, opens an offline preview without connecting to hardware.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hand_slider_demo import main

if __name__=='__main__':raise SystemExit(main(gripper_profile='five_finger'))
