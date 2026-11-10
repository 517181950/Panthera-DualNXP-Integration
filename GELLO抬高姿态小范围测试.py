"""基于用户指定的抬高姿态，先对ID1做±10度测试，其他关节保持卸力。"""
from pathlib import Path
import GELLO逐轴行程测试 as sweep

IDS=[1]
HALF_RANGE_DEG=10.0
POSE=Path(__file__).resolve().parent/'logs/gello_clearance_pose_20261109_211430_763507/pose.json'

if __name__=='__main__':
    sweep.IDS=IDS
    sweep.CLEARANCE_POSE=POSE
    sweep.LOCAL_SWEEP_DEG=HALF_RANGE_DEG
    raise SystemExit(sweep.run())
