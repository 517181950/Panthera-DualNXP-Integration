"""双GELLO分别控制左右臂/手；运行后沿用原有回零流程，再等待各自主手固定零位。
Ctrl+C请求两侧NXP柔顺保持。接线与独立标定见 双GELLO遥操说明.txt。
"""
from manual_entry import run_dual_gello_teleop
from 运行参数 import TELEOP_SECONDS

if __name__ == '__main__':
    run_dual_gello_teleop(TELEOP_SECONDS)
