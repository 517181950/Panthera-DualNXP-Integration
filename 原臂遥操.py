"""直接运行：GELLO通过原臂NXP J11控制原臂＋原手。会回零并进入跟随。
Ctrl+C请求所选机械臂停止驱动，退出前支撑好；SW2仍触发两臂回零。
"""
from manual_entry import run_teleop
from 运行参数 import TELEOP_SECONDS

SIDE = 'right'
SECONDS = TELEOP_SECONDS

if __name__ == '__main__':
    run_teleop(SIDE, SECONDS)
