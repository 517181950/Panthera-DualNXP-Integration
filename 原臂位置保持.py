"""直接运行：原臂以启动时实测姿态为目标运行阻抗保持。退出时沿用现有保持清理。"""
from manual_entry import run_program
from 运行参数 import arm_arguments

SIDE = 'right'
SCRIPT = 'source/host/examples/4_impedance-control.py'
ARGUMENTS = arm_arguments(SIDE)

if __name__ == '__main__':
    run_program(SCRIPT, ARGUMENTS)
