"""直接运行：原臂启用纯重力补偿，可手动拖动；不保持位置。退出时沿用现有保持清理。"""
from manual_entry import run_program
from 运行参数 import arm_arguments

SIDE = 'right'
SCRIPT = 'source/host/examples/3_gravity-compensation.py'
ARGUMENTS = arm_arguments(SIDE)

if __name__ == '__main__':
    run_program(SCRIPT, ARGUMENTS)
