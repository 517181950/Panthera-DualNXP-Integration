"""直接运行：原手滑条＋原臂重力补偿。会同时控制该侧机械臂补偿，并非只动手。"""
from manual_entry import run_program
from 运行参数 import arm_arguments, HAND_MODE, HAND_CURRENT_MA, HAND_FRICTION_SCALE

SIDE = 'right'
SCRIPT = 'source/host/examples/16_gravity-five-finger-grasp.py'
ARGUMENTS = [*arm_arguments(SIDE), '--hand-mode', HAND_MODE,
             '--hand-current-ma', HAND_CURRENT_MA,
             '--friction-scale', HAND_FRICTION_SCALE]

if __name__ == '__main__':
    run_program(SCRIPT, ARGUMENTS)
