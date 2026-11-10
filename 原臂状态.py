"""直接运行：只查询原臂及末端手的状态，不发送运动目标。"""
from manual_entry import run_program
from 运行参数 import board_arguments, DIAGNOSTIC_SECONDS

SIDE = 'right'
SCRIPT = 'source/host/examples/1_robot-state.py'
ARGUMENTS = [*board_arguments(SIDE), '--motor-query-hz', 500,
             '--hand-query-hz', 50, '--hand-angles-only',
             '--seconds', DIAGNOSTIC_SECONDS,
             '--report', 'evidence/manual-right-state.json']

if __name__ == '__main__':
    run_program(SCRIPT, ARGUMENTS)
