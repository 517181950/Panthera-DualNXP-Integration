"""直接运行：只读原臂NXP J11连接的GELLO七个节点，不启动遥操。"""
from manual_entry import run_program
from 运行参数 import DIAGNOSTIC_SECONDS

SCRIPT = 'tools/watch-board-gello.py'
ARGUMENTS = ['--seconds', DIAGNOSTIC_SECONDS, '--output', 'evidence/manual-gello-read.json']

if __name__ == '__main__':
    run_program(SCRIPT, ARGUMENTS)
