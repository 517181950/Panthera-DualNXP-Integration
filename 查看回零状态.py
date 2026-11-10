"""直接运行：只观察两臂回零阶段，不触发回零；触发仍用任一板SW2。"""
from manual_entry import run_program
from 运行参数 import DIAGNOSTIC_SECONDS

SCRIPT = 'tools/watch-local-home.py'
ARGUMENTS = ['--seconds', DIAGNOSTIC_SECONDS, '--output', 'evidence/manual-home-watch.json']

if __name__ == '__main__':
    run_program(SCRIPT, ARGUMENTS)
