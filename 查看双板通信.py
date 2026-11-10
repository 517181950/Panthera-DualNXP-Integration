"""直接运行：只观察双板通信和反馈频率，不发送运动指令。"""
from manual_entry import run_program
from 运行参数 import DIAGNOSTIC_SECONDS

SCRIPT = 'source/host/dual_peer.py'
ARGUMENTS = ['--seconds', DIAGNOSTIC_SECONDS, '--output', 'evidence/manual-peer-state.json']

if __name__ == '__main__':
    run_program(SCRIPT, ARGUMENTS)
