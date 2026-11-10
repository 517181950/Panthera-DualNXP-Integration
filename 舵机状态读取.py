"""直接运行，只读舵机状态，不发送运动指令。"""
from tools.servo_travel_calibration import main

if __name__ == '__main__':
    raise SystemExit(main([]))
