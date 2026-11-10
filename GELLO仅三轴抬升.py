"""其余轴保持使能时的位置，仅ID3沿此前抬高方向试动5度；总时限5秒。"""
import GELLO全使能姿态遍历 as motion

motion.STOP_CURRENT_MA = 1500.0
motion.LIFT_ONLY = True
motion.FOLLOW_ERROR_TICKS = 34
motion.HOLD_SECONDS = 1.0
motion.OVER_RATED_BUDGET_SECONDS = None
motion.TEST_DURATION_SECONDS = 5.0
motion.SINGLE_AXIS = 3
motion.SINGLE_AXIS_DELTA_DEG = 5.0

if __name__ == '__main__':
    raise SystemExit(motion.run())
