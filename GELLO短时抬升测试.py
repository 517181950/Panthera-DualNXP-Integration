"""8V供电下最多5秒抬升；1500mA停止。并非持续额定值。"""
import GELLO全使能姿态遍历 as motion

motion.STOP_CURRENT_MA = 1500.0
motion.LIFT_ONLY = True
motion.FOLLOW_ERROR_TICKS = 34  # 约3度，仅放宽短时抬升测试
motion.HOLD_SECONDS = 2.0
motion.OVER_RATED_BUDGET_SECONDS = None
motion.TEST_DURATION_SECONDS = 5.0

if __name__ == '__main__':
    raise SystemExit(motion.run())
