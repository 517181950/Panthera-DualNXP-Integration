"""位置模式短时抬升：1.5A停止，不按固定跟随角度退出；最多5秒。"""
import GELLO位置模式回姿态 as motion

motion.CURRENT_LIMIT_MA=1500.0
motion.CURRENT_STOP_MA=1500.0
motion.FOLLOW_TICKS=None
motion.TOTAL_ENABLED_SECONDS=5.0

if __name__=='__main__':raise SystemExit(motion.run())
