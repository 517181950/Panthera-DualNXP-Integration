"""七轴当前位置保持试验；参数预设，不移动到历史姿态。"""
import GELLO位置模式回姿态 as entry
entry.HOLD_CURRENT_POSITION=True
entry.CURRENT_LIMIT_MA=1200.0
entry.CURRENT_STOP_MA=1800.0
entry.HOLD_SECONDS=None
entry.TOTAL_ENABLED_SECONDS=None
if __name__=='__main__':raise SystemExit(entry.run())
