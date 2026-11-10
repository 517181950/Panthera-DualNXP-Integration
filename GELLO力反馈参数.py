"""无命令行参数；原臂 + USB HD1910M。方向由现场逐轴核对后保存。"""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SIDE='right'
CALIBRATION=ROOT/'logs/gello_reference_zero_20261109_210927_565834/calibration.json'
DIRECTIONS=ROOT/'gello-haptic-directions.json'
PORT=None
CURRENT_LIMIT_MA=100.0
CURRENT_STOP_MA=1800.0  # 短时峰值停止，不是持续输出额定值
CURRENT_RMS_MA=900.0  # 以7.4V额定值为参考的试验保护，不代表8V额定认证
CURRENT_RMS_SECONDS=.5
ARM_KP=[24.48,61.2,61.2,12.24,12.24,12.24]
ARM_KD=[1.2,1.92,1.92,.48,.48,.48]
ARM_SPEED_DEG_S=5.0
FEEDBACK_STRENGTH=.2
FEEDBACK_OFFSET_DEG=3.0
RUN_SECONDS=0.0  # 0=持续运行，Ctrl+C或异常保护停止
ARM_HZ=200
SERVO_HZ=50
SERVO_MAX_AGE_S=.06
COMMAND_TIMEOUT_S=.05
# ID1–6控制机械臂，ID7控制灵巧手开合并提供位置耦合反馈。

# 2026-11-09五秒测试结束后核实：ID5需-4096偏移，ID6模式切换已恢复整圈计数；仅软件坐标转换，不写舵机零位。
# 当前读数须仍落入唯一的人工行程区间；断电/重装后重新核对。
RAW_FRAME_OFFSETS=[0,0,0,0,-4096,0,0]  # 2026-11-10抬升测试切回模式0后，ID6整圈计数已恢复

HAND_ENABLED=True
TRIGGER_OPEN_TICKS=291
TRIGGER_CLOSED_TICKS=-292
HAND_TARGET_SPEED_DEG_S=None  # 不添加目标速度限制，沿用20ms更新
