"""所有参数在源码中设置，不需要命令行参数。"""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
URDF=ROOT/'source/models/gello_hd1910/gello_hd1910.urdf'
MAPPING=ROOT/'gello-urdf-mapping.json'
TORQUE_CALIBRATION=ROOT/'gello-current-torque.json'
STATIC_SAMPLES=ROOT/'logs/gello_static_identification.jsonl'
PORT=None
HZ=50.
PASSIVE_IDS=[1]  # 竖直J1重力矩为0，读取角度但不使能/下发位置
CURRENT_LIMIT_MA=1800.
CURRENT_STOP_MA=2000.
MAX_TEMPERATURE_C=50.
MAX_FEEDBACK_AGE_S=.15
# 六个值分别对应URDF joint1..joint6；单位SI。
VIRTUAL_MASS=[.15]*6
DAMPING=[1.5]*6
TORQUE_DEADBAND_NM=[.02]*6
MAX_SPEED_DEG_S=[10.]*6
MAX_ACCEL_DEG_S2=[30.]*6
MAX_TARGET_OFFSET_DEG=[3.]*6
CURRENT_FILTER_SECONDS=.08
# 单位：URDF base_link坐标系下的m/s²；须与实际安装方向一致。
GRAVITY_BASE=[0.,0.,-9.81]
# 只用于记录模型坐标对应点：把机构摆成下面的URDF角度再运行记录脚本。
KNOWN_URDF_Q_DEG=[0.]*6
GRAVITY_DIRECTION_CONFIRMED=True  # 用户确认底座竖直，base Z朝上
# True表示采样时没有人手承托/外力，只能在确认后修改。
STATIC_SAMPLE_NO_EXTERNAL_FORCE=False
