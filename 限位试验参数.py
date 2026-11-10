"""重力补偿限位试验参数；角度上下限统一读取运行参数.py。

这些是未做实机验证的试验初值，不代表已辨识制动能力或硬限位保证。
max_torque_nm 限制前馈和按反馈估算的总力矩，不是电机瞬时输出硬限幅；取当前URDF的effort值，
不是已验证的电机持续输出能力。静止在边界及越界处请求上限向内力矩；
运动时叠加双向速度阻尼，回弹时减力/制动。边界判断及过渡只按角度。
"""
from copy import deepcopy

_COMMON = {
    'damping_nm_s_rad': [12, 20, 28, 10, 5, 4], # 电机本地MIT Kd；模型估计初值，需实机验证
    'max_torque_nm': [21, 36, 36, 21, 10, 10], # Assets/PantheraHT_v4_hand620g.urdf effort
    'cushion_deg': [3.0]*6,          # 最后3°平滑增强，静止在边界时达到上限
    'damping_zone_deg': [4.5]*6,     # 最后4.5°渐入，距边界3°时角度权重全额
    'low_speed_damping_ratio': 0.25, # 静止附近保留25%阻尼，减轻慢速拖动黏滞
    'full_damping_speed_deg_s': 20.0, # 速度绝对值达到20°/s恢复原有阻尼，双向制动
    'startup_ramp_s': 1.0,          # 限位力矩渐入，重力补偿不等待
}
WALL_SETTINGS = {'left': deepcopy(_COMMON), 'right': deepcopy(_COMMON)}
