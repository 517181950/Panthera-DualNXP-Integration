"""直接运行程序的参数；修改后重新启动对应 Python 文件即可。

right = 原臂 node12；left = 新臂 node11。
这里只保存运行参数，不连接硬件。GELLO固定零点和动力学标定继续引用
已有标定文件，不重新置零。下面的增益、软限位及手参数用于新独立遥操入口。
旧 run-one-gello-two-arms.py 命令行入口继续读取原 JSON 配置。
"""

GELLO_SOURCE = "nxp-usb"  # GELLO -> 原臂 NXP J11；微雪B档
TELEOP_SECONDS = 0       # 0持续运行；Ctrl+C请求所选机械臂由NXP持续柔顺保持
DIAGNOSTIC_SECONDS = 30
HAND_CURRENT_MA = 500
HAND_MODE = 3
HAND_FRICTION_SCALE = 0.0

TELEOP_CONFIG = {'local': '192.168.10.1',
 'require_hands': False,  # 手未就绪时只跳过该手，机械臂继续；本次运行不自动接入
 'hz': 500,
 'soft_limits_file': 'soft-limits.json',
 'gello': {'port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB0182526-if00',
           'group': 0,
           'config': 'gello-single-arm.json'},
 'boards': {'left': {'node': 11,
                     'ip': '192.168.10.11',
                     'ports': [5005, 5006],
                     'kp': [4, 10, 10, 2, 2, 1],
                     'kd': [0.5, 0.8, 0.8, 0.2, 0.2, 0.1],
                     'hand': True,
                     'identified': 'source/host/examples/profiles/gravity_selected.json',
                     'homing_gains': 'original_calibrated',
                     'role': 'new_arm',
                     'encoder_modulo_joints': [2],
                     'soft_limits': {'reference_filter': False,  # 直接位置跟随；只截断角度，不启用下方轨迹限速
                                     'lower_deg': [-20, 0, 0, -90, -100, -160],
                                     'upper_deg': [20, 180, 240.64227395494575, 90, 100, 160],
                                     'velocity_deg_s': [15, 15, 15, 15, 15, 15],
                                     'acceleration_deg_s2': [30, 30, 30, 30, 30, 30],
                                     'exponent': 6,
                                     'target_gain_s': 4,
                                     'feedback_age_ms': 20,
                                     'max_period_ms': 20,
                                     'prediction_ms': 20},
                     'hand_control': {'profile': 'five_finger',
                                      'current_ma': 500,
                                      'grasp_scale': 1.0,
                                      'grasp_offset_percent': 0.0,
                                      'grasp_inverted': False,
                                      'joint_scale': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                                      'joint_offset_deg': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'max_target_speed_deg_s': None,
                                      'feedback_timeout_ms': 100.0}},
            'right': {'node': 12,
                      'ip': '192.168.10.12',
                      'ports': [5015, 5016],
                      'kp': [24.48, 61.2, 61.2, 12.24, 12.24, 12.24],
                      'kd': [1.2, 1.92, 1.92, 0.48, 0.48, 0.48],
                      'hand': True,
                      'identified': 'source/host/examples/profiles/gravity_selected.json',
                      'homing_gains': 'original_calibrated',
                      'role': 'original_arm',
                      'encoder_modulo_joints': [2],
                      'soft_limits': {'reference_filter': False,  # 直接位置跟随；只截断角度，不启用下方轨迹限速
                                     'lower_deg': [-20, 0, 0, -90, -100, -160],
                                      'upper_deg': [20, 180, 240.64227395494575, 90, 100, 160],
                                      'velocity_deg_s': [15, 15, 15, 15, 15, 15],
                                      'acceleration_deg_s2': [30, 30, 30, 30, 30, 30],
                                      'exponent': 6,
                                      'target_gain_s': 4,
                                      'feedback_age_ms': 20,
                                      'max_period_ms': 20,
                                      'prediction_ms': 20},
                      'hand_control': {'profile': 'five_finger',
                                       'current_ma': 500,
                                       'grasp_scale': 1.0,
                                       'grasp_offset_percent': 0.0,
                                       'grasp_inverted': False,
                                       'joint_scale': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                                       'joint_offset_deg': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                       'max_target_speed_deg_s': None,
                                       'feedback_timeout_ms': 100.0}}}}

def board_arguments(side):
    board = TELEOP_CONFIG['boards'][side]
    return ['--local', TELEOP_CONFIG['local'], '--board', board['ip'],
            '--board-node', board['node'], '--ports', *board['ports']]


def arm_arguments(side):
    board = TELEOP_CONFIG['boards'][side]
    return [*board_arguments(side), '--identified', board['identified'],
            '--kp', *board['kp'], '--kd', *board['kd'],
            '--hz', TELEOP_CONFIG['hz'], '--seconds', 0, '--execute']


# 双GELLO独立入口。profile_group是标定文件中的组名，不是机械臂侧别。
# 原有GELLO使用gello-single-arm.json的left标定，继续控制原臂right。
# 第二个GELLO为同总线IDs11–17；group=1，必须单独标定，不能复制现有零位。
from copy import deepcopy
DUAL_GELLO_CONFIG = deepcopy(TELEOP_CONFIG)
DUAL_GELLO_CONFIG.pop('gello')
DUAL_GELLO_CONFIG['gellos'] = {
    'left': {'transport': 'nxp-usb', 'board': '192.168.10.12', 'node': 12,
             'group': 1, 'profile_group': 'left', 'config': 'gello-left.json'},
    'right': {'transport': 'nxp-usb', 'board': '192.168.10.12', 'node': 12,
              'group': 0, 'profile_group': 'left', 'config': 'gello-single-arm.json'},
}
