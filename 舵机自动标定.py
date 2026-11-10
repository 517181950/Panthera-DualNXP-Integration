"""直接运行即按下方预设标定；不询问参数。导入此文件不会驱动舵机。"""
from tools.servo_travel_calibration import main

# ===== 用户配置：需要调整时只修改这里 =====
PORT = None                 # None：自动识别唯一USB适配器；也可填写串口路径
SERVO_ID = 1
BAUD = 1000000
DIRECTION = 1               # 1：编码器增大为上限；-1：减小为上限
ZERO_MODE = 'current'       # current：当前位置为零点；lower：先搜索下挡块
MAX_TRAVEL_DEG = 30.0       # 每段最大搜索角度，单位度
TIMEOUT_S = 45.0            # 每段搜索超时，单位秒
CURRENT_LIMIT_MA = 150.0    # 驱动限流，实际值按6.5mA刻度向下取整
CONTACT_MA = 100.0          # 疑似触边电流阈值
MAX_CURRENT_MA = 200.0      # 紧急停止电流阈值
MARGIN_DEG = 2.0            # 运行范围预留余量
# =========================================


def run():
    print(f'HD1910M 自动标定：ID={SERVO_ID}，上限方向={DIRECTION}，'
          f'零点方式={ZERO_MODE}，每段最大行程={MAX_TRAVEL_DEG}°', flush=True)
    args = ['--execute', '--id', str(SERVO_ID), '--baud', str(BAUD),
            '--direction', str(DIRECTION), '--zero', ZERO_MODE,
            '--max-travel-deg', str(MAX_TRAVEL_DEG), '--timeout', str(TIMEOUT_S),
            '--current-limit-ma', str(CURRENT_LIMIT_MA), '--contact-ma', str(CONTACT_MA),
            '--max-current-ma', str(MAX_CURRENT_MA), '--margin-deg', str(MARGIN_DEG)]
    if PORT is not None:
        args.extend(['--port', str(PORT)])
    return main(args)


if __name__ == '__main__':
    raise SystemExit(run())
