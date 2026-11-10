"""运行：python 读取电机角度.py；先退出遥操，Ctrl+C 结束。

只开启反馈查询，不申请控制、不使能、不回零、不发送运动目标。
显示电机实际编码器角度（度），没有减去 GELLO 零点或整圈偏移。
"""
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent / 'source' / 'host'))
from robot_sdk import Robot

BOARDS = [
    ('新臂 left ', 11, (5005, 5006)),
    ('原臂 right', 12, (5015, 5016)),
]
REFRESH_SECONDS = 0.5


def main():
    robots = []
    querying = []
    try:
        for name, node, ports in BOARDS:
            r = Robot(board=f'192.168.10.{node}', board_node=node, ports=ports)
            robots.append((name, r))
            if any(sock is None for sock in r.socks):
                raise RuntimeError(f'{name}：无法占用读取端口，请先退出遥操或其他状态读取程序')
            r.start()
            if not r.discover(3):
                raise RuntimeError(f'{name}：NXP 未连接，请检查网络')
            result = r.start_state_stream(motor_query_hz=500, hand_query_hz=50,
                                          hand_angles_only=True)
            querying.append(r)
            if result.get('status') != 0:
                raise RuntimeError(f'{name}：开启反馈失败：{result}')
        print('实际编码器角度，单位 °；Ctrl+C 退出\n'
              '时间      机械臂          J1        J2        J3        J4        J5        J6', flush=True)
        while True:
            for name, r in robots:
                s = r.snapshot('ArmState').get('ArmState', {})
                q = s.get('q_rad', [float('nan')]*6)
                ages = s.get('feedback_age_us', [float('inf')]*6)
                host_age = (time.perf_counter() - s.get('host_received_monotonic_s', 0))*1000
                values = []
                for i in range(6):
                    valid = (s.get('valid_q_mask', 0) & (1 << i)
                             and math.isfinite(q[i]) and 0 <= host_age
                             and 0 <= ages[i]/1000 + host_age < 20)
                    values.append(f'{math.degrees(q[i]):9.2f}' if valid else f'{"--":>9}')
                print(f'{time.strftime("%H:%M:%S")}  {name} ' + ' '.join(values), flush=True)
            time.sleep(REFRESH_SECONDS)
    finally:
        for _, r in robots:
            try:
                if r in querying:
                    r.stop_state_stream()
            finally:
                r.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n读取结束。')
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
