#!/usr/bin/env python3
"""只读检查 Gello 串口连接：只发送位置/速度读取请求，不发送任何动作指令。"""
from pathlib import Path
import time

from gello.config import load_config
from gello.reader import GROUP_SERVO_IDS, GelloReader


def main() -> int:
    config_path = Path(__file__).with_name("gello_config.json")
    cfg = load_config(config_path)
    s = cfg.serial
    print(f"配置：{config_path}")
    print(f"串口：{s.port}  波特率：{s.baud}  组：0x{s.group_mask:x}")
    print("只读检查，不会让 Gello 或机械臂运动。\n")

    print(f"速度读取：{s.read_velocity}  单位：{s.velocity_rpm_per_unit:g} RPM/原始值")
    reader = GelloReader(
        port=s.port,
        baudrate=s.baud,
        group_mask=s.group_mask,
        timeout_s=s.timeout_ms / 1000.0,
        scan_interval_s=s.scan_interval_ms / 1000.0,
        max_delta_ticks=s.max_delta_ticks,
        auto_reconnect=s.auto_reconnect,
        link_loss_s=s.link_loss_ms / 1000.0,
        reconnect_initial_s=s.reconnect_initial_ms / 1000.0,
        reconnect_max_s=s.reconnect_max_ms / 1000.0,
        read_velocity=s.read_velocity,
        velocity_rpm_per_unit=s.velocity_rpm_per_unit,
    )
    reader.start()
    try:
        time.sleep(3.0)
        status = reader.connection_status
        stats = reader.stats
        print(f"连接状态：{status.state}")
        print(f"串口已打开：{reader.is_connected}")
        if status.last_error:
            print(f"最后错误：{status.last_error}")
        print(
            "扫描次数：{}  完整：{}  不完整：{}  扫描频率：{:.1f} Hz".format(
                stats.scans, stats.complete_scans, stats.incomplete_scans, stats.scan_hz
            )
        )
        print(
            "舵机超时：{}  校验错：{}  帧错：{}  状态错：{}  未知ID：{}".format(
                stats.servo_timeouts,
                stats.checksum_errors,
                stats.frame_errors,
                stats.status_errors,
                stats.unexpected_ids,
            )
        )

        snapshot = reader.latest_snapshot()
        if snapshot is None:
            print("没有收到任何快照。")
        else:
            print(f"最新快照序号：{snapshot.sequence}")
            for group in snapshot.groups:
                raw = tuple("-" if value is None else value for value in group.raw_positions)
                print(f"  group {group.group}: 数据有效={group.data_valid} 原始位置={raw}")
                print(f"    速度有效={group.velocity_valid} 原始速度={group.raw_velocities}")
                print(f"    六关节速度 rad/s={group.arm_velocities_rad_s}")
                print(f"    夹爪舵机轴速度 rad/s={group.gripper_servo_velocity_rad_s}")

        valid_groups = [group.group for group in snapshot.groups if group.data_valid] if snapshot else []

        if stats.complete_scans == 0:
            if valid_groups:
                print(f"\n结论：部分连接。group {valid_groups} 已有完整数据，但还有一组缺少舵机响应。")
                for group in snapshot.groups:
                    if group.data_valid:
                        continue
                    missing = [
                        servo_id
                        for servo_id, value in zip(
                            GROUP_SERVO_IDS[group.group], group.raw_positions
                        )
                        if value is None
                    ]
                    print(f"  group {group.group} 无响应的舵机 ID：{missing}")
                print("请检查对应手臂的舵机电源和串联信号线，尤其是第一个无响应舵机之前的连接。")
            else:
                print("\n结论：串口能打开，但没有收到任何 Gello 舵机数据。")
                print("请检查 Gello 电源、USB-RS485 接线（A/B、GND）、串口是否选对，")
                print("以及总线上是否还有另一个主机在占用。")
            return 2

        print("\n结论：已收到完整的 Gello 舵机数据，连接正常。")
        return 0
    finally:
        reader.stop()


if __name__ == "__main__":
    raise SystemExit(main())
