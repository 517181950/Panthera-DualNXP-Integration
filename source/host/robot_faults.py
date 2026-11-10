"""Fault interpretation is descriptive. No device stop/clear calls exist here."""
from dataclasses import dataclass
import errno

ARM = {
    0: "正常", 1: "DMA数据流传输错误", 2: "DMA FIFO错误", 3: "UART溢出", 4: "UART帧错误",
    5: "UART噪声", 6: "UART缓冲溢出", 7: "UART奇偶校验错误", 32: "校准故障",
    33: "电机驱动故障", 34: "过压", 35: "编码器故障", 36: "电机未校准",
    37: "PWM周期过限", 38: "温度过高", 39: "起始位置超限", 40: "电压过低",
    41: "配置已更改", 42: "换相角度无效", 43: "输出位置无效", 44: "驱动器使能故障",
    45: "停止位置使用错误", 46: "时序错误", 47: "反电动势前馈错误",
}
HAND = {
    0: "正常", 1: "电机过温告警", 2: "电机过温保护", 3: "电机低压保护",
    4: "电机过压保护", 5: "电机过流保护", 6: "电机力矩保护", 7: "电机熔丝位错误保护",
    8: "电机堵转保护", 9: "驱动器异常保护", 10: "电机Hall错误保护",
}

@dataclass(frozen=True)
class FaultInfo:
    device_class: int
    device_id: int
    channel: int
    raw_code: int
    known: bool
    name: str
    protection_indicated: object = None

def describe(device_class, device_id, channel, raw):
    raw = int(raw) & 0xFFFFFFFF
    if device_class == 2 and channel >= 0x80:
        name = "手指令被拒绝" if raw == 0 else "手指令确认成功" if raw == 1 else f"未知手确认状态 0x{raw:08X}"
        return FaultInfo(device_class, device_id, channel, raw, raw in (0, 1), name, False)
    if device_class in (1, 2):
        catalog = ARM if device_class == 1 else HAND
        return FaultInfo(device_class, device_id, channel, raw, raw in catalog,
                         catalog.get(raw, f"未知故障 0x{raw:08X}"), (2 <= raw <= 10) if device_class == 2 and raw in HAND else False if raw == 0 else None)
    if device_class == 0 and channel == 1:
        states = {0: "CAN error-active", 1: "CAN error-warning", 2: "CAN error-passive", 3: "CAN bus-off", 4: "CAN stopped"}
        return FaultInfo(device_class, device_id, channel, raw, raw in states, states.get(raw, f"未知CAN状态 {raw}"))
    signed = raw - (1 << 32) if raw >= (1 << 31) else raw
    label = errno.errorcode.get(abs(signed))
    name = label or f"原始通信状态 0x{raw:08X}"
    return FaultInfo(device_class, device_id, channel, raw, label is not None, name)
