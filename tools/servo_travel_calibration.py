"""HD1910M bounded external-stop calibration. Default invocation is read-only.

Tested off-line for identity 3.46 / 10.31, not validated by a live stall test.
Position is a signed full-angle encoder count; calibration uses temporary
multi-turn absolute position mode with bounded host travel. This tool never writes encoder offset,
ID, baud, EEPROM unlock, or factory reset. Calibration is saved as JSON.
"""
from __future__ import annotations
import argparse
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PORT = None
TICKS_PER_DEG = 4096 / 360
CURRENT_UNIT_MA = 6.5
EXPECTED_IDENTITY = bytes([3, 46, 0, 10, 31])
# Address/width whitelist. EEPROM-region writes require lock=1 and are volatile.
WIDTHS = {9:2, 11:2, 13:1, 14:1, 15:1, 19:1, 28:2, 33:1, 38:1,
          40:1, 41:1, 42:2, 44:2, 46:2, 48:2}
RESTORE = (11, 9, 13, 14, 15, 19, 28, 33, 38, 41, 44, 46, 48)


class CalibrationError(RuntimeError):
    pass


def resolve_port(port=None, directory=Path('/dev/serial/by-id')):
    if port:
        if not Path(port).exists():
            raise CalibrationError(f'指定串口不存在：{port}；请检查连接或省略 --port 自动识别')
        return str(port)
    # Only the USB adapter family used for the servo, never NXP debug ports.
    candidates = sorted(str(p) for p in directory.glob('usb-1a86_USB_Single_Serial_*-if00') if p.exists())
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise CalibrationError('未找到舵机USB适配器；请检查连接，或用 --port 指定串口')
    raise CalibrationError('发现多个USB串口，不能自动确定舵机连接；请用 --port 指定：' + ', '.join(candidates))


def checksum(data):
    return (~sum(data)) & 255


def signed(word, bit=15):
    value = word & ((1 << bit)-1)
    return -value if word & (1 << bit) else value


def encode_position(value):
    if not isinstance(value, int) or not -32767 <= value <= 32767:
        raise CalibrationError('目标位置超出带符号位置协议范围')
    return abs(value) | (0x8000 if value < 0 else 0)


@dataclass
class Feedback:
    position: int
    speed_raw: int
    current_ma: float
    voltage_v: float
    temperature_c: int
    status: int
    load_raw: int = 0

    @classmethod
    def decode(cls, data, packet_status=0):
        if len(data) != 15:
            raise CalibrationError('反馈长度不符')
        word = lambda i: int.from_bytes(data[i:i+2], 'little')
        return cls(signed(word(0)), signed(word(2)), signed(word(13))*CURRENT_UNIT_MA,
                   data[6]/10, data[7], packet_status | data[9], signed(word(4),10))


class Bus:
    def __init__(self, port, node=1, baud=1000000, timeout=.025):
        import serial
        if not 0 <= node <= 253:
            raise ValueError('ID必须是0..253，禁止广播')
        busy = subprocess.run(['fuser', str(Path(port).resolve())], capture_output=True)
        if busy.returncode == 0:
            raise CalibrationError('串口正在被FD或其他程序使用，请先关闭其串口')
        self.node, self.timeout = node, timeout
        self.serial = serial.Serial(port, baud, timeout=.002, write_timeout=timeout, exclusive=True)
        self.writes_allowed = False
        from collections import deque
        self.recent_packets = deque(maxlen=12)
        self.serial.reset_input_buffer()

    def exchange(self, instruction, params, reply_length):
        body = bytes([self.node, len(params)+2, instruction])+bytes(params)
        request = b'\xff\xff'+body+bytes([checksum(body)])
        self.serial.reset_input_buffer()
        self.serial.write(request)
        deadline = time.monotonic()+self.timeout
        buffer = bytearray()
        while time.monotonic() < deadline:
            buffer.extend(self.serial.read(self.serial.in_waiting or 1))
            while True:
                at = buffer.find(b'\xff\xff')
                if at < 0:
                    buffer[:] = b'\xff' if buffer.endswith(b'\xff') else b''
                    break
                del buffer[:at]
                if len(buffer) < 4: break
                length = buffer[3]
                if not 2 <= length <= 32:
                    del buffer[0]; continue
                if len(buffer) < length+4: break
                packet = bytes(buffer[:length+4]); del buffer[:length+4]
                if packet == request: continue  # Adapter local echo is not an ACK.
                if packet[2] != self.node or checksum(packet[2:-1]) != packet[-1]: continue
                if len(packet) != reply_length+6: continue
                self.recent_packets.append({"id":self.node,"instruction":instruction,"params":bytes(params).hex(),"reply":packet.hex()})
                return packet[5:-1], packet[4]
        raise CalibrationError(f'ID{self.node}通信超时；不能确认舵机已收到停止命令')

    def read(self, address, length, *, allow_fault=False):
        data, status = self.exchange(2, [address, length], length)
        if status and not allow_fault:
            raise CalibrationError(f'ID{self.node}读取寄存器{address}时状态异常0x{status:02x}（bit0电压、bit1编码器、bit2温度、bit3电流）')
        return data

    def integer(self, address, length, **kwargs):
        return int.from_bytes(self.read(address, length, **kwargs), 'little')

    def feedback(self):
        data, status = self.exchange(2, [56, 15], 15)
        return Feedback.decode(data, status)

    def write(self, address, value, *, stopping=False):
        if not self.writes_allowed or address not in WIDTHS:
            raise CalibrationError('禁止的写入；诊断模式不发送控制指令')
        if address < 40 and self.integer(55, 1) != 1:
            raise CalibrationError('EEPROM锁未锁定，拒绝写入临时参数')
        if stopping and (address, value) != (40, 0):
            raise CalibrationError('故障路径只允许关闭扭矩')
        wire_value = encode_position(value) if address == 42 else value
        payload = wire_value.to_bytes(WIDTHS[address], 'little', signed=False)
        _, status = self.exchange(3, bytes([address])+payload, 0)
        if status and not stopping:
            raise CalibrationError(f'写入应答故障0x{status:02x}')
        observed = self.integer(address, WIDTHS[address], allow_fault=stopping)
        if observed != wire_value:
            raise CalibrationError(f'寄存器{address}回读不符：{observed} != {value}')

    def write_motion(self, position, settings):
        # FD.exe 0x4161a0 -> 0x415ff0: one WRITE of seven bytes at address 41.
        if not self.writes_allowed:
            raise CalibrationError('诊断模式禁止运动指令')
        settings.validate()
        current = math.floor(settings.current_limit_ma/CURRENT_UNIT_MA)
        data = (bytes([1]) + encode_position(position).to_bytes(2,'little')
                + current.to_bytes(2,'little') + settings.speed_raw.to_bytes(2,'little'))
        _, status = self.exchange(3, bytes([41])+data, 0)
        if status:
            raise CalibrationError(f'运动指令应答故障0x{status:02x}')
        if self.read(41,7) != data:
            raise CalibrationError('运动控制区回读不符')

    def disable(self):
        errors = []
        for _ in range(3):
            try:
                self.write(40, 0, stopping=True)
                return
            except Exception as exc:
                errors.append(str(exc))
        raise CalibrationError('无法确认卸力！请切断舵机电源。'+ '; '.join(errors))

    def close(self):
        self.serial.close()


@dataclass
class Settings:
    direction: int = 1
    max_travel_deg: float = 90.0
    timeout_s: float = 45.0
    current_limit_ma: float = 150.0
    contact_ma: float = 100.0
    max_current_ma: float = 200.0
    max_temperature_c: float = 50.0
    torque_percent: float = 10.0
    period_s: float = .02
    contact_s: float = .08
    no_progress_s: float = .5
    margin_deg: float = 2.0
    speed_raw: int = 1  # This firmware family: about 50 ticks/s.
    step_ticks: int = 2
    lead_ticks: int = 12
    min_motion_ticks: int = 8

    def validate(self):
        if any(not math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('参数必须为有限数')
        if self.direction not in (-1,1) or not 1 <= self.max_travel_deg <= 180:
            raise ValueError('direction为±1；本程序每段搜索范围必须在1..180度内')
        if not 1 <= self.timeout_s <= 120:
            raise ValueError('搜索超时必须在1..120秒')
        actual_limit = math.floor(self.current_limit_ma/CURRENT_UNIT_MA)*CURRENT_UNIT_MA
        if not 0 < self.contact_ma < actual_limit <= 300 or not actual_limit < self.max_current_ma <= 400:
            raise ValueError('需满足0 < 触边电流 < 实际限流值 <=300mA；紧急停止阈值须大于限流值且不超过400mA')
        if not 0 < self.torque_percent <= 20 or not 20 <= self.max_temperature_c <= 60:
            raise ValueError('试验转矩占空比上限0..20%；温度上限20..60℃')
        if not .01 <= self.period_s <= .04 or not .04 <= self.contact_s <= .15:
            raise ValueError('无效采样周期或触边确认时间')
        if not self.contact_s < self.no_progress_s <= 1 or not 0 < self.margin_deg < self.max_travel_deg/2:
            raise ValueError('无效停止等待时间或安全余量')
        if self.speed_raw != 1 or not 1 <= self.step_ticks <= 4 or not 8 <= self.lead_ticks <= 16 or self.min_motion_ticks < 8:
            raise ValueError('此初版只允许最低非零速度、小步进和小目标超前量')


def check_feedback(f, c):
    if f.status:
        raise CalibrationError(f'故障状态0x{f.status:02x}；电压={f.voltage_v:.1f}V，不把故障位置当作限位')
    if not 4.0 <= f.voltage_v <= 8.4:
        raise CalibrationError(f'电压{f.voltage_v:.1f}V超出HD1910M允许范围4–8.4V')
    if f.temperature_c >= c.max_temperature_c:
        raise CalibrationError('温度达到停止阈值')
    if abs(f.current_ma) >= c.max_current_ma:
        raise CalibrationError(f'电流{f.current_ma:g}mA达到紧急停止阈值{c.max_current_ma:g}mA，位置{f.position}；不是有效挡块标定')
    if not -32767 <= f.position <= 32767:
        raise CalibrationError('反馈超出带符号位置协议范围')


class ContactDetector:
    def __init__(self, settings, start, direction, now):
        self.c, self.start, self.direction = settings, start, direction
        self.window = deque()
        self.last_position, self.progress_at = start, now
        self.previous_time, self.previous_position = now, start
        self.contact_at = None

    def update(self, f, target, now):
        c = self.c
        check_feedback(f,c)
        dt = now-self.previous_time
        if dt < 0 or dt > .1:
            raise CalibrationError('主机采样间隔异常，停止标定')
        if abs(f.position-self.previous_position) > 64:
            raise CalibrationError('反馈跳变/跨圈，停止标定')
        self.previous_time, self.previous_position = now, f.position
        travel = self.direction*(f.position-self.start)
        if travel < -8:
            raise CalibrationError('实际运动方向与搜索方向相反')
        if travel > c.max_travel_deg*TICKS_PER_DEG:
            raise CalibrationError('实际行程超过预算')
        if abs(f.position-self.last_position) >= 2:
            self.last_position, self.progress_at = f.position, now
        pushing = self.direction*(target-f.position) >= 8
        if abs(f.current_ma) >= c.contact_ma and pushing:
            self.window.append((now,f.position))
            # Keep the sample just before the window edge as duration evidence.
            while len(self.window)>1 and self.window[1][0] <= now-c.contact_s:
                self.window.popleft()
            if self.window and now-self.window[0][0] >= c.contact_s:
                stable = max(p for _,p in self.window)-min(p for _,p in self.window) <= 2
                if stable:
                    if travel < c.min_motion_ticks:
                        raise CalibrationError('起点就受阻/未确认有效运动；不记录为物理限位')
                    return True
        else:
            self.window.clear()
        if now-self.progress_at >= c.no_progress_s:
            raise CalibrationError('长时间无有效位移，且不满足触边条件；可能限流过低或卡滞')
        return False


def inspect(bus):
    identity = list(bus.read(0,5,allow_fault=True))
    config = {str(a):bus.integer(a,n,allow_fault=True) for a,n in
              ((8,1),(9,2),(11,2),(13,1),(14,1),(15,1),(18,1),(19,1),(28,2),(30,1),(31,2),(33,1),(38,1),(40,1),(44,2),(48,2),(55,1))}
    return dict(identity=identity,config=config,feedback=asdict(bus.feedback()))


def preflight(bus, c):
    c.validate()
    identity = bus.read(0,5,allow_fault=True)
    f = bus.feedback()
    check_feedback(f,c)
    if identity != EXPECTED_IDENTITY:
        raise CalibrationError(f'当前版本只适配已核对的固件3.46/型号10.31，实测{list(identity)}；未发送写指令')
    if bus.integer(55,1) != 1 or bus.integer(8,1) != 1:
        raise CalibrationError('需要EEPROM锁=1、写指令应答=1；不自动解锁或改应答策略')
    if bus.integer(30,1) != 1 or bus.integer(33,1) not in (0,4):
        raise CalibrationError('需要分辨率=1，当前为位置模式0或4')
    if not bus.integer(18,1) & 16:
        raise CalibrationError('需要全角度反馈；不自动修改相位配置')
    lo,hi = bus.integer(9,2),bus.integer(11,2)
    if (lo,hi) not in ((0,4095),(0,0)):
        raise CalibrationError('检测到自定义舵机角度限制；拒绝自动覆盖')
    original = {a:bus.integer(a,WIDTHS[a]) for a in RESTORE}
    return f,-32767,32767,original


def prepare(bus,c,original):
    # Always disable before changing limits or mode. Never unlock EEPROM.
    bus.disable()
    settings = {13:min(original[13],int(c.max_temperature_c)),14:min(original[14],84),
                15:max(original[15],40),19:original[19]|15,
                28:min(original[28],math.floor(c.current_limit_ma/CURRENT_UNIT_MA)),
                38:min(original[38],10),33:0,41:1,
                44:math.floor(c.current_limit_ma/CURRENT_UNIT_MA),46:c.speed_raw,
                48:min(original[48],int(c.torque_percent*10))}
    if settings[28]*CURRENT_UNIT_MA <= c.contact_ma or settings[38] <= 0:
        raise CalibrationError('原电流保护低于触边阈值或保护时间为零，需核对配置，未使能')
    for address,value in settings.items():bus.write(address,value)
    # HLS firmware 3.40–59: both limits zero selects multi-turn absolute mode.
    # Output remains disabled; lock=1 ensures changes are not persisted.
    bus.write(9,0)
    bus.write(11,0)
    if bus.integer(9,2) != 0 or bus.integer(11,2) != 0 or bus.integer(33,1) != 0:
        raise CalibrationError('多圈位置模式回读不符，拒绝使能')


def establish_reference(bus, c, report, sleep=time.sleep):
    # Mode changes may reset the full-angle counter. Establish a NEW frame while
    # torque is off; do not infer a modulo correction or command the old position.
    samples = report.setdefault('reference_samples', [])
    for index in range(5):
        if bus.integer(40,1) != 0:
            raise CalibrationError('建立坐标时扭矩未关闭，拒绝搜索')
        f = bus.feedback()
        samples.append(asdict(f))
        check_feedback(f,c)
        if abs(f.speed_raw) > 1:
            raise CalibrationError('建立坐标时机构仍在运动，拒绝使能')
        if index < 4:
            sleep(.02)
    if max(s['position'] for s in samples)-min(s['position'] for s in samples) > 2:
        raise CalibrationError('模式切换后位置未稳定，拒绝使能；保持卸力')
    return f


def seek(bus,c,direction,lo,hi,emit,clock=time.monotonic,sleep=time.sleep,expected_start=None):
    f = bus.feedback();check_feedback(f,c)
    if expected_start is not None and abs(f.position-expected_start) > 2:
        raise CalibrationError('使能前位置已变化，拒绝搜索')
    started = clock();start_position = f.position;target = start_position
    detector = ContactDetector(c,start_position,direction,started)
    bus.write_motion(target,c);bus.write(40,1)
    last_command_at = clock()
    while True:
        now = clock()
        if now-started >= c.timeout_s:
            raise CalibrationError('搜索超时；未检测到可信挡块')
        f = bus.feedback();now = clock()
        row=dict(t=now-started,direction=direction,target=target,**asdict(f))
        try:
            hit = detector.update(f,target,now)
        except CalibrationError:
            bus.disable()  # Capture the failing sample only AFTER confirmed stop.
            emit(dict(row,aborted=True))
            raise
        if hit:
            # Stop BEFORE logging, returning, or any file I/O.
            bus.disable()
            emit(row)
            return dict(raw_ticks=f.position,raw_deg=f.position/TICKS_PER_DEG,
                        detection='current_and_stationary_with_position_error',
                        physical_stop_confirmed=False,torque_disabled_confirmed=True,
                        feedback=asdict(f))
        emit(row)
        # Freeze the target during high-current contact detection.
        if abs(f.current_ma) < c.contact_ma and (direction*(target-f.position) <= c.step_ticks or now-last_command_at >= .4):
            # Repeated position writes restart this firmware's acceleration ramp.
            # Hold each bounded target until reached; never resend a stalled target.
            proposed = f.position+direction*c.lead_ticks
            proposed = min(proposed,f.position+c.lead_ticks) if direction>0 else max(proposed,f.position-c.lead_ticks)
            if not lo < proposed < hi:
                raise CalibrationError('到达已有软件角度边界；不把软件限位当成物理挡块')
            if direction*(proposed-start_position) > c.max_travel_deg*TICKS_PER_DEG:
                raise CalibrationError('到达最大搜索角度；没有把此位置记录为物理限位')
            if proposed != target:
                target=proposed;bus.write_motion(target,c)
                last_command_at=clock()
        sleep(c.period_s)


def calibrate(bus,c,zero_mode='lower',emit=lambda row:None,report=None):
    report = {} if report is None else report
    f,lo,hi,original = preflight(bus,c)  # No control writes before this succeeds.
    first_direction = -c.direction if zero_mode == 'lower' else c.direction
    if not lo < f.position + first_direction*c.lead_ticks < hi:
        raise CalibrationError('起点沿搜索方向太靠近带符号位置协议边界；未发送运动指令')
    report.update(status='running',settings=asdict(c),original_registers=original,
                  initial=asdict(f),endpoints={})
    bus.writes_allowed=True
    primary_error=None
    try:
        prepare(bus,c,original)
        after = establish_reference(bus,c,report)
        report['position_mode'] = 'temporary_signed_multiturn'
        report['after_prepare'] = asdict(after)
        report['coordinate_change_ticks'] = after.position-f.position
        if not lo < after.position + first_direction*c.lead_ticks < hi:
            raise CalibrationError('切换后位置太靠近协议边界，拒绝使能')
        print('开始搜索挡块，Ctrl+C停止。',flush=True)
        if zero_mode=='lower':
            lower=seek(bus,c,-c.direction,lo,hi,emit,expected_start=after.position)
            report['endpoints']['lower']=lower
            zero=lower['raw_ticks']
        else:
            zero=after.position
            report['endpoints']['zero']=dict(raw_ticks=zero,source='stationary_position_after_mode_setup')
        upper=seek(bus,c,c.direction,lo,hi,emit,expected_start=zero)
        report['endpoints']['upper']=upper
        span=c.direction*(upper['raw_ticks']-zero)
        margin=math.ceil(c.margin_deg*TICKS_PER_DEG)
        lower_margin=margin if zero_mode=='lower' else 0
        if span <= lower_margin+margin:
            raise CalibrationError('行程小于预留余量，未生成有效运行范围')
        report['calibration']=dict(zero_raw_ticks=zero,upper_raw_ticks=upper['raw_ticks'],
            direction=c.direction,travel_ticks=span,travel_deg=span/TICKS_PER_DEG,
            recommended_logical_min_ticks=lower_margin,recommended_logical_max_ticks=span-margin,
            conversion='logical_ticks = direction * (raw_ticks - zero_raw_ticks)',
            physical_stop_confirmed=False,encoder_offset_written=False,
            coordinate_system='signed_full_angle_in_calibration_mode',
            requires_revalidation_after_mode_restore=True,
            requires_revalidation_after_power_cycle=True)
        report['status']='completed_candidate'
    except BaseException as exc:
        primary_error=exc;report.update(status='failed',error=str(exc))
        raise
    finally:
        errors=[];disabled=False
        try:bus.disable();disabled=True
        except Exception as exc:errors.append(str(exc))
        # Never restore stronger limits unless output disable was read back.
        if disabled:
            for address in RESTORE:
                try:bus.write(address,original[address])
                except Exception as exc:errors.append(f'恢复寄存器{address}失败：{exc}')
        report['cleanup']=dict(torque_disabled_confirmed=disabled,errors=errors,
                               torque_reenabled=False)
        if errors:
            report['status']='failed';report.pop('calibration',None)
            print('清理未完全确认：'+'; '.join(errors),file=sys.stderr,flush=True)
            if primary_error is None:raise CalibrationError('卸力/临时参数恢复未完全确认，不能把本次标定视为成功')
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default=DEFAULT_PORT,help='省略时自动选择唯一的1a86 USB Single Serial适配器');p.add_argument('--id',type=int,default=1)
    p.add_argument('--baud',type=int,default=1000000)
    p.add_argument('--execute',action='store_true',help='低电流驱动外部挡块搜索；省略则仅只读诊断')
    p.add_argument('--direction',type=int,choices=[-1,1],help='逻辑上限方向：+1编码器增大，-1减小')
    p.add_argument('--zero',choices=['lower','current'],default='lower',help='lower先找下挡块作为零点；current当前位置作为零点只找上挡块')
    p.add_argument('--max-travel-deg',type=float,default=90)
    p.add_argument('--timeout',type=float,default=45)
    p.add_argument('--current-limit-ma',type=float,default=150)
    p.add_argument('--contact-ma',type=float,default=100)
    p.add_argument('--max-current-ma',type=float,default=200)
    p.add_argument('--margin-deg',type=float,default=2)
    p.add_argument('--output',type=Path)
    a=p.parse_args(argv)
    if a.execute and a.direction is None:p.error('执行标定时必须指定 --direction 1 或 -1')
    c=Settings(direction=a.direction or 1,max_travel_deg=a.max_travel_deg,timeout_s=a.timeout,
               current_limit_ma=a.current_limit_ma,contact_ma=a.contact_ma,
               max_current_ma=a.max_current_ma,margin_deg=a.margin_deg)
    try:c.validate()
    except ValueError as exc:p.error(str(exc))
    output=a.output or ROOT/'logs'/('servo_calibration_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    if output.exists():p.error(f'输出目录已存在，避免覆盖旧标定：{output}')
    output.mkdir(parents=True)
    report=dict(at=datetime.now().isoformat(),port=a.port,id=a.id,baud=a.baud,execute=a.execute)
    bus=None;code=0
    def interrupted(signum,frame):raise KeyboardInterrupt(f'signal {signum}')
    old=signal.signal(signal.SIGTERM,interrupted)
    try:
        port=resolve_port(a.port)
        report['port']=port
        bus=Bus(port,a.id,a.baud)
        report['before']=inspect(bus)
        f = report['before']['feedback']
        print(f"ID{a.id}：位置 {f['position']}，电压 {f['voltage_v']:.1f}V，"
              f"温度 {f['temperature_c']}℃，电流 {f['current_ma']:g}mA，"
              f"故障 0x{f['status']:02x}，扭矩 {'开启' if report['before']['config']['40'] else '关闭'}",flush=True)
        if a.execute:
            with (output/'samples.jsonl').open('w',buffering=1) as log:
                calibrate(bus,c,a.zero,lambda row:log.write(json.dumps(row)+'\n'),report)
            result=report['calibration']
            print(f"已卸力：零点 {result['zero_raw_ticks']}，候选上限 {result['upper_raw_ticks']}，"
                  f"行程 {result['travel_deg']:.2f}°（需现场核实挡块）。",flush=True)
        else:report['status']='inspection_only'
    except BaseException as exc:
        report.update(status='failed',error=str(exc));code=130 if isinstance(exc,KeyboardInterrupt) else 1
        print(str(exc),file=sys.stderr,flush=True)
    finally:
        if bus:bus.close()
        signal.signal(signal.SIGTERM,old)
        (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(f'记录：{output / "result.json"}',flush=True)
    return code


if __name__=='__main__':raise SystemExit(main())
