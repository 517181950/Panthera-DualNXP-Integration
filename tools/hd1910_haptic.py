"""Single serial owner for six low-current position servos; no constant-current mode."""
import threading
from types import SimpleNamespace
from tools.servo_travel_calibration import *


class HapticBus:
    def __init__(self, config, joints):
        self.c=config;self.joints=joints;self.b=None;self.original={}
        offsets=getattr(config,'RAW_FRAME_OFFSETS',[0]*7)
        if len(offsets)!=7 or any(not isinstance(v,int) or v%4096 for v in offsets):
            raise ValueError('坐标偏移须为7个整圈计数')
        self.shift={n:offsets[n-1] for n in range(1,8)}
        self.lock=threading.Lock();self.stop_event=threading.Event();self.thread=None
        self.error=None;self.latest=None;self.command=None;self.cleanup_errors={}
        self.changed=False;self.enabled=set();self.last_goals={};self.final_torque={};self.last_fault_feedback=None
        self.ids=tuple(getattr(config,'FEEDBACK_IDS',range(1,7)))
        if self.ids not in (tuple(range(1,7)),tuple(range(1,8))):raise ValueError('反馈轴必须为1..6或1..7')
        self.rms2={};self.rms_time={}
        if not 0<config.CURRENT_LIMIT_MA<=300 or not config.CURRENT_LIMIT_MA<config.CURRENT_STOP_MA<=1800:
            raise ValueError('主手限流须<=300mA，峰值停止阈值须大于限流且<=1800mA')
        if not 0<getattr(config,'CURRENT_RMS_MA',900)<=900 or not 0<getattr(config,'CURRENT_RMS_SECONDS',.5)<=.5:
            raise ValueError('持续电流试验阈值/时间常数不适用')
        self.limits=SimpleNamespace(max_current_ma=config.CURRENT_STOP_MA,max_temperature_c=50.)

    def open(self):
        self.b=Bus(resolve_port(self.c.PORT))
        for n in range(1,8):
            self.b.node=n
            if self.b.read(0,5)!=EXPECTED_IDENTITY or self.b.integer(40,1)!=0:
                raise CalibrationError(f'ID{n}型号不符或已使能')
        self.read_all()
        return self

    def read_all(self):
        positions=[];currents=[];stamps=[]
        for n in range(1,8):
            self.b.node=n;f=self.b.feedback()
            try:check_feedback(f,self.limits)
            except CalibrationError as exc:
                self.last_fault_feedback={'id':n,**asdict(f)}
                raise CalibrationError(f'ID{n}: {exc}') from exc
            if n in self.enabled:
                self.check_rms(n,f.current_ma,time.perf_counter())
                expected=math.floor(self.c.CURRENT_LIMIT_MA/CURRENT_UNIT_MA)
                if self.b.integer(33,1)!=0 or self.b.integer(44,2)!=expected or self.b.integer(40,1)!=1:
                    raise CalibrationError(f'ID{n}运行模式/限流/使能被改变')
            pos=f.position-self.shift.get(n,0);j=self.joints[n]
            if not j['min_ticks']<=pos<=j['max_ticks']:raise CalibrationError(f'ID{n}坐标超出人工行程，需核对多圈零位')
            positions.append(pos);currents.append(f.current_ma);stamps.append(time.perf_counter())
        sample={'ticks':positions,'current_ma':currents,'oldest':min(stamps),'received':max(stamps)}
        with self.lock:self.latest=sample
        return sample

    def check_rms(self,n,current_ma,now):
        previous=self.rms_time.get(n,now)
        dt=max(0,now-previous);self.rms_time[n]=now
        alpha=1-math.exp(-dt/getattr(self.c,'CURRENT_RMS_SECONDS',.5))
        self.rms2[n]=(1-alpha)*self.rms2.get(n,0.)+alpha*current_ma**2
        if self.rms2[n]>=getattr(self.c,'CURRENT_RMS_MA',900.)**2:
            raise CalibrationError(f'ID{n}持续电流RMS超过900mA，卸力')

    def _send(self,n,logical):
        if self.last_goals.get(n)==int(logical):return
        self.b.node=n
        pos=int(logical)+self.shift[n]
        limit=math.floor(self.c.CURRENT_LIMIT_MA/CURRENT_UNIT_MA)
        data=bytes([1])+encode_position(pos).to_bytes(2,'little')+limit.to_bytes(2,'little')+(3).to_bytes(2,'little')
        _,status=self.b.exchange(3,bytes([41])+data,0)
        if status:raise CalibrationError(f'ID{n}位置限流指令故障{status}')
        self.last_goals[n]=int(logical)

    def prepare(self):
        self.b.writes_allowed=True
        for n in self.ids:
            self.b.node=n
            if self.b.integer(55,1)!=1 or self.b.integer(33,1) not in (0,4) or not self.b.integer(18,1)&16:
                raise CalibrationError(f'ID{n}模式/锁/反馈不符')
            if (self.b.integer(9,2),self.b.integer(11,2)) not in ((0,0),(0,4095)):
                raise CalibrationError(f'ID{n}已有自定义限位')
            self.original[n]={a:self.b.integer(a,WIDTHS[a]) for a in (33,11,9,41,42,44,46,48)}
            self.changed=True
            if (self.original[n][33],self.original[n][9],self.original[n][11])==(0,0,0):
                # No coordinate transition: do not rewrite mode or rebase a moving encoder.
                after=self.b.feedback();check_feedback(after,self.limits)
            else:
                before=establish_reference(self.b,self.limits,{})
                self.b.write(33,0);self.b.write(9,0);self.b.write(11,0)
                after=establish_reference(self.b,self.limits,{})
                delta=after.position-before.position;shift=round(delta/4096)*4096
                if abs(delta-shift)>2:raise CalibrationError(f'ID{n}切换模式时发生移动/未知坐标变化')
                self.shift[n]+=shift
            self._send(n,after.position-self.shift[n])
        self.read_all()

    def enable(self):
        # Establish targets immediately before enabling. No saved zero trajectory.
        for n in self.ids:
            self.b.node=n;f=self.b.feedback()
            try:check_feedback(f,self.limits)
            except CalibrationError as exc:
                self.last_fault_feedback={'id':n,**asdict(f)}
                raise CalibrationError(f'ID{n}: {exc}') from exc
            self._send(n,f.position-self.shift[n]);self.b.write(40,1);self.enabled.add(n)
        self.read_all()

    def publish(self, ticks, source_ticks=None):
        if len(ticks)!=len(self.ids):raise ValueError('反馈目标数量不匹配')
        with self.lock:
            source=list(source_ticks) if source_ticks is not None else self.latest['ticks'][:len(self.ids)]
            if len(source)!=len(self.ids):raise ValueError('反馈源数量不匹配')
            self.command=(list(map(int,ticks)),time.perf_counter(),source)

    def rebase_goal(self, n, goal, source, actual):
        j=self.joints[n]
        if not j['min_ticks']<=goal<=j['max_ticks']:
            raise RuntimeError(f'ID{n}反馈目标超出人工行程')
        offset=goal-source
        cap=math.ceil(getattr(self.c,'FEEDBACK_OFFSET_DEG',3.)*TICKS_PER_DEG)+1
        if not math.isfinite(offset) or abs(offset)>cap:
            raise RuntimeError(f'ID{n}计算的反力偏移{offset}tick超出{cap}tick')
        # Retain only the bounded feedback displacement; user movement since
        # the controller sample must not become an unintended spring stretch.
        return int(max(j['min_ticks'],min(j['max_ticks'],actual+offset)))

    def snapshot(self):
        with self.lock:sample=self.latest;error=self.error
        if error is not None:raise RuntimeError('GELLO通信/保护停止：'+str(error))
        if sample is None or time.perf_counter()-sample['oldest']>self.c.SERVO_MAX_AGE_S:
            raise RuntimeError('GELLO反馈过期')
        return sample

    def start(self):
        if self.command is None:raise RuntimeError('No initial command')
        self.thread=threading.Thread(target=self._worker,name='haptic-usb',daemon=False);self.thread.start()

    def _worker(self):
        try:
            while not self.stop_event.is_set():
                started=time.perf_counter();sample=self.read_all()
                with self.lock:command=self.command
                if command is None or time.perf_counter()-command[1]>self.c.COMMAND_TIMEOUT_S:
                    raise RuntimeError('机械臂回传目标过期，主手卸力')
                for n,goal in enumerate(command[0],1):
                    if self.stop_event.is_set():break
                    if time.perf_counter()-command[1]>self.c.COMMAND_TIMEOUT_S:raise RuntimeError('回传目标超时')
                    goal=self.rebase_goal(n,goal,command[2][n-1],sample['ticks'][n-1])
                    self._send(n,goal)
                self.stop_event.wait(max(0,1/self.c.SERVO_HZ-(time.perf_counter()-started)))
        except BaseException as exc:
            self.last_fault_feedback={
                "id":getattr(self.b,"node",None), "error":str(exc),
                "feedback":self.last_fault_feedback,
                "recent_packets":list(getattr(self.b,"recent_packets",[]))}
            with self.lock:self.error=exc
        finally:self._disable_all()

    def _disable_all(self):
        for n in reversed(list(self.original)):
            try:self.b.node=n;self.b.disable();self.enabled.discard(n)
            except Exception as exc:self.cleanup_errors[n]=str(exc)

    def close(self):
        self.stop_event.set()
        if self.thread:self.thread.join()  # Never race serial cleanup against its owner.
        if self.b:
            if self.changed:
                self._disable_all()
                # Do not replay old mode/position/current registers after stop.
                # Live testing showed enables remaining after the old restore path.
                for n in range(1,8):
                    self.b.node=n
                    try:
                        if self.b.integer(40,1,allow_fault=True)!=0:self.b.disable()
                        self.final_torque[n]=self.b.integer(40,1,allow_fault=True)
                        if self.final_torque[n]!=0:raise RuntimeError('扭矩仍开启')
                    except Exception as exc:self.cleanup_errors[n]=str(exc)
            self.b.close();self.b=None
        if self.cleanup_errors:raise RuntimeError('主手最终卸力未确认：'+str(self.cleanup_errors))
