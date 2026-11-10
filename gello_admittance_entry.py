"""HD1910M position-mode admittance and explicit calibration entry points."""
from pathlib import Path
import sys,json,time,signal,math
from datetime import datetime
ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'source/host')]
import numpy as np
import GELLO导纳参数 as cfg
from gello_admittance import Model,Mapping,TorqueCalibration,Admittance,fit_mapping,fit_current
from tools.servo_travel_calibration import Bus,resolve_port,EXPECTED_IDENTITY,CalibrationError,check_feedback,encode_position,CURRENT_UNIT_MA
from types import SimpleNamespace
from dataclasses import asdict

def load(path):return json.loads(Path(path).read_text())
def save(path,doc):Path(path).write_text(json.dumps(doc,ensure_ascii=False,indent=2,allow_nan=False))
def append(path,doc):
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    with Path(path).open('a') as f:f.write(json.dumps(doc,ensure_ascii=False,allow_nan=False)+'\n')
def rows(path):return [json.loads(s) for s in Path(path).read_text().splitlines() if s.strip()]
def model():return Model(cfg.URDF,cfg.GRAVITY_BASE)

def record_mapping():
    """No writes: capture one known URDF pose. At least two poses/axis needed."""
    m=model();q=np.deg2rad(np.asarray(cfg.KNOWN_URDF_Q_DEG,float))
    if q.shape!=(6,) or not np.all(np.isfinite(q)) or np.any(q<m.model.lowerPositionLimit) or np.any(q>m.model.upperPositionLimit):raise ValueError('已知URDF角度无效')
    b=Bus(resolve_port(cfg.PORT));samples=[]
    try:
        for n in range(1,7):
            b.node=n
            if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(40,1)!=0:raise ValueError(f'ID{n}型号或卸力状态不符')
            if b.integer(33,1)!=0:raise ValueError('记录前必须是已核对的模式0坐标，不能跨模式套用零位')
        for _ in range(15):
            row=[]
            for n in range(1,7):
                b.node=n;f=b.feedback()
                if f.status:raise ValueError(f'ID{n}故障{f.status}')
                row.append(f.position)
            samples.append(row);time.sleep(.02)
    finally:b.close()
    a=np.asarray(samples)
    if np.any(np.ptp(a,axis=0)>2):raise ValueError('姿态未稳定，未保存')
    item={'urdf_sha256':m.sha,'q_rad':q.tolist(),'ticks':np.median(a,axis=0).tolist(),'modes':[0]*6}
    path=ROOT/'logs/gello_urdf_pose_samples.jsonl';append(path,item)
    all_rows=rows(path)
    if len(all_rows)>=2:
        doc=fit_mapping(all_rows,m,cfg.GRAVITY_DIRECTION_CONFIRMED);save(cfg.MAPPING,doc)
        print('映射各轴验证状态：',doc['verified'],'底座方向确认：',doc['gravity_verified'])
    print('已保存已知姿态对应点：',path)

class PositionBus:
    def __init__(self,folder):
        self.b=None;self.changed=False;self.last={};self.final={};self.folder=folder
        self.passive=set(cfg.PASSIVE_IDS)
        if not self.passive.issubset(set(range(1,8))):raise ValueError("无效被动轴")
        self.limits=SimpleNamespace(max_current_ma=cfg.CURRENT_STOP_MA,max_temperature_c=cfg.MAX_TEMPERATURE_C)
        self.limit=math.floor(cfg.CURRENT_LIMIT_MA/CURRENT_UNIT_MA)
        if not 0<cfg.CURRENT_LIMIT_MA<cfg.CURRENT_STOP_MA<=2000:raise ValueError('电流设置无效')
    def open(self):
        self.b=Bus(resolve_port(cfg.PORT))
        for n in range(1,8):
            self.b.node=n
            if self.b.read(0,5)!=EXPECTED_IDENTITY or self.b.integer(40,1)!=0 or self.b.integer(33,1)!=0:
                raise ValueError(f'ID{n}须为已卸力的模式0，不自动切换坐标')
            if self.b.integer(9,2)!=0 or self.b.integer(11,2)!=0:raise ValueError(f'ID{n}坐标范围配置不符')
        return self.sample()
    def sample(self):
        started=time.monotonic();f=[]
        for n in range(1,8):
            self.b.node=n;v=self.b.feedback();f.append(v)
            try:check_feedback(v,self.limits)
            except CalibrationError as exc:raise CalibrationError(f'ID{n}: {exc}') from exc
        if time.monotonic()-started>cfg.MAX_FEEDBACK_AGE_S:raise ValueError('反馈采集过期')
        return np.array([x.position for x in f]),np.array([x.current_ma for x in f]),[asdict(x) for x in f]
    def send(self,ticks):
        for n,tick in enumerate(ticks,1):
            if n in self.passive:continue
            tick=int(tick)
            if self.last.get(n)==tick:continue
            self.b.node=n
            payload=bytes([0])+encode_position(tick).to_bytes(2,'little')+self.limit.to_bytes(2,'little')+(3).to_bytes(2,'little')
            _,status=self.b.exchange(3,bytes([41])+payload,0)
            if status:raise CalibrationError(f'ID{n}位置指令故障0x{status:02x}')
            self.last[n]=tick
    def enable(self,ticks):
        self.b.writes_allowed=True;self.changed=True;self.send(ticks)
        for n in range(1,8):
            if n in self.passive:continue
            self.b.node=n;self.b.write(40,1)
    def verify(self):
        for n in range(1,8):
            self.b.node=n
            if n in self.passive:
                if self.b.integer(40,1)!=0:raise ValueError(f'ID{n}应保持卸力')
                continue
            if self.b.integer(33,1)!=0 or self.b.integer(44,2)!=self.limit or self.b.integer(40,1)!=1:
                raise ValueError(f'ID{n}模式/电流指令/使能发生改变')
    def close(self):
        if not self.b:return
        errors={}
        if self.changed:
            for n in range(1,8):
                self.b.node=n
                try:self.b.disable()
                except Exception as exc:errors[n]=str(exc)
        for n in range(1,8):
            self.b.node=n
            try:self.final[n]=self.b.integer(40,1,allow_fault=True)
            except Exception as exc:errors[n]=str(exc)
        self.b.close();self.b=None
        if errors or any(v!=0 for v in self.final.values()):raise RuntimeError('卸力未确认：'+str(errors or self.final))

def run(kind='readonly'):
    m=model()
    if not cfg.MAPPING.exists():raise ValueError('尚未完成URDF姿态映射，请先记录已知URDF姿态对应点')
    mapping=Mapping(load(cfg.MAPPING),m)
    cal=None
    if kind=='admittance':cal=TorqueCalibration(load(cfg.TORQUE_CALIBRATION),m,mapping)
    if kind=='static' and not cfg.STATIC_SAMPLE_NO_EXTERNAL_FORCE:raise ValueError('请先确认静态采样无外力并设置源码中的STATIC_SAMPLE_NO_EXTERNAL_FORCE')
    if kind not in ('readonly','static','admittance'):raise ValueError('Unknown mode')
    folder=ROOT/'logs'/('gello_admittance_'+kind+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report={'status':'starting','mode':kind,'urdf_sha256':m.sha,'mapping_identity':mapping.identity};bus=PositionBus(folder)
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    previous=signal.signal(signal.SIGTERM,stop);sample_rows=[]
    try:
        ticks,current,_=bus.open();q=mapping.q(ticks[:6]);fixed7=int(ticks[6])
        if kind!='readonly':q=mapping.bounds(q)
        if kind!='readonly':
            bus.enable(ticks)
            ticks,current,_=bus.sample();q=mapping.bounds(mapping.q(ticks[:6]))
        control=Admittance(m,mapping,cal,cfg,q,current[:6]) if cal else None
        print({'readonly':'只读重力模型，不使能','static':'七轴当前位置静态采样5秒后卸力，请勿施加外力','admittance':'位置模式导纳已启动，ID7固定保持，Ctrl+C停止'}[kind],flush=True)
        began=last=time.monotonic();next_verify=began;next_print=began
        with (folder/'samples.jsonl').open('w',buffering=1) as log:
            while kind!='static' or time.monotonic()-began<5:
                tick=time.monotonic();dt=tick-last;last=tick
                if dt>cfg.MAX_FEEDBACK_AGE_S:raise ValueError('控制循环过期')
                ticks,current,feedback=bus.sample();q=mapping.q(ticks[:6])
                if kind!='readonly':q=mapping.bounds(q)
                record={'t':tick-began,'q_rad':q.tolist(),'current_ma':current.tolist(),'feedback':feedback,'g_nm':m.gravity(q).tolist()}
                if control:
                    target,detail=control.update(q,current[:6],max(dt,1e-6));record.update(detail)
                    bus.send(np.r_[mapping.ticks(target),fixed7])
                if kind!='readonly' and tick>=next_verify:bus.verify();next_verify=tick+.1
                log.write(json.dumps(record,ensure_ascii=False)+'\n');sample_rows.append(record) if kind=='static' else None
                if tick>=next_print:
                    print('q_deg='+str(np.round(np.rad2deg(q),1).tolist())+' I_mA='+str(current[:6].tolist())+' g_Nm='+str(np.round(m.gravity(q),3).tolist()),flush=True);next_print=tick+1
                time.sleep(max(0,1/cfg.HZ-(time.monotonic()-tick)))
        if kind=='static':
            tail=[r for r in sample_rows if r['t']>=3]
            qs=np.array([r['q_rad'] for r in tail]);curr=np.array([r['current_ma'][:6] for r in tail])
            if len(tail)<20 or np.any(np.ptp(qs,axis=0)>np.deg2rad(.5)):raise ValueError('采样姿态未稳定，不用于力矩辨识')
            append(cfg.STATIC_SAMPLES,{'urdf_sha256':m.sha,'mapping_identity':mapping.identity,'q_rad':np.median(qs,axis=0).tolist(),'current_ma':np.median(curr,axis=0).tolist(),'no_external_force':True})
            print('静态样本已保存。',flush=True)
        report['status']='completed'
    except BaseException as exc:
        report.update(status='stopped',error=str(exc));print('停止：'+str(exc),flush=True)
        if bus.b:report['fault_context']={'id':bus.b.node,'recent_packets':list(bus.b.recent_packets)}
    finally:
        try:bus.close()
        except Exception as exc:report['cleanup_error']=str(exc);print(str(exc),flush=True)
        report['final_torque']=bus.final;save(folder/'result.json',report);signal.signal(signal.SIGTERM,previous)
        print('记录：'+str(folder/'result.json'),flush=True)
    return report

def identify():
    m=model();mapping=Mapping(load(cfg.MAPPING),m)
    doc=fit_current(rows(cfg.STATIC_SAMPLES),m,mapping);save(cfg.TORQUE_CALIBRATION,doc);print(json.dumps(doc,ensure_ascii=False,indent=2))

def stable_ticks():
    b=Bus(resolve_port(cfg.PORT));values=[]
    try:
        for n in range(1,7):
            b.node=n
            if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(33,1)!=0 or b.integer(40,1)!=0:
                raise ValueError(f'ID{n}必须为卸力的模式0，先核对坐标')
        for _ in range(15):
            row=[]
            for n in range(1,7):
                b.node=n;f=b.feedback()
                if f.status:raise ValueError(f'ID{n}故障状态{f.status}')
                row.append(f.position)
            values.append(row);time.sleep(.02)
    finally:b.close()
    values=np.array(values)
    if np.any(np.ptp(values,axis=0)>2):raise ValueError('姿态未稳定')
    return np.median(values,axis=0)

def record_zero():
    m=model();ticks=stable_ticks()
    doc={'urdf_sha256':m.sha,'ids':list(range(1,7)),'zero_ticks':ticks.tolist(),'sign':[0]*6,'verified':[False]*6,
         'gravity_base':m.gravity_vector.tolist(),'gravity_verified':cfg.GRAVITY_DIRECTION_CONFIRMED,
         'source':'用户按URDF Pose A摆放，软件记录；不写硬件零位'}
    save(cfg.MAPPING,doc);print('URDF零姿态已记录；下一步按参考图B的方向小幅改变六轴，再运行GELLO记录URDF方向.py。')

def record_directions():
    """Reference B establishes only direction, NOT an assumed exact 10° angle."""
    m=model();doc=load(cfg.MAPPING)
    if doc['urdf_sha256']!=m.sha:raise ValueError('URDF变化，重新记录零姿态')
    ticks=stable_ticks();delta=ticks-np.asarray(doc['zero_ticks'])
    expected=np.array([1,-1,1,1,1,1])
    valid=(abs(delta)>=4*4096/360)&(abs(delta)<=30*4096/360)
    evidence=doc.setdefault('direction_evidence_by_axis',{})
    for i in range(6):
        if not valid[i]:continue
        doc['sign'][i]=int(np.sign(delta[i])*expected[i]);doc['verified'][i]=True
        evidence[str(i+1)]={'ticks':float(ticks[i]),'delta_ticks':float(delta[i]),'requested_urdf_direction':int(expected[i])}
    doc['gravity_verified']=cfg.GRAVITY_DIRECTION_CONFIRMED
    save(cfg.MAPPING,doc)
    print('相对零姿态变化度：',np.round(delta*360/4096,2).tolist())
    print('已确认轴：',[i+1 for i,v in enumerate(doc['verified']) if v])
    print('待确认轴：',[i+1 for i,v in enumerate(doc['verified']) if not v])
    print('底座重力方向确认：',doc['gravity_verified'])
