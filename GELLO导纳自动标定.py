"""Hold, wait for hands-off acknowledgement, then collect nearby static poses.
No admittance enabled until independently checked current calibration is usable.
"""
from gello_admittance_entry import *
from gello_admittance import TICK_RAD
from tools.servo_travel_calibration import CURRENT_UNIT_MA
SCAN_SPEED_DEG_S=3.
SCAN_DELTA_DEG=12.

def run_scan():
    m=model();mapping=Mapping(load(cfg.MAPPING),m)
    folder=ROOT/'logs'/('gello_admittance_scan_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report={'status':'starting','mapping_identity':mapping.identity,'urdf_sha256':m.sha}
    bus=PositionBus(folder);stage='waiting_release';samples=[];last_verify=0
    previous=signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    log=(folder/'samples.jsonl').open('w',buffering=1)
    def read():
        nonlocal last_verify
        ticks,current,feedback=bus.sample();q=mapping.bounds(mapping.q(ticks[:6]));now=time.monotonic()
        row={'t':now,'stage':stage,'q_rad':q.tolist(),'current_ma':current.tolist(),'g_nm':m.gravity(q).tolist(),'feedback':feedback}
        log.write(json.dumps(row)+'\n')
        if now-last_verify>.1:bus.verify();last_verify=now
        return q,current,row
    def hold(seconds):
        deadline=time.monotonic()+seconds;out=[]
        while time.monotonic()<deadline:
            q,current,row=read();out.append(row);time.sleep(.02)
        return out
    def move(destination):
        origin,_,_=read();duration=max(.04,1.5*np.max(abs(destination-origin))/np.deg2rad(SCAN_SPEED_DEG_S));steps=math.ceil(duration/.04)
        for k in range(1,steps+1):
            began=time.monotonic();read();u=k/steps;target=origin+(destination-origin)*(u*u*(3-2*u))
            bus.send(np.r_[mapping.ticks(target),fixed7]);time.sleep(max(0,.04-(time.monotonic()-began)))
    try:
        ticks,current,_=bus.open();mapping.bounds(mapping.q(ticks[:6]));fixed7=int(ticks[6]);bus.enable(ticks)
        save(ROOT/'logs/gello_admittance_scan_active.json',{'folder':str(folder),'release_file':str(folder/'hands_off.json')})
        print('ID2–7已保持当前位置，ID1保持卸力。等待撤去托举力后开始采集。',flush=True)
        print('release_file='+str(folder/'hands_off.json'),flush=True)
        while not (folder/'hands_off.json').exists():read();time.sleep(.02)
        ack=load(folder/'hands_off.json')
        if ack.get('no_external_force') is not True:raise ValueError('需要无外力确认')
        stage='settling';hold(2);center,_,_=read();report['center']=center.tolist()
        # Includes repeated center and diagonal changes; last three held out from fit.
        offsets=[(0,0),(12,0),(-12,0),(0,12),(0,-12),(12,12),(-12,-12),(12,-12),(-12,12),(0,0),(6,6),(-6,-6),(6,-6)]
        plan=[]
        for a,b in offsets:
            q=center.copy();q[1]+=np.deg2rad(a);q[2]+=np.deg2rad(b)
            if np.any(q<mapping.lower+np.deg2rad(2)) or np.any(q>mapping.upper-np.deg2rad(2)):
                raise ValueError('当前姿态太靠近URDF边界，无法进行±12度标定')
            plan.append(q)
        save(folder/'plan.json',{'q_rad':[q.tolist() for q in plan]})
        print('开始13个附近姿态采样；第2、3轴各最多变化12度，最大速度3度/秒。请勿托举或推拉。',flush=True)
        for index,destination in enumerate(plan):
            stage='move_'+str(index+1);move(destination);stage='settle_'+str(index+1);hold(2)
            stage='sample_'+str(index+1);tail=hold(2)
            qs=np.array([r['q_rad'] for r in tail]);cs=np.array([r['current_ma'][:6] for r in tail])
            if np.any(np.ptp(qs,axis=0)>np.deg2rad(.5)):raise ValueError('姿态仍在移动，不能当作静态力矩样本')
            sample={'mapping_identity':mapping.identity,'urdf_sha256':m.sha,'q_rad':np.median(qs,axis=0).tolist(),'current_ma':np.median(cs,axis=0).tolist(),'no_external_force':True,'source':str(folder),'index':index+1}
            sample['saturated_axes']=[i+1 for i,v in enumerate(sample['current_ma']) if abs(v)>=.95*bus.limit*CURRENT_UNIT_MA]
            append(folder/'static_samples.jsonl',sample)
            if sample['saturated_axes']:raise ValueError('当前姿态输出饱和，无法可靠标定，轴：'+str(sample['saturated_axes']))
            samples.append(sample)
            print(f'静态采样{index+1}/13：电流mA='+str(sample['current_ma']),flush=True)
        stage='return_center';move(center);hold(2)
        try:
            fitted=fit_current(samples[:10],m,mapping);cal=TorqueCalibration(fitted,m,mapping)
            errors=np.array([m.gravity(r['q_rad'])-cal.torque(r['current_ma']) for r in samples[10:]])
            if np.any(np.max(abs(errors),axis=0)[cal.active]>.06):raise ValueError('独立验证姿态力矩误差超过0.06Nm')
            fitted['validation_max_error_nm']=np.max(abs(errors),axis=0).tolist();fitted['source']=str(folder)
            save(cfg.TORQUE_CALIBRATION,fitted);report['calibration']=fitted;report['status']='calibrated_holding'
            print('电流力矩标定通过，保持中心姿态；尚未切入导纳。',flush=True)
        except ValueError as exc:
            report.update(status='identification_failed_holding',identification_error=str(exc));print('标定尚不可信：'+str(exc)+'；继续保持当前姿态。',flush=True)
        save(folder/'identification.json',report);stage='holding_after_scan'
        while True:read();time.sleep(.02)
    except BaseException as exc:
        report.update(status='stopped',error=str(exc));print('停止：'+str(exc),flush=True)
        if bus.b:report['fault_context']={'id':bus.b.node,'recent_packets':list(bus.b.recent_packets)}
    finally:
        try:bus.close()
        except Exception as exc:report['cleanup_error']=str(exc)
        report['final_torque']=bus.final;save(folder/'result.json',report);log.close();signal.signal(signal.SIGTERM,previous)
        print('七轴最终使能状态：'+str(bus.final),flush=True);print('记录：'+str(folder/'result.json'),flush=True)

if __name__=='__main__':run_scan()
