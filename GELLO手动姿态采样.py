"""用户手动摆姿态；程序只保持启动位置，确认无外力后记录一组静态样本。
采样完成后继续保持，等用户托住并停止后才卸力；不包含任何换姿态轨迹。
"""
from gello_admittance_entry import *
from tools.servo_travel_calibration import CURRENT_UNIT_MA

def run_manual():
    m=model();mapping=Mapping(load(cfg.MAPPING),m)
    folder=ROOT/'logs'/('gello_manual_static_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report={'status':'starting','mapping_identity':mapping.identity,'urdf_sha256':m.sha,'motion':'hold_start_position_only'}
    bus=PositionBus(folder);stage='waiting_release';last_verify=0.;release_time=None;window=[];finished=False
    previous=signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        ticks,current,_=bus.open();report['held_raw_ticks']=ticks.tolist()
        report['raw_model_q_rad']=mapping.q(ticks[:6]).tolist()
        report['coordinate_note']='Static hold sends the measured raw position unchanged. URDF joint limits are not used to reject this read-only gravity evaluation; full encoder counts are retained.'
        bus.enable(ticks)
        save(ROOT/'logs/gello_manual_static_active.json',{'folder':str(folder),'release_file':str(folder/'hands_off.json')})
        print('ID2–7已保持当前姿态，ID1卸力；不会自动换姿态。等待确认已松开。',flush=True)
        print('release_file='+str(folder/'hands_off.json'),flush=True)
        with (folder/'samples.jsonl').open('w',buffering=1) as log:
            while True:
                began=time.monotonic();ticks,current,feedback=bus.sample();q=mapping.q(ticks[:6])
                row={'t':began,'stage':stage,'q_rad':q.tolist(),'current_ma':current.tolist(),'raw_ticks':ticks.tolist(),'g_nm':m.gravity(q).tolist(),'feedback':feedback}
                log.write(json.dumps(row)+'\n')
                if began-last_verify>=.1:bus.verify();last_verify=began
                if release_time is None and (folder/'hands_off.json').exists():
                    ack=load(folder/'hands_off.json')
                    if ack.get('no_external_force') is not True:raise ValueError('无外力确认无效')
                    release_time=began;stage='settling';print('已确认松手，等待稳定并采样；目标位置保持不变。',flush=True)
                if release_time is not None and not finished and began-release_time>=2:
                    stage='sampling';window.append(row);window=[x for x in window if began-x['t']<=2]
                    if len(window)>=40 and window[-1]['t']-window[0]['t']>=1.9:
                        qs=np.array([x['q_rad'] for x in window]);cs=np.array([x['current_ma'][:6] for x in window]);span=np.rad2deg(np.ptp(qs,axis=0));median=np.median(cs,axis=0)
                        saturated=[i+1 for i,v in enumerate(median) if abs(v)>=.95*bus.limit*CURRENT_UNIT_MA]
                        if np.all(span<=.5) and not saturated:
                            sample={'mapping_identity':mapping.identity,'urdf_sha256':m.sha,'q_rad':np.median(qs,axis=0).tolist(),'current_ma':median.tolist(),
                                    'no_external_force':True,'source':str(folder),'saturated_axes':[],'usable':True,'position_span_deg':span.tolist(),'raw_ticks':np.median(np.array([x['raw_ticks'][:6] for x in window]),axis=0).tolist()}
                            append(cfg.STATIC_SAMPLES,sample);save(folder/'sample.json',sample)
                            report.update(status='sample_saved_holding',sample=sample);save(folder/'sampling_result.json',report)
                            print('静态样本已保存：电流mA='+str(sample['current_ma'])+'；继续保持，等托住后再卸力。',flush=True)
                            finished=True;stage='holding_after_sample'
                        elif began-release_time>=15:
                            report.update(status='sample_rejected_holding',reason='未稳定或输出饱和',position_span_deg=span.tolist(),saturated_axes=saturated)
                            save(folder/'sampling_result.json',report);print('本姿态未得到合格样本，继续保持；等待托住后卸力。',flush=True)
                            finished=True;stage='holding_rejected'
                time.sleep(max(0,.02-(time.monotonic()-began)))
    except BaseException as exc:
        report.update(exit_status='stopped',error=str(exc));print('停止：'+str(exc),flush=True)
        if bus.b:report['fault_context']={'id':bus.b.node,'recent_packets':list(bus.b.recent_packets)}
    finally:
        try:bus.close()
        except Exception as exc:report['cleanup_error']=str(exc)
        report['final_torque']=bus.final;save(folder/'result.json',report);signal.signal(signal.SIGTERM,previous)
        print('七轴最终使能状态：'+str(bus.final),flush=True);print('记录：'+str(folder/'result.json'),flush=True)

if __name__=='__main__':run_manual()
