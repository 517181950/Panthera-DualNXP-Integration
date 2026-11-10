"""Read-only local homing preflight for the shared GELLO launcher."""
import time
from dual_peer import PeerMonitor
PHASES={0:'电脑可接管',1:'等待六轴静止反馈',2:'回零增益渐变',3:'其余五轴回零',4:'等待到位',5:'零位保持',6:'故障停止',7:'J2先回零（其余轴保持）',8:'等待J2到位',9:'协调抬臂至竖直',10:'等待竖直到位',11:'竖直姿态对正J1',12:'等待对正',13:'协调降回零位',14:'等待零位稳定'}
ERRORS={0:'无',1:'六轴反馈无效或超过20ms',2:'电机故障',3:'起点超出回零允许范围或整圈零位不明确',4:'运行中编码器跳圈',5:'回零/静止超时',6:'非有限反馈',7:'双板回零确认/通信或对侧反馈失败',8:'回零跟踪异常或电脑请求中止',9:'板内回零控制超时',10:'烧录后检修锁，未启动回零'}
PHASES[15]='已停止，等待程序接管'
PHASES[16]='柔顺保持（重力补偿＋阻尼）'
ERRORS[11]='电脑运动目标超过50ms未更新'
def wait_for_local_home(config, timeout=90, *, monitor_factory=PeerMonitor, clock=time.monotonic, sleep=time.sleep):
    monitor=monitor_factory(config['local']);deadline=clock()+timeout;last=None;next_print=0
    try:
        while True:
            status={};ready=True
            for side,b in config['boards'].items():
                s=monitor.query(b['node'])['local'];phase=s.get('home_state',0)
                if phase==6:
                    raise RuntimeError(f'{side}: NXP本地回零故障：{ERRORS.get(s.get("home_error"),s.get("home_error"))}；排除原因后按本侧SW2重试')
                if phase not in PHASES:raise RuntimeError(f'{side}: unknown local homing state {phase}')
                status[side]=PHASES[phase];ready=ready and phase in (0,5,15,16)
            if ready:return status
            now=clock()
            if status!=last or now>=next_print:
                print('等待 NXP 本地回零：'+str(status),flush=True);last=status;next_print=now+5
            if now>=deadline:raise RuntimeError('等待 NXP 回零超时；请检查六轴供电与反馈，未启动GELLO控制')
            sleep(.1)
    finally:monitor.close()
