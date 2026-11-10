"""Independent or shared GELLO inputs; construction never opens hardware."""
from pathlib import Path
import json
import time
import numpy as np
from board_gello import BoardGelloReader


def source_specs(config):
    if 'gellos' in config:
        missing=set(config['boards'])-set(config['gellos'])
        if missing:raise ValueError(f'缺少对应 GELLO 配置：{sorted(missing)}')
        return {side:dict(config['gellos'][side]) for side in config['boards']}
    return {'shared':dict(config['gello'])}


def read_profile(root, spec, *, require_zero=True):
    path=Path(root)/spec['config']
    try:profile=json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise ValueError(f'缺少 GELLO 独立标定 {path}；先运行 tools/calibrate-gello.py --help') from exc
    group=spec.get('profile_group',('left','right')[spec['group']])
    data=profile.get(group,{})
    if 'axes' not in data or 'gripper' not in data:
        raise ValueError(f'{path}: {group} 标定未完成，需zero/open/closed三项')
    if require_zero and 'zero_raw_ticks' not in data:
        raise ValueError(f'{path}: {group} 未保存六轴零位，请先完成独立标定')
    axes=data['axes']
    if len(axes)!=6 or any(a['sign'] not in (-1,1) or not np.isfinite(a['scale']) or a['scale']<=0
                          or not np.isfinite(a['offset_rad']) for a in axes):
        raise ValueError(f'{path}: 六轴方向/比例/偏置无效')
    grip=data['gripper'];lo,hi=grip['min_raw'],grip['max_raw']
    if not np.isfinite([lo,hi]).all() or not 0<hi-lo<4096 or not isinstance(grip['inverted'],bool):
        raise ValueError(f'{path}: 扳机标定无效')
    return profile,data


def make_reader(spec, profile, local, teleop):
    transport=spec.get('transport','usb')
    if transport in ('nxp-usb','nxp-uart'):
        return BoardGelloReader(board=spec.get('board','192.168.10.12'),local=local,
                                node=spec.get('node',12),port=spec.get('query_port',5021),
                                group_mask=spec.get('group_mask',1<<spec['group']))
    return teleop.GelloReader(spec['port'],baudrate=profile['serial']['baud'],
        group_mask=spec.get('group_mask',1<<spec['group']),timeout_s=.03,read_velocity=True,
        velocity_rpm_per_unit=profile['serial'].get('velocity_rpm_per_unit',.732))


class GelloInputs:
    def __init__(self,config,root,teleop,reader_factory=make_reader):
        self.teleop=teleop;self.specs=source_specs(config);self.shared='gellos' not in config
        self.sides=list(config['boards']);self.readers={};self.mappings={};self.grips={};self.ready={}
        profiles={};endpoints={};endpoint_by_name={}
        # Complete all validation before creating any reader, much less sessions.
        for name,spec in self.specs.items():
            transport=spec.get('transport','usb');group=spec['group']
            if transport not in ('usb','nxp-usb','nxp-uart') or group not in (0,1):
                raise ValueError(f'{name}: 无效 GELLO transport/group')
            if transport.startswith('nxp-'):
                endpoint=('nxp',spec.get('board','192.168.10.12'),spec.get('query_port',5021))
            else:endpoint=('usb',str(Path(spec['port']).resolve()))
            for other in endpoints.get(endpoint,[]):
                old=self.specs[other]
                if old['group']==group:
                    raise ValueError('双 GELLO 不能把同一个输入组复制给两臂')
                if old.get('node',12)!=spec.get('node',12):
                    raise ValueError('同一总线的板身份配置不一致')
            endpoints.setdefault(endpoint,[]).append(name);endpoint_by_name[name]=endpoint
            profile,data=read_profile(root,spec);profiles[name]=profile
            self.mappings[name]=teleop.FixedGelloZero(data);self.grips[name]=data['gripper'];self.ready[name]=False
        buses={}
        for name,spec in self.specs.items():
            endpoint=endpoint_by_name[name]
            if endpoint not in buses:
                peers=endpoints[endpoint]
                if any(profiles[p]['serial']['baud']!=profiles[name]['serial']['baud'] for p in peers):
                    raise ValueError('同一总线的GELLO波特率必须相同')
                bus_spec=dict(spec,group_mask=sum(1<<self.specs[p]['group'] for p in peers))
                buses[endpoint]=reader_factory(bus_spec,profiles[name],config['local'],teleop)
            self.readers[name]=buses[endpoint]
            grip=self.grips[name]
            self.readers[name].set_gripper_calibration(spec['group'],grip['min_raw'],grip['max_raw'])

    def start(self):
        try:
            for reader in dict.fromkeys(self.readers.values()):reader.start()
        except BaseException:
            self.stop();raise

    def wait_ready(self,timeout=2):
        self.start();deadline=time.perf_counter()+timeout
        while True:
            missing=[];now=time.perf_counter()
            for name,reader in self.readers.items():
                group=reader.latest(self.specs[name]['group'])
                if (not reader.is_connected or group is None or not group.data_valid
                    or any(t is None or not 0<=now-t<.15 for t in group.servo_captured_at)):
                    missing.append(name)
            if not missing:return
            if now>=deadline:
                raise RuntimeError(f'GELLO七节点未全部就绪：{missing}；检查对应板接线/固件，未接管机械臂')
            time.sleep(.01)

    def poll(self,now):
        samples={}
        for name,reader in self.readers.items():
            group=reader.latest(self.specs[name]['group'])
            values=self.teleop.gello_values(group,now,stale_s=.15)
            fresh=(group is not None and all(t is not None and 0<=now-t<=.15 for t in group.servo_captured_at[:6]))
            desired=self.mappings[name].target(values['ticks']) if reader.is_connected and fresh else None
            if not self.ready[name]:
                if desired is not None and np.max(np.abs(desired))<=np.deg2rad(3):
                    self.ready[name]=True;print(f'{name}: GELLO SAVED ZERO READY',flush=True)
                else:desired=None
            grasp=self.teleop.grasp_percent(group,now,self.grips[name]['inverted'],.15) if desired is not None else None
            samples[name]=(desired,grasp)
        return {side:samples['shared' if self.shared else side] for side in self.sides}

    def stop(self):
        for reader in dict.fromkeys(self.readers.values()):reader.stop()
