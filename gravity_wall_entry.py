"""Independent manual-guidance experiment. Importing does not open hardware."""
from datetime import datetime
from pathlib import Path
import csv
import json
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'source'/'host'))
import numpy as np
from arm_dynamics import Gravity, read_json
from arm_lab import Session, ZERO, parser
from gravity_wall import GravityWall, WallSettings
from local_home import wait_for_local_home
from teleop_tuning import resolve_tuning
from 运行参数 import TELEOP_CONFIG
from 限位试验参数 import WALL_SETTINGS


def run(side):
    board = TELEOP_CONFIG['boards'][side]
    config = dict(TELEOP_CONFIG, boards={side: board})
    soft, _ = resolve_tuning(config, read_json(ROOT/config['soft_limits_file']))
    wall = GravityWall(soft[side]['lower_deg'], soft[side]['upper_deg'],
                       WallSettings(**WALL_SETTINGS[side]))
    if '--help' in sys.argv[1:]:
        print('无需参数；只运行所选侧重力补偿＋限位阻力，不运行回零轨迹。Ctrl+C请求柔顺保持。')
        return
    if sys.argv[1:]:
        raise ValueError('无需命令行参数；角度改运行参数.py，阻力改限位试验参数.py')
    args = parser('gravity').parse_args([])
    args.execute = True
    args.local, args.board = config['local'], board['ip']
    args.board_node, args.ports = board['node'], board['ports']
    args.hz = config['hz']
    args.identified = str(ROOT/board['identified'])
    args.output = str(ROOT/'logs'/('gravity_wall_'+side+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
    model = Gravity(args.urdf, None, args.identified, args.payload_mass, args.payload_com, args.gravity)
    print(f'{side}: 重力补偿限位试验；lower={soft[side]["lower_deg"]}, upper={soft[side]["upper_deg"]} °', flush=True)
    print(f'重力＋限位总力矩上限={WALL_SETTINGS[side]["max_torque_nm"]} Nm；角度过渡区={WALL_SETTINGS[side]["cushion_deg"]} °，低速减黏、快速双向阻尼抑制回弹，不能保证绝不越界。', flush=True)
    wait_for_local_home(config)
    try:
        with Session(args, model) as s:
            reference = s.configure_encoder_turns(board['encoder_modulo_joints'])
            s.summary.update(experiment='gravity_wall', side=side, wall_settings=WALL_SETTINGS[side],
                             soft_limits=soft[side], encoder_reference=reference)
            (s.directory/'wall-settings.json').write_text(json.dumps(s.summary, indent=2, ensure_ascii=False)+'\n')
            started = time.perf_counter()
            next_print = started
            with (s.directory/'wall.csv').open('w', newline='', buffering=1) as f:
                writer = csv.writer(f)
                writer.writerow(['t']+[f'{name}{i}' for name in ('q_deg', 'dq_deg_s', 'wall_nm', 'distance_deg', 'outside') for i in range(1,7)])
                while True:
                    state = s.state()
                    now = time.perf_counter()
                    gravity = args.gravity_scale*model(state['q_rad'])
                    torque, diag = wall.command(state['q_rad'], state['dq_rad_s'], now-started,
                                               gravity=gravity)
                    # Kp=0, dqref=0; motor-local Kd damps both directions.
                    # Do not also send host-computed damping as feedforward.
                    # Measured qref permits
                    # HOME_HOLD/SOFT_HOLD takeover without a reference jump.
                    s.step('manual', 'gravity_wall', state['q_rad'], ZERO, ZERO, diag['kd'],
                           extra=diag['feedforward_nm']-gravity, state=state)
                    writer.writerow([now-started]+np.rad2deg(state['q_rad']).tolist()+
                                    np.rad2deg(state['dq_rad_s']).tolist()+torque.tolist()+
                                    diag['distance_deg'].tolist()+diag['outside'].astype(int).tolist())
                    if now >= next_print:
                        active = [f'J{i+1}' for i in range(6) if diag['active'][i]]
                        outside = [f'J{i+1}' for i in range(6) if diag['outside'][i]]
                        print(f'限位阻力={active or "无"}；估算总Nm={np.round(diag["command_nm"], 2).tolist()}；MIT Kd={np.round(diag["kd"], 2).tolist()}；实测越界={outside or "无"}', flush=True)
                        next_print = now+1
    except KeyboardInterrupt:
        print('\n试验结束，已请求NXP柔顺保持。')


if __name__ == '__main__':
    raise SystemExit('请运行 原臂重力补偿限位.py 或 新臂重力补偿限位.py')
