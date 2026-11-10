"""Launch the connected GELLO and single Panthera arm; keep a console record."""
from datetime import datetime
from pathlib import Path
import json
import os
import runpy
import sys
import traceback

ROOT = Path(__file__).resolve().parent
stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
logs = ROOT / 'logs'
logs.mkdir(exist_ok=True)
output = logs / ('gello_single_arm_' + stamp)
console = logs / (output.name + '.console.log')
record_path = ROOT / 'evidence/current-single-arm-session.json'


class Tee:
    def __init__(self, terminal, log):
        self.terminal, self.log = terminal, log
    def write(self, text):
        self.terminal.write(text)
        self.log.write(text)
        return len(text)
    def flush(self):
        self.terminal.flush()
        self.log.flush()


script = ROOT / 'source/host/examples/18_gello-low-impedance.py'
sys.path.insert(0, str(script.parent))
sys.argv = [str(script), '--execute', '--group', '0', '--home-zero',
            '--local', '192.168.10.1', '--board', '192.168.10.12',
            '--board-node', '12', '--ports', '5015', '5016',
            '--port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB0182526-if00',
            '--gello-config', str(ROOT / 'gello-single-arm.json'),
            '--kp', '24.48', '61.2', '61.2', '12.24', '12.24', '12.24',
            '--kd', '1.2', '1.92', '1.92', '0.48', '0.48', '0.48',
            '--require-hand', '--log', '--seconds', '0', '--output', str(output)]
record = dict(pid=os.getpid(), started=datetime.now().isoformat(),
              status='starting', output=str(output), console=str(console), argv=sys.argv)
record_path.write_text(json.dumps(record, indent=2) + '\n')
exit_code = 0
stdout, stderr = sys.stdout, sys.stderr
with console.open('w', buffering=1) as log:
    sys.stdout, sys.stderr = Tee(stdout, log), Tee(stderr, log)
    try:
        print('Panthera 单臂＋灵巧手 GELLO：先缓慢回零，再使用历史高增益跟随，第 7 轴控制五指开合。', flush=True)
        print('扳机张开=手张开；扳机闭合=五指抓握。手指运动时请留出空间。', flush=True)
        print('请将 GELLO 放在已标定零位并保持不动；使用保存的六轴零点，显示 FOLLOW GELLO 后再小幅移动。', flush=True)
        print('Ctrl+C 结束跟随并尝试保持当前位置；不是断电或急停。', flush=True)
        print('日志：' + str(output), flush=True)
        record['status'] = 'process_running'
        record_path.write_text(json.dumps(record, indent=2) + '\n')
        runpy.run_path(str(script), run_name='__main__')
    except SystemExit as exc:
        exit_code = exc.code if isinstance(exc.code, int) else 1
    except BaseException:
        exit_code = 1
        traceback.print_exc()
    finally:
        record.update(status='exited', exit_code=exit_code, ended=datetime.now().isoformat())
        record_path.write_text(json.dumps(record, indent=2) + '\n')
        sys.stdout.flush()
        sys.stderr.flush()
        sys.stdout, sys.stderr = stdout, stderr
raise SystemExit(exit_code)
