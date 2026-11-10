"""Read node12 USB host diagnostics via SWD attach, without halt or reset."""
import argparse
import json
from pathlib import Path
from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper

ROOT = Path(__file__).resolve().parents[1]

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate', type=Path, default=ROOT/'evidence/board-gello-usb-candidate')
    p.add_argument('--output', type=Path)
    a = p.parse_args()
    names = ('gello_usb_state', 'gello_usb_vid_pid', 'gello_usb_rx_bytes',
             'gello_usb_tx_bytes', 'gello_usb_errors', 'gello_usb_timeouts',
             'gello_usb_connections', 'gello_debug_passive')
    with (a.candidate/'right.elf').open('rb') as f:
        symbols = {s.name:s['st_value'] for s in ELFFile(f).get_section_by_name('.symtab').iter_symbols()
                   if s.name in names}
    if len(symbols) != len(names):
        raise RuntimeError('USB diagnostic symbols missing')
    expected = (a.candidate/'right.bin').read_bytes()
    session = ConnectHelper.session_with_chosen_probe(
        unique_id='FVIZ5SIGKZEBZ', target_override='mcxn947',
        pack='/home/aster/.local/share/cmsis-pack-manager/NXP/MCXN947_DFP/26.09.00.pack',
        frequency=1000000, connect_mode='attach',
        options={'resume_on_disconnect':False,'cache.enable_memory':False})
    if session is None:
        raise RuntimeError('Original-arm probe unavailable')
    with session:
        target = session.target
        if bytes(target.read_memory_block8(0,len(expected))) != expected:
            raise RuntimeError('Installed image differs; no symbol addresses read')
        result = {n:target.read32(symbols[n]) for n in names}
        value = result['gello_usb_state']
        if value & 0x80000000:
            result['gello_usb_state'] = value - 0x100000000
        result['core_state'] = str(target.get_state())
    encoded = json.dumps(result, indent=2)
    if a.output:
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(encoded+'\n')
    print(encoded)

if __name__ == '__main__':
    main()
