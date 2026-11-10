"""Pause/resume only GELLO UART queries via SWD, without resetting the NXP.

Requires the currently installed matching candidate ELF/bin. No arm or hand
state is changed. Use passive mode before exchanging UART wires.
"""
import argparse
from pathlib import Path
import time
from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['pause','resume'])
    args=parser.parse_args()
    candidate=ROOT/'evidence/board-gello-candidate'
    with (candidate/'right.elf').open('rb') as f:
        symbols={s.name:s['st_value'] for s in ELFFile(f).get_section_by_name('.symtab').iter_symbols()
                 if s.name in ('gello_debug_passive','gello_debug_baud')}
    if len(symbols)!=2:raise RuntimeError('Candidate does not expose GELLO UART diagnostics')
    expected=(candidate/'right.bin').read_bytes()
    session=ConnectHelper.session_with_chosen_probe(unique_id='FVIZ5SIGKZEBZ',target_override='mcxn947',
        pack='/home/aster/.local/share/cmsis-pack-manager/NXP/MCXN947_DFP/26.09.00.pack',
        frequency=1000000,connect_mode='attach',options={'resume_on_disconnect':False,'cache.enable_memory':False})
    if session is None:raise RuntimeError('Original-arm NXP probe unavailable')
    with session:
        target=session.target
        if bytes(target.read_memory_block8(0,len(expected)))!=expected:
            raise RuntimeError('Installed image differs from candidate; nothing written')
        target.write32(symbols['gello_debug_baud'],1000000)
        target.write32(symbols['gello_debug_passive'],int(args.mode=='pause'))
        time.sleep(.1)
        assert target.read32(symbols['gello_debug_passive'])==int(args.mode=='pause')
        # Matching image uses UART5 at 0x500b5000. Stop RX IRQ traffic as
        # well as TX during pause, then explicitly restore RX on resume.
        ctrl=0x500b5018;rx_bits=(1<<21)|(1<<18)
        value=target.read32(ctrl)
        target.write32(ctrl,(value&~rx_bits) if args.mode=='pause' else (value|rx_bits))
        print('GELLO UART5:',args.mode,'(1 Mbps; NXP not reset; robot control untouched)')

if __name__=='__main__':main()
