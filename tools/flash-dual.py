"""Explicitly flash one identified NXP, save old application and verify every byte."""
import argparse
from datetime import datetime
import hashlib
import json
import time
from pathlib import Path
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
from firmware_image import maintenance_address

root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('side',choices=['left','right']);p.add_argument('--image',type=Path,help='Explicit alternative image, e.g. saved rollback binary');p.add_argument('--backup-dir',type=Path)
p.add_argument('--maintenance-elf',type=Path,help='Matching ELF: inhibit homing on this flash boot; current firmware waits for explicit host takeover or SW2')
args=p.parse_args()
serial={'left':'KWP5LJ1OK2D5A','right':'FVIZ5SIGKZEBZ'}[args.side]
image=(args.image or root/f'build-{args.side}/zephyr/zephyr.bin').resolve();data=image.read_bytes()
inhibit_address=None
if args.maintenance_elf:
    inhibit_address=maintenance_address(args.maintenance_elf,data)
folder=args.backup_dir or root/'evidence/before-dual-arm-hand'
folder.mkdir(parents=True,exist_ok=True)
session=ConnectHelper.session_with_chosen_probe(unique_id=serial,target_override='mcxn947',
    pack='/home/aster/.local/share/cmsis-pack-manager/NXP/MCXN947_DFP/26.09.00.pack',
    frequency=1000000,connect_mode='halt',options={'resume_on_disconnect':False,'cache.enable_memory':False})
if session is None:raise RuntimeError('Selected probe unavailable')
with session:
    target=session.target;target.halt()
    backup=folder/(args.side+'-application-before-flash.bin')
    if not backup.exists():backup.write_bytes(bytes(target.read_memory_block8(0,200704)))
    target.reset_and_halt()
    FileProgrammer(session,chip_erase='sector',smart_flash=False).program(str(image),base_address=0)
    target.reset_and_halt()
    readback=bytes(target.read_memory_block8(0,len(data)))
    if readback!=data:raise RuntimeError('Flash readback differs; target left halted')
    if inhibit_address is not None:
        target.write_memory_block32(inhibit_address,[0x50484d31,0xafb7b2ce])
        if target.read_memory_block32(inhibit_address,2)!=[0x50484d31,0xafb7b2ce]:raise RuntimeError('Maintenance latch write failed; target left halted')
    # Stay halted through debugger reattachment; execute startup exactly once.
    report={'side':args.side,'serial':serial,'image':str(image),'bytes':len(data),
      'sha256':hashlib.sha256(data).hexdigest(),'readback_verified':True,
      'backup':str(backup),'flashed_at':datetime.now().isoformat(),'state':str(target.get_state()),
      'maintenance_on_this_boot':inhibit_address is not None,'maintenance_address':inhibit_address}
# Reattach after flash/debug reset sequences, then explicitly release the core.
# The MCXN947 pack can leave a reset/debug catch after the programming session.
run_session=ConnectHelper.session_with_chosen_probe(unique_id=serial,target_override='mcxn947',
    pack='/home/aster/.local/share/cmsis-pack-manager/NXP/MCXN947_DFP/26.09.00.pack',
    frequency=1000000,connect_mode='attach',options={'resume_on_disconnect':False,'cache.enable_memory':False})
with run_session:
    if inhibit_address is not None:
        run_session.target.halt()
        if run_session.target.read_memory_block32(inhibit_address,2)!=[0x50484d31,0xafb7b2ce]:
            raise RuntimeError('Maintenance latch lost before resume; target left halted')
    run_session.target.resume();time.sleep(.2)
    report['state']=str(run_session.target.get_state())
(root/f'evidence/dual-flash-{args.side}.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
