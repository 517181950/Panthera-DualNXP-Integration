"""COM5 GELLO angle/speed viewer + selected arm gravity compensation.

--execute: GELLO readout and arm gravity. --gello-only: serial readout only.
Without either: offline GUI preview. Both GELLO groups are selectable.
"""
from gello_arm_compare import main

if __name__=='__main__':raise SystemExit(main())
