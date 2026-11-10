"""Arm gravity + 60% friction; six Dear PyGui hand sliders, teleoperation mode 3.

Without --execute: offline GUI only. With --execute: arm 500 Hz / hand 50 Hz.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hand_slider_demo import main

if __name__=='__main__':raise SystemExit(main())
