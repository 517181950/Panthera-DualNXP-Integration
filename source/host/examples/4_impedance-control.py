"""impedance-control: run --help for options; --execute starts the experiment."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_lab import main

if __name__ == '__main__':
    raise SystemExit(main('impedance'))
