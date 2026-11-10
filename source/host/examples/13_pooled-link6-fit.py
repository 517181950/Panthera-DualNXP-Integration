"""Combine completed datasets and fit link6 only, with equal point weights."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pooled_link6_fit import main

if __name__=='__main__':raise SystemExit(main())
