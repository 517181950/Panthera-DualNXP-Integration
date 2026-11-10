"""Shared invocation plumbing for the separate no-argument Python programs.

Importing this module does not connect to hardware. Robot behavior remains in
the existing control/diagnostic programs; parameters live in 运行参数.py.
"""
from contextlib import contextmanager
from pathlib import Path
import os
import runpy
import sys

ROOT = Path(__file__).resolve().parent

@contextmanager
def program_context(script, arguments):
    previous_dir, previous_argv, previous_path = Path.cwd(), sys.argv, sys.path[:]
    try:
        os.chdir(ROOT)
        sys.argv = [str(ROOT / script), *map(str, arguments)]
        sys.path.insert(0, str((ROOT / script).parent))
        yield
    finally:
        os.chdir(previous_dir)
        sys.argv = previous_argv
        sys.path[:] = previous_path

def run_program(script, arguments):
    # Preserve --help so requesting help never starts an actuator.
    arguments = [*arguments, *sys.argv[1:]]
    with program_context(script, arguments):
        runpy.run_path(str(ROOT / script), run_name='__main__')

def run_teleop(side, seconds):
    from 运行参数 import TELEOP_CONFIG, GELLO_SOURCE
    script = 'run-one-gello-two-arms.py'
    arguments = ['--gello-source', GELLO_SOURCE, '--side', side,
                 '--seconds', seconds, '--execute', *sys.argv[1:]]
    with program_context(script, arguments):
        module = runpy.run_path(str(ROOT / script), run_name='manual_teleop')
        module['main'](sys.argv[1:], config_override=TELEOP_CONFIG)


def run_dual_gello_teleop(seconds):
    from 运行参数 import DUAL_GELLO_CONFIG
    script = 'run-one-gello-two-arms.py'
    arguments = ['--side', 'both', '--seconds', seconds, '--execute', *sys.argv[1:]]
    with program_context(script, arguments):
        module = runpy.run_path(str(ROOT / script), run_name='manual_dual_gello')
        module['main'](sys.argv[1:], config_override=DUAL_GELLO_CONFIG)
