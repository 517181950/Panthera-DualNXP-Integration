"""Allow direct IDE/interpreter launches of a Windows Conda environment.

Conda activation normally adds Library/bin; without it NumPy LAPACK delay-load
can terminate the process even though importing NumPy succeeds.
"""
import os
from pathlib import Path
import sys

_dll_handles=[]
if sys.platform=='win32' and (Path(sys.prefix)/'conda-meta').is_dir():
    for folder in (Path(sys.prefix)/'Library'/'bin',Path(sys.prefix)/'DLLs'):
        if folder.is_dir():
            os.environ['PATH']=str(folder)+os.pathsep+os.environ.get('PATH','')
            _dll_handles.append(os.add_dll_directory(str(folder)))
