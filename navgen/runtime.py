"""Numba threads and a source-versioned compiled-code cache (never map data)."""
import hashlib
import os
from pathlib import Path

import llvmlite
import numba
import numpy as np
import scipy
from numba import config, get_num_threads, get_thread_id, prange, set_num_threads

# A cached serial trace can call get_thread_id before any parallel dispatcher
# has launched the backend. Initialize it through the public API before loading
# such native code (also covers direct Collision/compare users, not just CLI).
get_num_threads()


def source_fingerprint(folder=None):
    folder=Path(folder) if folder is not None else Path(__file__).parent
    digest=hashlib.sha256()
    for path in sorted(folder.glob('*.py')):
        digest.update(path.name.encode());digest.update(b'\0');digest.update(path.read_bytes())
    digest.update(repr((numba.__version__,llvmlite.__version__,np.__version__,scipy.__version__)).encode())
    return digest.hexdigest()


# Numba alone does not invalidate callers when an imported helper changes.
CACHE_FINGERPRINT=source_fingerprint()
CACHE_ROOT=Path(os.environ.get('NUMBA_CACHE_DIR') or Path(__file__).parent/'__pycache__'/'numba')
config.CACHE_DIR=str(CACHE_ROOT/CACHE_FINGERPRINT)


def njit(*args,**kwargs):
    kwargs.setdefault('cache',True)
    return numba.njit(*args,**kwargs)


def configure_threads(threads=None):
    threads=config.NUMBA_NUM_THREADS if threads is None else threads
    if isinstance(threads,bool) or not isinstance(threads,int) or not 1<=threads<=config.NUMBA_NUM_THREADS:
        raise ValueError(f'Threads must be an integer from 1 to {config.NUMBA_NUM_THREADS}')
    set_num_threads(threads)
    return get_num_threads()


def trace_count(scene):
    return int(scene[-1].sum())
