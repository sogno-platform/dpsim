from . import matpower
from .matpower import Reader

try:
    from dpsimpy import *
except ImportError:  # pragma: no cover
    print('Error: Could not find dpsim C++ module.')

__all__ = ['matpower']
