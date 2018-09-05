"""DPsim

"""

import types
import sys

__author__ = "Markus Mirz, Steffen Vogel"
__copyright__ = "Copyright 2017, Institute for Automation of Complex Power Systems, EONERC"
__credits__ = [ "Georg Reinke", "Steffen Vogel", "Markus Mirz" ]
__license__ = "GPL-3.0"
__version__ = "0.1.0"
__maintainer__ = "Steffen Vogel"
__email__ = "stvogel@eonerc.rwth-aachen.de"
__status__ = "Beta"

import _dpsim

from _dpsim import Interface
from _dpsim import SystemTopology

from _dpsim import load_cim
from _dpsim import open_interface

from .Simulation import Simulation, RealTimeSimulation
from .EventChannel import EventChannel

def __get_module(parts):
    full_name = ".".join(parts)
    name = parts[-1]

    if full_name in sys.modules:
        return sys.modules[full_name]
    else:
        module = types.ModuleType(name, 'Module created to provide a context for tests')
        sys.modules[full_name] = module

        parent = ".".join(parts[:-1])
        if parent == "":
            globals()[name] = module
        else:
            if parent in sys.modules:
                parent_module = sys.modules[parent]
            else:
                parent_module = __get_module(parts[:-1])

            parent_module.__dict__[name] = module

        return module

# Register aliases
for ctor in [x for x in _dpsim.__dict__ if re.match("^_(dp|emt|signal)", x) ]:
    parts = ctor.split("_")

    module_parts = parts[1:-1]
    name = parts[-1]

    module = __get_module(module_parts)
    module.__dict__[name] = _dpsim.__dict__[ctor]

__all__ = [
    'Component',
    'Interface',
    'Simulation',
    'load_cim',
    'open_interface'
]
