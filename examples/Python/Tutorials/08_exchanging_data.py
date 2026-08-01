# Exporting a value to another tool through the VILLASnode interface, using a file node.
# Documented at docs/hugo/content/en/docs/Tutorials/Python/exchanging-data/index.md
import json

import dpsimpy
import dpsimpyvillas

name = "exchanging_data"
time_step = 0.01
final_time = 1.0

gnd = dpsimpy.dp.SimNode.gnd
n1 = dpsimpy.dp.SimNode("n1", dpsimpy.PhaseType.Single, [10])
n2 = dpsimpy.dp.SimNode("n2", dpsimpy.PhaseType.Single, [5])

# The component whose current is handed to the other side.
boundary = dpsimpy.dp.ph1.VoltageSource("boundary")
boundary.set_parameters(complex(5, 0))

src = dpsimpy.dp.ph1.VoltageSource("src")
src.set_parameters(complex(10, 0))

line = dpsimpy.dp.ph1.Resistor("line")
line.set_parameters(1)

src.connect([gnd, n1])
line.connect([n1, n2])
boundary.connect([gnd, n2])

system = dpsimpy.SystemTopology(50, [n1, n2], [src, line, boundary])

logger = dpsimpy.Logger(name)
logger.log_attribute("v2", "v", n2)
logger.log_attribute("boundary.i", "i_intf", boundary)

# A real-time simulation, because the exchange is paced by the wall clock.
sim = dpsimpy.RealTimeSimulation(name)
sim.set_system(system)
sim.set_time_step(time_step)
sim.set_final_time(final_time)
sim.add_logger(logger)

# This configuration belongs to VILLASnode, not to DPsim.
interface_config = {
    "type": "file",
    "format": "csv",
    "uri": "logs/exchanged.csv",
    "out": {"flush": True},
}

interface = dpsimpyvillas.InterfaceVillas(
    name="dpsim-file", config=json.dumps(interface_config)
)
# The second argument is the position in the signal list, not a name.
interface.export_attribute(boundary.attr("i_intf").derive_coeff(0, 0), 0)
sim.add_interface(interface)

boundary.set_intf_current([[complex(5, 0)]])
sim.run(1)

with open("logs/exchanged.csv") as handle:
    lines = handle.read().splitlines()
print("rows written:", len(lines) - 1)
print("header:", lines[0])
print("first:", lines[1])
