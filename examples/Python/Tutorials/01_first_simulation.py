# First simulation: a voltage source feeding a resistor.
# Documented at docs/hugo/content/en/docs/Tutorials/Python/first-simulation/index.md
import dpsimpy
import villas.dataprocessing.readtools as rt

name = "first_simulation"

gnd = dpsimpy.dp.SimNode.gnd
n1 = dpsimpy.dp.SimNode("n1")

src = dpsimpy.dp.ph1.VoltageSource("src")
src.V_ref = complex(100, 0)

load = dpsimpy.dp.ph1.Resistor("load")
load.R = 10.0

src.connect([gnd, n1])
load.connect([n1, gnd])

system = dpsimpy.SystemTopology(50, [gnd, n1], [src, load])

logger = dpsimpy.Logger(name)
logger.log_attribute("n1.v", "v", n1)
logger.log_attribute("load.i_intf", "i_intf", load)

sim = dpsimpy.Simulation(name)
sim.set_domain(dpsimpy.Domain.DP)
sim.set_system(system)
sim.set_time_step(1e-3)
sim.set_final_time(0.1)
sim.add_logger(logger)
sim.run()

results = rt.read_timeseries_dpsim("logs/" + name + ".csv")
voltage = results["n1.v"]
print("t=%.3f  |v|=%.1f V" % (voltage.time[1], abs(voltage.values[1])))
