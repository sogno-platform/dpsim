# Applying and clearing a fault during a run.
# Documented at docs/hugo/content/en/docs/Tutorials/Python/applying-a-fault/index.md
import dpsimpy
import villas.dataprocessing.readtools as rt

name = "fault"
Vnom = 20e3

gnd = dpsimpy.dp.SimNode.gnd
n1 = dpsimpy.dp.SimNode("n1", dpsimpy.PhaseType.Single)
n2 = dpsimpy.dp.SimNode("n2", dpsimpy.PhaseType.Single)

src = dpsimpy.dp.ph1.VoltageSource("src")
src.set_parameters(V_ref=complex(Vnom, 0))

line = dpsimpy.dp.ph1.PiLine("line")
line.set_parameters(
    series_resistance=0.5, series_inductance=0.5 / 314, parallel_capacitance=50e-6
)

load = dpsimpy.dp.ph1.RXLoad("load")
load.set_parameters(active_power=100e3, reactive_power=50e3, volt=Vnom)

fault = dpsimpy.dp.ph1.Switch("fault")
fault.set_parameters(open_resistance=1e9, closed_resistance=10.0)
fault.open()

src.connect([gnd, n1])
line.connect([n1, n2])
load.connect([n2])
fault.connect([gnd, n2])

system = dpsimpy.SystemTopology(50, [gnd, n1, n2], [src, line, load, fault])

log = dpsimpy.Logger(name)
log.log_attribute("v2", "v", n2)
log.log_attribute("i_fault", "i_intf", fault)

sim = dpsimpy.Simulation(name)
sim.set_domain(dpsimpy.Domain.DP)
sim.set_system(system)
sim.set_time_step(1e-4)
sim.set_final_time(0.3)
sim.add_logger(log)
sim.add_event(dpsimpy.event.SwitchEvent(0.1, fault, True))
sim.add_event(dpsimpy.event.SwitchEvent(0.2, fault, False))
sim.run()

r = rt.read_timeseries_dpsim("logs/%s.csv" % name)
v = r["v2"]
for t in (0.05, 0.101, 0.15, 0.199, 0.21, 0.29):
    k = min(range(len(v.time)), key=lambda i: abs(v.time[i] - t))
    print("t=%.3f  |v2|=%9.1f V" % (v.time[k], abs(v.values[k])))
