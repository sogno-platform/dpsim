# A two-bus network initialized from a powerflow.
# Documented at docs/hugo/content/en/docs/Tutorials/Python/two-bus-network/index.md
import dpsimpy
import villas.dataprocessing.readtools as rt

Vnom = 20e3

# --- powerflow ---
n1pf = dpsimpy.sp.SimNode("n1", dpsimpy.PhaseType.Single)
n2pf = dpsimpy.sp.SimNode("n2", dpsimpy.PhaseType.Single)
slack = dpsimpy.sp.ph1.NetworkInjection("slack")
slack.set_parameters(voltage_set_point=Vnom)
slack.set_base_voltage(Vnom)
slack.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.VD)
line = dpsimpy.sp.ph1.PiLine("line")
line.set_parameters(R=0.5, L=0.5 / 314, C=50e-6)
line.set_base_voltage(Vnom)
load = dpsimpy.sp.ph1.Load("load")
load.set_parameters(active_power=100e3, reactive_power=50e3, nominal_voltage=Vnom)
load.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PQ)
slack.connect([n1pf])
line.connect([n1pf, n2pf])
load.connect([n2pf])
syspf = dpsimpy.SystemTopology(50, [n1pf, n2pf], [slack, line, load])
lpf = dpsimpy.Logger("pf")
lpf.log_attribute("v1", "v", n1pf)
lpf.log_attribute("v2", "v", n2pf)
simpf = dpsimpy.Simulation("pf")
simpf.set_system(syspf)
simpf.set_domain(dpsimpy.Domain.SP)
simpf.set_solver(dpsimpy.Solver.NRP)
simpf.set_solver_component_behaviour(dpsimpy.SolverBehaviour.Initialization)
simpf.do_init_from_nodes_and_terminals(False)
simpf.set_time_step(0.1)
simpf.set_final_time(0.1)
simpf.add_logger(lpf)
simpf.run()

r = rt.read_timeseries_dpsim("logs/pf.csv")
print(
    "PF  |v1| = %.1f V   |v2| = %.1f V"
    % (abs(r["v1"].values[0]), abs(r["v2"].values[0]))
)

# --- dynamic, initialized from the powerflow ---
n1 = dpsimpy.dp.SimNode("n1", dpsimpy.PhaseType.Single)
n2 = dpsimpy.dp.SimNode("n2", dpsimpy.PhaseType.Single)
slackd = dpsimpy.dp.ph1.NetworkInjection("slack")
slackd.set_parameters(V_ref=complex(Vnom, 0))
lined = dpsimpy.dp.ph1.PiLine("line")
lined.set_parameters(
    series_resistance=0.5, series_inductance=0.5 / 314, parallel_capacitance=50e-6
)
loadd = dpsimpy.dp.ph1.RXLoad("load")
loadd.set_parameters(active_power=100e3, reactive_power=50e3, volt=Vnom)
slackd.connect([n1])
lined.connect([n1, n2])
loadd.connect([n2])
sysd = dpsimpy.SystemTopology(50, [n1, n2], [slackd, lined, loadd])
sysd.init_with_powerflow(systemPF=syspf, domain=dpsimpy.Domain.DP)

ld = dpsimpy.Logger("dyn")
ld.log_attribute("v2", "v", n2)
simd = dpsimpy.Simulation("dyn")
simd.set_system(sysd)
simd.set_domain(dpsimpy.Domain.DP)
simd.set_time_step(1e-3)
simd.set_final_time(0.05)
simd.add_logger(ld)
simd.run()
d = rt.read_timeseries_dpsim("logs/dyn.csv")["v2"]
print("DYN |v2| first=%.1f  last=%.1f" % (abs(d.values[1]), abs(d.values[-1])))
