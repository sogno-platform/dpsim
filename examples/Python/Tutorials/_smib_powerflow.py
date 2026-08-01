# Shared powerflow for the machine tutorial; imported by 06_a_machine.py.
import dpsimpy, math
import villas.dataprocessing.readtools as rt

Vnom, Snom, fnom = 24e3, 555e6, 60.0
P, Q = 300e6, 0.0


def powerflow():
    n1 = dpsimpy.sp.SimNode("n1", dpsimpy.PhaseType.Single)
    n2 = dpsimpy.sp.SimNode("n2", dpsimpy.PhaseType.Single)
    gen = dpsimpy.sp.ph1.SynchronGenerator("gen")
    gen.set_parameters(
        rated_apparent_power=Snom,
        rated_voltage=Vnom,
        set_point_active_power=P,
        set_point_voltage=Vnom,
        powerflow_bus_type=dpsimpy.PowerflowBusType.PV,
    )
    gen.set_base_voltage(Vnom)
    line = dpsimpy.sp.ph1.PiLine("line")
    line.set_parameters(R=0.02, L=0.0008, C=0.0)
    line.set_base_voltage(Vnom)
    slack = dpsimpy.sp.ph1.NetworkInjection("slack")
    slack.set_parameters(voltage_set_point=Vnom)
    slack.set_base_voltage(Vnom)
    slack.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.VD)
    gen.connect([n1])
    line.connect([n1, n2])
    slack.connect([n2])
    sysPF = dpsimpy.SystemTopology(fnom, [n1, n2], [gen, line, slack])
    log = dpsimpy.Logger("sg_pf")
    log.log_attribute("v1", "v", n1)
    sim = dpsimpy.Simulation("sg_pf")
    sim.set_system(sysPF)
    sim.set_domain(dpsimpy.Domain.SP)
    sim.set_solver(dpsimpy.Solver.NRP)
    sim.set_solver_component_behaviour(dpsimpy.SolverBehaviour.Initialization)
    sim.do_init_from_nodes_and_terminals(False)
    sim.set_time_step(0.1)
    sim.set_final_time(0.1)
    sim.add_logger(log)
    sim.run()
    return sysPF


sysPF = powerflow()
r = rt.read_timeseries_dpsim("logs/sg_pf.csv")
print(
    "PF gen bus |v| = %.1f V (%.4f pu)"
    % (abs(r["v1"].values[0]), abs(r["v1"].values[0]) / Vnom)
)
