# Adding dynamics: an RL branch, run at two time steps to show what the step buys.
# Documented at docs/hugo/content/en/docs/Tutorials/Python/adding-dynamics/index.md
import sys
import dpsimpy
import villas.dataprocessing.readtools as rt


def run(name, dt):
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    n2 = dpsimpy.dp.SimNode("n2")
    src = dpsimpy.dp.ph1.VoltageSource("src")
    src.V_ref = complex(100, 0)
    r = dpsimpy.dp.ph1.Resistor("r")
    r.R = 10.0
    l = dpsimpy.dp.ph1.Inductor("l")
    l.L = 0.05
    src.connect([gnd, n1])
    r.connect([n1, n2])
    l.connect([n2, gnd])
    system = dpsimpy.SystemTopology(50, [gnd, n1, n2], [src, r, l])
    log = dpsimpy.Logger(name)
    log.log_attribute("i_l", "i_intf", l)
    sim = dpsimpy.Simulation(name)
    sim.set_domain(dpsimpy.Domain.DP)
    sim.set_system(system)
    sim.set_time_step(dt)
    sim.set_final_time(0.05)
    sim.add_logger(log)
    sim.run()
    return rt.read_timeseries_dpsim("logs/%s.csv" % name)["i_l"]


fine = run("rl_fine", 1e-4)
coarse = run("rl_coarse", 5e-3)
print("tau = L/R = %.4f s" % (0.05 / 10))
for t in (0.005, 0.010, 0.020, 0.050):
    fi = min(range(len(fine.time)), key=lambda k: abs(fine.time[k] - t))
    ci = min(range(len(coarse.time)), key=lambda k: abs(coarse.time[k] - t))
    print(
        "t=%.3f  fine |i|=%7.4f   coarse |i|=%7.4f"
        % (t, abs(fine.values[fi]), abs(coarse.values[ci]))
    )
