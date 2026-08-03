# The same RL circuit in EMT and DP, with the DP result shifted back onto the carrier.
# Documented at docs/hugo/content/en/docs/Tutorials/Python/comparing-domains/index.md
import dpsimpy
import villas.dataprocessing.readtools as rt
from villas.dataprocessing.timeseries import TimeSeries as ts


def build(domain, ns, ph1, name, dt):
    gnd = ns.SimNode.gnd
    n1 = ns.SimNode("n1")
    n2 = ns.SimNode("n2")
    src = ph1.VoltageSource("src")
    src.set_parameters(
        V_ref=complex(100, 0), f_src=(50.0 if domain == dpsimpy.Domain.EMT else 0.0)
    )
    r = ph1.Resistor("r")
    r.set_parameters(R=10.0)
    l = ph1.Inductor("l")
    l.set_parameters(L=0.05)
    src.connect([gnd, n1])
    r.connect([n1, n2])
    l.connect([n2, gnd])
    system = dpsimpy.SystemTopology(50, [gnd, n1, n2], [src, r, l])
    log = dpsimpy.Logger(name)
    log.log_attribute("i_l", "i_intf", l)
    sim = dpsimpy.Simulation(name)
    sim.set_domain(domain)
    sim.set_system(system)
    sim.set_time_step(dt)
    sim.set_final_time(0.06)
    sim.add_logger(log)
    sim.run()
    return rt.read_timeseries_dpsim("logs/%s.csv" % name)


emt = build(dpsimpy.Domain.EMT, dpsimpy.emt, dpsimpy.emt.ph1, "cmp_emt", 5e-5)["i_l"]
dp = build(dpsimpy.Domain.DP, dpsimpy.dp, dpsimpy.dp.ph1, "cmp_dp", 1e-3)
dp_shift = ts.frequency_shift_list(dp, 50)["i_l_shift"]
print("EMT  samples=%d  dt=50us" % len(emt.time))
print("DP   samples=%d  dt=1ms" % len(dp["i_l"].time))
for t in (0.02, 0.04, 0.06):
    ke = min(range(len(emt.time)), key=lambda i: abs(emt.time[i] - t))
    kd = min(range(len(dp_shift.time)), key=lambda i: abs(dp_shift.time[i] - t))
    print(
        "t=%.3f  EMT i=%8.4f   DP(shifted) i=%8.4f"
        % (t, emt.values[ke], dp_shift.values[kd])
    )
import pickle

pickle.dump(
    {
        "emt": (list(emt.time), list(emt.values)),
        "dp": (list(dp_shift.time), list(dp_shift.values)),
        "env": (list(dp["i_l"].time), [abs(v) for v in dp["i_l"].values]),
    },
    open("cmp.pkl", "wb"),
)
