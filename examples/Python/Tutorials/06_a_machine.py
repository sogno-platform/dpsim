# A synchronous machine through a fault, compared across three model orders.
# Documented at docs/hugo/content/en/docs/Tutorials/Python/a-machine/index.md
exec(open(__file__.replace("06_a_machine.py", "_smib_powerflow.py")).read())


def dyn(cls, name, setup):
    n1 = dpsimpy.dp.SimNode("n1", dpsimpy.PhaseType.Single)
    n2 = dpsimpy.dp.SimNode("n2", dpsimpy.PhaseType.Single)
    gen = cls("gen")
    setup(gen)
    line = dpsimpy.dp.ph1.PiLine("line")
    line.set_parameters(
        series_resistance=0.02, series_inductance=0.0008, parallel_capacitance=0.0
    )
    slack = dpsimpy.dp.ph1.NetworkInjection("slack")
    slack.set_parameters(V_ref=complex(Vnom, 0))
    fault = dpsimpy.dp.ph1.varResSwitch("fault")
    fault.set_parameters(open_resistance=1e9, closed_resistance=0.02)
    fault.open()
    fault.set_init_parameters(1e-4)
    gen.connect([n1])
    line.connect([n1, n2])
    slack.connect([n2])
    fault.connect([dpsimpy.dp.SimNode.gnd, n1])
    sysDP = dpsimpy.SystemTopology(fnom, [n1, n2], [gen, line, slack, fault])
    sysDP.init_with_powerflow(systemPF=sysPF, domain=dpsimpy.Domain.DP)
    vterm = n1.initial_single_voltage()
    sterm = gen.attr("s_intf").get() if False else complex(P, 0)
    gen.set_initial_values(
        init_complex_electrical_power=sterm,
        init_mechanical_power=P,
        init_complex_terminal_voltage=vterm,
    )
    print(
        "   %s: terminal %.1f V angle %.3f rad"
        % (name, abs(vterm), __import__("cmath").phase(vterm))
    )
    log = dpsimpy.Logger(name)
    log.log_attribute("wr", "w_r", gen)
    sim = dpsimpy.Simulation(name)
    sim.set_system(sysDP)
    sim.set_domain(dpsimpy.Domain.DP)
    sim.set_time_step(1e-4)
    sim.set_final_time(3.0)
    sim.add_logger(log)
    sim.add_event(dpsimpy.event.SwitchEvent(0.5, fault, True))
    sim.add_event(dpsimpy.event.SwitchEvent(0.6, fault, False))
    sim.run()
    return rt.read_timeseries_dpsim("logs/%s.csv" % name)["wr"]


base = dict(
    nom_power=Snom,
    nom_voltage=Vnom,
    nom_frequency=fnom,
    H=3.7,
    Ld=1.81,
    Lq=1.76,
    L0=0.15,
    Ld_t=0.3,
)
w3 = dyn(
    dpsimpy.dp.ph1.SynchronGenerator3OrderVBR,
    "sg3",
    lambda g: g.set_operational_parameters_per_unit(Td0_t=8.0, **base),
)
w4 = dyn(
    dpsimpy.dp.ph1.SynchronGenerator4OrderVBR,
    "sg4",
    lambda g: g.set_operational_parameters_per_unit(
        Lq_t=0.65, Td0_t=8.0, Tq0_t=1.0, **base
    ),
)
w6 = dyn(
    dpsimpy.dp.ph1.SynchronGenerator6aOrderVBR,
    "sg6a",
    lambda g: g.set_operational_parameters_per_unit(
        Lq_t=0.65,
        Td0_t=8.0,
        Tq0_t=1.0,
        Ld_s=0.23,
        Lq_s=0.25,
        Td0_s=0.03,
        Tq0_s=0.07,
        Taa=0.002,
        **base,
    ),
)

import pickle


def stats(w, label):
    v = [x.real for x in w.values]
    pk = max(v)
    i = v.index(pk)
    # settle: last time |w-1| > 1e-4
    st = max((w.time[k] for k in range(len(v)) if abs(v[k] - 1) > 1e-4), default=0)
    print(
        "%-5s peak=%.5f at t=%.2fs   |w-1|<1e-4 after %.2fs"
        % (label, pk, w.time[i], st)
    )
    return v


v3 = stats(w3, "3rd")
v4 = stats(w4, "4th")
v6 = stats(w6, "6a")
pickle.dump(
    {"t": list(w4.time), "3rd": v3, "4th": v4, "6a": v6}, open("orders.pkl", "wb")
)
