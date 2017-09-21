#!/usr/bin/python3

import dpsim

comps = [dpsim.VoltSourceRes("v_s", 1, 0, 10000+0j, 1),
    dpsim.Inductor("l_1", 1, 2, 1e-3)]
evs = dpsim.ExternalVoltageSource("v_ext", 2, 0, 0+0j, 1)
comps.append(evs)
intf = dpsim.ShmemInterface("/dpsim12", "/dpsim21", samplelen=2)
intf.register_source(evs, 0, 1)
intf.export_current(evs, 0, 1)
sim = dpsim.Simulation(comps, duration=1, llog="lvector-shmem1.csv")
sim.add_interface(intf)
sim.start()
sim.wait()
