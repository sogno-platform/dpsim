#!/usr/bin/env python3

import dpsim

ecs = dpsim.ExternalCurrentSource("i_ext", 1, 0, 0+0j)

comps = [
  dpsim.LinearResistor("r_1", 1, 0, 1),
  ecs
]

intf = dpsim.open_shmem_interface("/dpsim21", "/dpsim12", samplelen=2)
intf.register_source(ecs, 0, 1)
intf.export_voltage(1, 0, 0, 1)

sim = dpsim.Simulation(comps, duration=1, llog="lvector-shmem2.csv")
sim.add_interface(intf)
sim.start()
sim.wait()
