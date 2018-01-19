import sys

def test_shmem1():
    import dpsim
    import dpsim.components.dp as dp

    evs = dp.VoltageSource("v_ext", 2, 0, 0+0j, 1)

    comps = [
      dp.VoltageSourceNorton("v_s", 1, 0, 10000+0j, 1),
      dp.Inductor("l_1", 1, 2, 1e-3),
      evs
    ]

    intf = dpsim.open_shmem_interface("/dpsim12", "/dpsim21", samplelen = 2)
    intf.register_source(evs, 0, 1)
    intf.export_current(evs, 0, 1)

    sim = dpsim.Simulation("shmem1", duration = 1)
    sim.add_interface(intf)
    sim.run()

def test_shmem2:
    ecs = dp.CurrentSource("i_ext", 1, 0, 0+0j)

    comps = [
      dp.Resistor("r_1", 1, 0, 1),
      ecs
    ]

    intf = dpsim.open_shmem_interface("/dpsim21", "/dpsim12", samplelen = 2)
    intf.register_source(ecs, 0, 1)
    intf.export_voltage(1, 0, 0, 1)

    sim = dpsim.Simulation("shmem2", comps, duration = 1)
    sim.add_interface(intf)
    sim.run()

if __name__ == "__main__":
    if sys.argv[1] == "1":
        test_shmem1()
    else:
        test_shmem2()
