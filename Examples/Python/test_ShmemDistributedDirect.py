import sys
import threading
import dpsim

class Left(threading.Thread):
    def run(self):
        # Nodes
        gnd = dpsim.dp.Node.GND()
        n1  = dpsim.dp.Node("n1")
        n2  = dpsim.dp.Node("n2")

        vs =  dpsim.dp.ph1.VoltageSourceNorton("v_s", [n1, gnd], 10000+0j, 1)
        evs = dpsim.dp.ph1.VoltageSource("v_ext", [n2, gnd], 0+0j)
        l1 =  dpsim.dp.ph1.Inductor("l_1", [n1, n2], 1e-3)

        intf = dpsim.open_interface("/dpsim12", "/dpsim21", samplelen = 2)
        intf.import_attribute(evs, "v_ref", 1, 0, 1)
        intf.export_attribute(evs, "comp_current", 1, 0, 1)

        sys = dpsim.SystemTopology(50, [gnd, n1, n2], [evs, vs, l1])

        sim = dpsim.Simulation("shmem1", sys, duration = 1)
        sim.add_interface(intf)

        print("Starting simulation on left side")
        sim.run()

class Right(threading.Thread):
    def run(self):
        # Nodes
        gnd = dpsim.dp.Node.GND()
        n3  = dpsim.dp.Node("n3")

        # Components
        ecs = dpsim.dp.ph1.CurrentSource("i_ext", [n3, gnd], 0+0j)
        r1 = dpsim.dp.ph1.Resistor("r_1", [n3, gnd], 1)

        intf = dpsim.open_interface("/dpsim21", "/dpsim12", samplelen = 2)
        intf.import_attribute(ecs, "i_ref", 1, 0, 1)
        intf.export_attribute(r1, "comp_voltage", 1, 0, 1)

        sys = dpsim.SystemTopology(50, [gnd, n3], [ecs, r1])

        sim = dpsim.Simulation("shmem2", sys, duration = 1)
        sim.add_interface(intf)

        print("Starting simulation on right side")
        sim.run()

def test_ShmemDistributedDirect():
    left_thread = Left()
    right_thread = Right()

    left_thread.start()
    right_thread.start()

    left_thread.join();
    right_thread.join()
