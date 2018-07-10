import sys
import threading
import dpsim as dps
import dpsim.components.dp as dp

class Left(threading.Thread):
    def run(self):
        # Nodes
        gnd = dps.Node.GND()
        n1  = dps.Node("n1")
        n2  = dps.Node("n2")

        vs = dp.VoltageSourceNorton("v_s", [n1, gnd], 10000+0j, 1)
        evs = dp.VoltageSource("v_ext", [n2, gnd], 0+0j)
        l1 = dp.Inductor("l_1", [n1, n2], 1e-3)

        intf = dps.open_interface("/dpsim12", "/dpsim21", samplelen = 2)
        intf.import_attribute(evs, "voltage_ref", 1, 0, 1)
        intf.export_attribute(evs, "comp_current", 1, 0, 1)

        sys = dps.SystemTopology(50, [gnd, n1, n2], [evs, vs, l1])

        sim = dps.Simulation("shmem1", sys, duration = 1)
        sim.add_interface(intf)

        print("Starting simulation on left side")
        sim.run()

class Right(threading.Thread):
    def run(self):
        # Nodes
        gnd = dps.Node.GND()
        n3  = dps.Node("n3")

        # Components
        ecs = dp.CurrentSource("i_ext", [n3, gnd], 0+0j)
        r1 = dp.Resistor("r_1", [n3, gnd], 1)

        intf = dps.open_interface("/dpsim21", "/dpsim12", samplelen = 2)
        intf.import_attribute(ecs, "current_ref", 1, 0, 1)
        intf.export_attribute(r1, "comp_voltage", 1, 0, 1)

        sys = dps.SystemTopology(50, [gnd, n3], [ecs, r1])

        sim = dps.Simulation("shmem2", sys, duration = 1)
        sim.add_interface(intf)

        print("Starting simulation on right side")
        sim.run()

def no_test_ShmemDistributedDirect():
    left_thread = Left()
    right_thread = Right()

    left_thread.start()
    right_thread.start()

    left_thread.join();
    right_thread.join()

if __name__ == "__main__":
    if len(sys.argv) == 2:
        if sys.argv[1] == "left":
            left = Left()
            left.run()
        elif sys.argv[1] == "right":
            right = Right()
            right.run()
    else:
        test_ShmemDistributedDirect()
