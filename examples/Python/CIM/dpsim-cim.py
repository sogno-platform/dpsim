import sys
import dpsim

def run_cim():
    if len(sys.argv) <= 1:
        print("usage: %s CIM_FILES" % sys.argv[0])
        sys.exit(-1)

    files = sys.argv[1:]

    name = "python"

    system = dpsim.load_cim(name, files, frequency=60)

    sw = dpsim.dp.ph1.Switch("DP_SynGen_TrStab_Step_StepLoad")
    sw.resistance_open = 1e9
    sw.resistance_closed = 0.1
    sw.closed = False
    sw.connect([ dpsim.dp.Node.GND(), system.nodes["BUS9"] ])

    system.add_component(sw)

    sim = dpsim.Simulation(name, system, timestep=0.0001, duration=0.1, pbar=True)
    sim.add_switch_event(sw, 0.05, True)
    sim.run()

if __name__ == "__main__":
    run_cim()
