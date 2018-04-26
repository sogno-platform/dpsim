import dpsim as dps
import dpsim.components.dp as dp
import pytest
import asyncio
import time

def my_callback(sim, evt):
    if evt == 1:
        print("Simulation started")

    if evt == 3:
        print("Completed. Goodbye")
        sim.wait()

        sim.loop.stop()

def test_async():
    el = asyncio.get_event_loop()

    # Nodes
    gnd = dps.Node.GND()
    n1  = dps.Node("n1")

    # Components
    v1 = dp.VoltageSource("v_1", [gnd, n1], 10)
    r1 = dp.Resistor("r_1", [n1, gnd], 1)

    system = dps.SystemTopology(50, [gnd, n1], [v1, r1])

    sim = dps.Simulation('async', system, duration=200, timestep=0.0005)

    # Start in two seconds!
    sim.start(when = time.time() + 2)
    sim.register_callback(my_callback)
    sim.show_progressbar()

    el.run_forever()

if __name__ == "__main__":
    test_async()
