import dpsim
from dpsim.Event import Event
import logging

from dpsim.Event import Event

def no_test_simulation():
    logging.getLogger().setLevel(logging.DEBUG)
    logging.info("hello\n")

    n1 = dpsim.dp.Node("n1")
    gnd = dpsim.dp.Node.GND()

    r = dpsim.dp.ph1.Resistor("r1", [gnd, n1])

    sys = dpsim.SystemTopology(50, [n1], [r])

    sim = dpsim.Simulation(__name__, sys, duration=10, rt=True, pbar=True, single_stepping=True)

    sim.step()
    assert sim.wait_until() == Event.starting
    assert sim.wait_until() == Event.running
    assert sim.wait_until() == Event.paused

    for x in range(2,10):
        sim.step()
        assert sim.wait_until() == Event.resuming
        assert sim.wait_until() == Event.running
        assert sim.wait_until() == Event.paused
        assert sim.steps == x

    sim.stop()
    assert sim.wait_until() == Event.stopping
    assert sim.wait_until() == Event.stopped


if __name__ == "__main__":
    test_simulation()
