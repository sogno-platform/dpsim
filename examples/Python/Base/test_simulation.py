import dpsim
from dpsim.Event import Event
import logging

from dpsim.Event import Event

def test_simulation():
    logging.getLogger().setLevel(logging.DEBUG)
    logging.info('hello\n')

    n1 = dpsim.dp.Node('n1')
    gnd = dpsim.dp.Node.GND()
    r = dpsim.dp.ph1.Resistor('r1', [gnd, n1])

    sys = dpsim.SystemTopology(50, [n1], [r])

    sim = dpsim.Simulation(__name__, sys, duration=10, pbar=True)

    sim.start()

    assert sim.wait_until() == Event.starting
    assert sim.wait_until() == Event.running

    sim.pause()

    assert sim.wait_until() == Event.pausing
    assert sim.wait_until() == Event.paused

    steps_start = sim.steps

    while sim.steps < steps_start + 100:
        steps_before = sim.steps

        sim.step()

        assert sim.wait_until() == Event.resuming
        assert sim.wait_until() == Event.running
        assert sim.wait_until() == Event.paused
        assert steps_before + 1 == sim.steps

    sim.start()

    assert sim.wait_until() == Event.resuming
    assert sim.wait_until() == Event.running

    sim.stop()

    assert sim.wait_until() == Event.stopping
    assert sim.wait_until() == Event.stopped

if __name__ == '__main__':
    test_simulation()
