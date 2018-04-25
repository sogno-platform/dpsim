import dpsim
import dpsim.components.dp as dp
import pytest

def state_changed(newstate):
    print("New state is: %s", newstate)

def test_async():
    sim = dpsim.Simulation()

    sim.register_callback(state_changed)

    sim.run()
