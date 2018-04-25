import time
import dpsim
import dpsim.components.dp as dp
import pytest

def test_async():
    duration = 1

    sim = dpsim.Simulation(rt = 1, duration = duration)

    start = time.perf_counter()
    sim.run()
    end = time.perf_counter()

    assert abs(end - start - duration) < 1e-2


