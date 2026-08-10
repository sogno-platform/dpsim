# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

# RealTimeSimulation paces a solve against the wall clock.

import time

import pytest

import dpsimpy

TIME_STEP = 0.001
FINAL_TIME = 0.05


def rl_circuit(name):
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    vs = dpsimpy.dp.ph1.VoltageSource("v_1")
    vs.set_parameters(complex(10, 0))
    r = dpsimpy.dp.ph1.Resistor("r_1")
    r.set_parameters(1.0)
    vs.connect([gnd, n1])
    r.connect([n1, gnd])

    system = dpsimpy.SystemTopology(50, [n1], [vs, r])
    sim = dpsimpy.RealTimeSimulation(name)
    sim.set_system(system)
    sim.set_domain(dpsimpy.Domain.DP)
    sim.set_time_step(TIME_STEP)
    sim.set_final_time(FINAL_TIME)
    return sim


@pytest.mark.skipif(
    not hasattr(dpsimpy, "RealTimeSimulation"),
    reason="requires a build with real-time support",
)
def test_realtime_simulation_runs():
    sim = rl_circuit("realtime")
    sim.run(1)


@pytest.mark.skipif(
    not hasattr(dpsimpy, "RealTimeSimulation"),
    reason="requires a build with real-time support",
)
def test_realtime_simulation_is_paced_by_the_wall_clock():
    # Real-time pacing means the solve tracks the simulated time span, unlike
    # the equivalent offline Simulation which finishes in well under a
    # millisecond. One time step of slack covers the final step boundary.
    sim = rl_circuit("realtime_paced")
    started = time.monotonic()
    sim.run(0)
    elapsed = time.monotonic() - started
    assert elapsed >= FINAL_TIME - TIME_STEP
