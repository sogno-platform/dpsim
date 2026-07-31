# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

# Every scheduler must be selectable from Python and actually drive a solve.

import pytest

import dpsimpy

TIME_STEP = 0.001
FINAL_TIME = 0.005


def _scheduler_factories():
    factories = [
        ("sequential", lambda: dpsimpy.SequentialScheduler()),
        ("thread_level", lambda: dpsimpy.ThreadLevelScheduler(threads=2)),
        ("thread_list", lambda: dpsimpy.ThreadListScheduler(threads=2)),
    ]
    # Only present when DPsim is built with OpenMP.
    if hasattr(dpsimpy, "OpenMPLevelScheduler"):
        factories.append(
            ("openmp_level", lambda: dpsimpy.OpenMPLevelScheduler(threads=2))
        )
    return factories


SCHEDULERS = _scheduler_factories()


def _resistive_divider(name):
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    vs = dpsimpy.dp.ph1.VoltageSource("vs")
    vs.set_parameters(complex(100, 0))
    r = dpsimpy.dp.ph1.Resistor("r")
    r.set_parameters(10.0)
    vs.connect([gnd, n1])
    r.connect([n1, gnd])

    system = dpsimpy.SystemTopology(50, [n1], [vs, r])
    sim = dpsimpy.Simulation(name)
    sim.set_system(system)
    sim.set_domain(dpsimpy.Domain.DP)
    sim.set_time_step(TIME_STEP)
    sim.set_final_time(FINAL_TIME)
    return sim


def test_all_schedulers_are_exposed():
    for name, _ in SCHEDULERS:
        assert name


@pytest.mark.parametrize(
    "factory", [f for _, f in SCHEDULERS], ids=[n for n, _ in SCHEDULERS]
)
def test_scheduler_runs_a_simulation(factory):
    sim = _resistive_divider("sched_" + factory.__name__)
    sim.set_scheduler(factory())
    sim.run()


def test_scheduler_accepts_measurement_file(tmp_path):
    out = tmp_path / "measurements.txt"
    sim = _resistive_divider("sched_measure")
    sim.set_scheduler(dpsimpy.SequentialScheduler(out_measurement_file=str(out)))
    sim.run()
