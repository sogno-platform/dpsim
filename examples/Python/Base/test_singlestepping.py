# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

# Driving the solver one step at a time and observing state between steps.

import dpsimpy

TIME_STEP = 0.001
FINAL_TIME = 0.01
STEPS = int(FINAL_TIME / TIME_STEP)


def rc_circuit(name):
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    vs = dpsimpy.dp.ph1.VoltageSource("vs")
    vs.set_parameters(complex(10, 0))
    r = dpsimpy.dp.ph1.Resistor("r")
    r.set_parameters(1.0)
    c = dpsimpy.dp.ph1.Capacitor("c")
    c.set_parameters(1e-3)
    vs.connect([gnd, n1])
    r.connect([n1, gnd])
    c.connect([n1, gnd])

    system = dpsimpy.SystemTopology(50, [n1], [vs, r, c])
    sim = dpsimpy.Simulation(name)
    sim.set_system(system)
    sim.set_domain(dpsimpy.Domain.DP)
    sim.set_time_step(TIME_STEP)
    sim.set_final_time(FINAL_TIME)
    return sim, n1


def test_each_step_advances_the_solver():
    sim, node = rc_circuit("single_stepping")
    sim.start()
    voltages = []
    for _ in range(STEPS):
        sim.next()
        voltages.append(node.single_voltage())
    sim.stop()

    assert len(voltages) == STEPS
    # The source is constant, so every step must produce a finite solution.
    assert all(abs(v) < 1e6 for v in voltages)


def test_stepping_past_the_final_time_is_safe():
    sim, _ = rc_circuit("overrun")
    sim.start()
    for _ in range(STEPS * 2):
        sim.next()
    sim.stop()


def test_attributes_are_readable_between_steps():
    sim, _ = rc_circuit("stepwise_attributes")
    sim.start()
    sim.next()
    value = sim.get_idobj_attr("c", "C").get()
    sim.stop()
    assert value == 1e-3
