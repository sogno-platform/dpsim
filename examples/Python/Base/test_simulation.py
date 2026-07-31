# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

# Simulation lifecycle: run to completion, and drive the solver step by step.

import dpsimpy

TIME_STEP = 0.001
FINAL_TIME = 0.01
STEPS = int(FINAL_TIME / TIME_STEP)


def rl_circuit(name):
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    vs = dpsimpy.dp.ph1.VoltageSource("vs")
    vs.set_parameters(complex(10, 0))
    r = dpsimpy.dp.ph1.Resistor("r")
    r.set_parameters(1.0)
    vs.connect([gnd, n1])
    r.connect([n1, gnd])

    system = dpsimpy.SystemTopology(50, [n1], [vs, r])
    sim = dpsimpy.Simulation(name)
    sim.set_system(system)
    sim.set_domain(dpsimpy.Domain.DP)
    sim.set_time_step(TIME_STEP)
    sim.set_final_time(FINAL_TIME)
    return sim, n1


def test_simulation_name_round_trips():
    sim, _ = rl_circuit("named_simulation")
    assert sim.name() == "named_simulation"


def test_simulation_runs_to_completion():
    sim, _ = rl_circuit("run_to_completion")
    sim.run()


def test_simulation_can_be_stepped():
    sim, _ = rl_circuit("stepped")
    sim.start()
    for _ in range(STEPS):
        sim.next()
    sim.stop()


def test_stepping_and_running_agree_on_node_voltage():
    # A purely resistive circuit settles immediately, so both drive paths must
    # land on the same node voltage.
    ran, ran_node = rl_circuit("compare_run")
    ran.run()
    ran_voltage = ran_node.single_voltage()

    stepped, stepped_node = rl_circuit("compare_step")
    stepped.start()
    for _ in range(STEPS):
        stepped.next()
    stepped.stop()
    stepped_voltage = stepped_node.single_voltage()

    assert abs(ran_voltage - stepped_voltage) < 1e-9
