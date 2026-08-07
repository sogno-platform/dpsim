# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

# A series RL circuit feeding a resistive load, checked against the analytical
# steady-state solution rather than a stored reference file.

import cmath

import pytest

import dpsimpy

TIME_STEP = 0.0005
FINAL_TIME = 0.2

SOURCE_VOLTAGE = complex(10, 0)
R_LINE = 0.1
L_LINE = 0.001
R_LOAD = 20.0
FREQUENCY = 50


def build(name):
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    n2 = dpsimpy.dp.SimNode("n2")
    n3 = dpsimpy.dp.SimNode("n3")

    v1 = dpsimpy.dp.ph1.VoltageSource("v_1")
    v1.set_parameters(SOURCE_VOLTAGE)
    r_line = dpsimpy.dp.ph1.Resistor("r_L")
    r_line.set_parameters(R_LINE)
    l_line = dpsimpy.dp.ph1.Inductor("l_L")
    l_line.set_parameters(L_LINE)
    r_load = dpsimpy.dp.ph1.Resistor("r_1")
    r_load.set_parameters(R_LOAD)

    v1.connect([gnd, n1])
    r_line.connect([n1, n2])
    l_line.connect([n2, n3])
    r_load.connect([n3, gnd])

    system = dpsimpy.SystemTopology(
        FREQUENCY, [n1, n2, n3], [v1, r_line, l_line, r_load]
    )
    sim = dpsimpy.Simulation(name)
    sim.set_system(system)
    sim.set_domain(dpsimpy.Domain.DP)
    sim.set_time_step(TIME_STEP)
    sim.set_final_time(FINAL_TIME)
    return sim, (n1, n2, n3)


def analytical_load_voltage():
    omega = 2 * cmath.pi * FREQUENCY
    impedance = R_LINE + 1j * omega * L_LINE + R_LOAD
    current = SOURCE_VOLTAGE / impedance
    return current * R_LOAD


def test_source_node_holds_the_source_voltage():
    sim, (n1, _, _) = build("circuit_source_node")
    sim.run()
    assert abs(n1.single_voltage() - SOURCE_VOLTAGE) < 1e-6


def test_load_voltage_matches_the_analytical_solution():
    sim, (_, _, n3) = build("circuit_load_voltage")
    sim.run()
    assert n3.single_voltage() == pytest.approx(analytical_load_voltage(), abs=1e-6)


def test_logged_attributes_are_written(tmp_path):
    dpsimpy.Logger.set_log_dir(str(tmp_path))
    sim, (n1, _, n3) = build("circuit_logging")
    logger = dpsimpy.Logger("circuit_logging")
    logger.log_attribute("v1", n1.attr("v"))
    logger.log_attribute("v3", n3.attr("v"))
    sim.add_logger(logger)
    sim.run()

    written = list(tmp_path.rglob("*.csv"))
    assert written, f"no log files written under {tmp_path}"
