import pytest
import dpsimpy


def build_system(resistance):
    gnd = dpsimpy.emt.SimNode.gnd
    n1 = dpsimpy.emt.SimNode("n1", dpsimpy.PhaseType.ABC)
    n2 = dpsimpy.emt.SimNode("n2", dpsimpy.PhaseType.ABC)

    vs = dpsimpy.emt.ph3.VoltageSource("vs")
    vs.set_parameters(
        dpsimpy.Math.single_phase_variable_to_three_phase(complex(1000, 0)), 50
    )
    vs.connect([gnd, n1])

    line = dpsimpy.emt.ph3.PiLine("line")
    line.set_parameters(
        dpsimpy.Math.single_phase_parameter_to_three_phase(resistance),
        dpsimpy.Math.single_phase_parameter_to_three_phase(0.001),
        dpsimpy.Math.single_phase_parameter_to_three_phase(1e-6),
        dpsimpy.Math.single_phase_parameter_to_three_phase(1e-6),
    )
    line.connect([n1, n2])

    load = dpsimpy.emt.ph3.Resistor("load")
    load.set_parameters(dpsimpy.Math.single_phase_parameter_to_three_phase(100))
    load.connect([n2, gnd])

    return dpsimpy.SystemTopology(50, [gnd, n1, n2], [vs, line, load])


def test_zero_series_resistance_is_rejected():
    sim = dpsimpy.Simulation("piline_zero_resistance")
    sim.set_system(build_system(0.0))
    sim.set_domain(dpsimpy.Domain.EMT)
    sim.set_time_step(1e-4)
    sim.set_final_time(1e-3)

    with pytest.raises(ValueError, match="R_series"):
        sim.run()


def test_valid_series_resistance_is_accepted():
    sim = dpsimpy.Simulation("piline_valid_resistance")
    sim.set_system(build_system(1.0))
    sim.set_domain(dpsimpy.Domain.EMT)
    sim.set_time_step(1e-4)
    sim.set_final_time(1e-3)

    sim.run()
