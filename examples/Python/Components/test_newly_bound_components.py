# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

# Exercises each component whose binding was previously missing: construct it,
# push parameters through every exposed setter, and where the component can
# stand alone in a circuit, solve with it in the loop.

import numpy as np
import pytest

import dpsimpy

TIME_STEP = 0.001
FINAL_TIME = 0.005
OMEGA_NOMINAL = 2 * np.pi * 50


def m3(value):
    return np.eye(3) * value


def _run(name, domain, nodes, components):
    system = dpsimpy.SystemTopology(50, nodes, components)
    sim = dpsimpy.Simulation(name)
    sim.set_system(system)
    sim.set_domain(domain)
    sim.set_time_step(TIME_STEP)
    sim.set_final_time(FINAL_TIME)
    sim.run()


# #### Lines ####


def test_dp_ph1_rxline_runs():
    gnd = dpsimpy.dp.SimNode.gnd
    n1, n2 = dpsimpy.dp.SimNode("n1"), dpsimpy.dp.SimNode("n2")
    vs = dpsimpy.dp.ph1.VoltageSource("vs")
    vs.set_parameters(complex(100, 0))
    line = dpsimpy.dp.ph1.RxLine("line")
    line.set_parameters(series_resistance=1.0, series_inductance=0.001)
    load = dpsimpy.dp.ph1.Resistor("r")
    load.set_parameters(10.0)
    vs.connect([gnd, n1])
    line.connect([n1, n2])
    load.connect([n2, gnd])
    _run("dp_ph1_rxline", dpsimpy.Domain.DP, [n1, n2], [vs, line, load])


def test_emt_ph1_piline_runs():
    gnd = dpsimpy.emt.SimNode.gnd
    n1, n2 = dpsimpy.emt.SimNode("n1"), dpsimpy.emt.SimNode("n2")
    vs = dpsimpy.emt.ph1.VoltageSource("vs")
    vs.set_parameters(complex(100, 0), 50)
    line = dpsimpy.emt.ph1.PiLine("line")
    line.set_parameters(
        series_resistance=1.0,
        series_inductance=0.001,
        parallel_capacitance=1e-6,
        parallel_conductance=1e-6,
    )
    load = dpsimpy.emt.ph1.Resistor("r")
    load.set_parameters(10.0)
    vs.connect([gnd, n1])
    line.connect([n1, n2])
    load.connect([n2, gnd])
    _run("emt_ph1_piline", dpsimpy.Domain.EMT, [n1, n2], [vs, line, load])


def test_emt_ph3_rxline_runs():
    gnd = dpsimpy.emt.SimNode.gnd
    n1 = dpsimpy.emt.SimNode("n1", dpsimpy.PhaseType.ABC)
    n2 = dpsimpy.emt.SimNode("n2", dpsimpy.PhaseType.ABC)
    vs = dpsimpy.emt.ph3.VoltageSource("vs")
    vs.set_parameters(
        dpsimpy.Math.single_phase_variable_to_three_phase(complex(100, 0)), 50
    )
    line = dpsimpy.emt.ph3.RxLine("line")
    line.set_parameters(series_resistance=m3(1.0), series_inductance=m3(0.001))
    load = dpsimpy.emt.ph3.Resistor("r")
    load.set_parameters(m3(10.0))
    vs.connect([gnd, n1])
    line.connect([n1, n2])
    load.connect([n2, gnd])
    _run("emt_ph3_rxline", dpsimpy.Domain.EMT, [n1, n2], [vs, line, load])


def test_sp_ph1_rxline_runs():
    gnd = dpsimpy.sp.SimNode.gnd
    n1, n2 = dpsimpy.sp.SimNode("n1"), dpsimpy.sp.SimNode("n2")
    vs = dpsimpy.sp.ph1.VoltageSource("vs")
    vs.set_parameters(complex(100, 0))
    line = dpsimpy.sp.ph1.RXLine("line")
    line.set_parameters(series_resistance=1.0, series_inductance=0.001)
    load = dpsimpy.sp.ph1.Resistor("r")
    load.set_parameters(10.0)
    vs.connect([gnd, n1])
    line.connect([n1, n2])
    load.connect([n2, gnd])
    _run("sp_ph1_rxline", dpsimpy.Domain.SP, [n1, n2], [vs, line, load])


def test_sp_ph1_rxline_power_flow_constructor():
    line = dpsimpy.sp.ph1.RXLine("uid", "line", 110e3, 1.0, 0.001)
    assert line.get_base_voltage() == pytest.approx(110e3)
    line.set_per_unit_system(base_apparent_power=1e6, base_omega=OMEGA_NOMINAL)


# #### Controlled sources ####


def test_dp_ph1_controlled_voltage_source_runs():
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    src = dpsimpy.dp.ph1.ControlledVoltageSource("cvs")
    src.set_parameters(complex(100, 0))
    load = dpsimpy.dp.ph1.Resistor("r")
    load.set_parameters(10.0)
    src.connect([gnd, n1])
    load.connect([n1, gnd])
    _run("dp_ph1_cvs", dpsimpy.Domain.DP, [n1], [src, load])


def test_emt_ph3_controlled_current_source_runs():
    gnd = dpsimpy.emt.SimNode.gnd
    n1 = dpsimpy.emt.SimNode("n1", dpsimpy.PhaseType.ABC)
    src = dpsimpy.emt.ph3.ControlledCurrentSource("ccs")
    src.set_parameters(m3(10.0))
    load = dpsimpy.emt.ph3.Resistor("r")
    load.set_parameters(m3(10.0))
    src.connect([gnd, n1])
    load.connect([n1, gnd])
    _run("emt_ph3_ccs", dpsimpy.Domain.EMT, [n1], [src, load])


def test_sp_ph1_controlled_sources_take_parameters():
    current = dpsimpy.sp.ph1.ControlledCurrentSource("ccs")
    current.set_parameters(complex(1, 0))
    # The value constructor exists only for the current source; the voltage
    # source declares the same overload in C++ but never defines it.
    dpsimpy.sp.ph1.ControlledCurrentSource("ccs2", complex(2, 0))

    voltage = dpsimpy.sp.ph1.ControlledVoltageSource("cvs")
    voltage.set_parameters(complex(10, 0))


def test_dp_ph1_controlled_current_source_takes_parameters():
    src = dpsimpy.dp.ph1.ControlledCurrentSource("ccs")
    src.set_parameters(complex(1, 0))


# #### Ramp and Norton sources ####


def test_dp_ph1_voltage_source_ramp_runs():
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    src = dpsimpy.dp.ph1.VoltageSourceRamp("vsr")
    src.set_parameters(complex(100, 0), complex(10, 0), 0.0, 0.0, 0.002, 0.001)
    load = dpsimpy.dp.ph1.Resistor("r")
    load.set_parameters(10.0)
    src.connect([gnd, n1])
    load.connect([n1, gnd])
    _run("dp_ph1_vsramp", dpsimpy.Domain.DP, [n1], [src, load])


def test_emt_ph1_voltage_source_ramp_takes_parameters():
    src = dpsimpy.emt.ph1.VoltageSourceRamp("vsr")
    src.set_parameters(complex(100, 0), complex(10, 0), 50.0, 0.0, 0.02, 0.01)


def test_emt_ph1_voltage_source_norton_runs():
    gnd = dpsimpy.emt.SimNode.gnd
    n1 = dpsimpy.emt.SimNode("n1")
    src = dpsimpy.emt.ph1.VoltageSourceNorton("vsn")
    src.set_parameters(complex(100, 0), 50.0, 1.0)
    src.set_voltage_ref(complex(110, 0))
    load = dpsimpy.emt.ph1.Resistor("r")
    load.set_parameters(10.0)
    src.connect([gnd, n1])
    load.connect([n1, gnd])
    _run("emt_ph1_vsn", dpsimpy.Domain.EMT, [n1], [src, load])


def test_emt_ph3_voltage_source_norton_runs():
    gnd = dpsimpy.emt.SimNode.gnd
    n1 = dpsimpy.emt.SimNode("n1", dpsimpy.PhaseType.ABC)
    src = dpsimpy.emt.ph3.VoltageSourceNorton("vsn")
    src.set_parameters(complex(100, 0), 50.0, 1.0)
    src.set_voltage_ref(complex(110, 0))
    load = dpsimpy.emt.ph3.Resistor("r")
    load.set_parameters(m3(10.0))
    src.connect([gnd, n1])
    load.connect([n1, gnd])
    _run("emt_ph3_vsn", dpsimpy.Domain.EMT, [n1], [src, load])


@pytest.mark.skipif(
    not hasattr(dpsimpy.dp.ph1, "ProfileVoltageSource"),
    reason="requires a build with VILLASnode support",
)
def test_dp_ph1_profile_voltage_source_rejects_malformed_file(tmp_path):
    # The source file is a VILLASnode protobuf sample dump, not CSV. A
    # malformed file must surface as a Python exception, not a crash.
    bad = tmp_path / "profile.bin"
    bad.write_bytes(b"\x00" * 32)
    with pytest.raises(RuntimeError):
        dpsimpy.dp.ph1.ProfileVoltageSource("pvs", str(bad))


# #### Loads ####


def test_dp_ph1_pq_load_cs_takes_parameters():
    load = dpsimpy.dp.ph1.PQLoadCS("load")
    load.set_parameters(1e5, 1e4, 20e3)
    dpsimpy.dp.ph1.PQLoadCS("load2", 1e5, 1e4, 20e3)


def test_dp_ph1_rx_load_switch_takes_parameters():
    load = dpsimpy.dp.ph1.RXLoadSwitch("load")
    load.set_parameters(1e5, 1e4, 20e3, 1e6, 1e-3, False)
    load.set_switch_parameters(1e6, 1e-3, True)


# #### Machines and devices ####


def test_synchron_generator_ideal_constructs():
    dpsimpy.dp.ph1.SynchronGeneratorIdeal("sg_dp")
    dpsimpy.emt.ph3.SynchronGeneratorIdeal("sg_emt")


@pytest.mark.parametrize("domain", ["sp", "dp", "emt"])
def test_synchron_generator_tr_stab_parameter_entry_points_match_across_domains(
    domain,
):
    mod = dpsimpy.emt.ph3 if domain == "emt" else getattr(dpsimpy, domain).ph1

    pu = mod.SynchronGeneratorTrStab(f"{domain}_sg_pu")
    pu.set_standard_parameters_PU(555e6, 24e3, 60, 0.3, 3.7)
    pu.set_initial_values(complex(300e6, 0), 300e6)

    si = mod.SynchronGeneratorTrStab(f"{domain}_sg_si")
    si.set_standard_parameters_SI(555e6, 24e3, 60, 2, 0.003, 0.0002, 2.8e4)

    fundamental = mod.SynchronGeneratorTrStab(f"{domain}_sg_fund")
    fundamental.set_fundamental_parameters_PU(555e6, 24e3, 60, 0.15, 1.66, 0.16, 3.7)


def test_dp_ph1_voltage_source_norton_runs():
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    src = dpsimpy.dp.ph1.VoltageSourceNorton("vsn")
    src.set_parameters(complex(100, 0), 50.0, 1.0)
    src.set_voltage_ref(complex(110, 0))
    load = dpsimpy.dp.ph1.Resistor("r")
    load.set_parameters(10.0)
    src.connect([gnd, n1])
    load.connect([n1, gnd])
    _run("dp_ph1_vsn", dpsimpy.Domain.DP, [n1], [src, load])


def test_emt_ph3_synchron_generator_tr_stab_parameter_entry_points():
    pu = dpsimpy.emt.ph3.SynchronGeneratorTrStab("sg_pu")
    pu.set_standard_parameters_PU(555e6, 24e3, 60, 0.3, 3.7)
    pu.set_initial_values(complex(300e6, 0), 300e6)

    si = dpsimpy.emt.ph3.SynchronGeneratorTrStab("sg_si")
    si.set_standard_parameters_SI(555e6, 24e3, 60, 2, 0.003, 0.0002, 2.8e4)

    fundamental = dpsimpy.emt.ph3.SynchronGeneratorTrStab("sg_fund")
    fundamental.set_fundamental_parameters_PU(555e6, 24e3, 60, 0.15, 1.66, 0.16, 3.7)


def test_emt_ph3_synchron_generator_4order_pcm_parameters():
    gen = dpsimpy.emt.ph3.SynchronGenerator4OrderPCM("sg")
    gen.set_operational_parameters_per_unit(
        555e6, 24e3, 60, 3.7, 1.81, 1.76, 0.15, 0.3, 0.65, 8.0, 1.0
    )
    # Inherited from the shared reduced-order generator base.
    gen.set_base_parameters(555e6, 24e3, 60)


def test_emt_ph3_synchron_generator_vbr_parameter_entry_points():
    operational = dpsimpy.emt.ph3.SynchronGeneratorVBR("sg_op")
    operational.set_base_and_operational_per_unit_parameters(
        555e6,
        24e3,
        60,
        2,
        1300,
        0.003,
        1.81,
        1.76,
        0.3,
        0.65,
        0.23,
        0.25,
        0.15,
        8.0,
        1.0,
        0.03,
        0.07,
        3.7,
    )
    operational.set_initial_values(300e6, 0.0, 24e3, 0.0, 300e6)

    fundamental = dpsimpy.emt.ph3.SynchronGeneratorVBR("sg_fund")
    fundamental.set_base_and_fundamental_per_unit_parameters(
        555e6,
        24e3,
        60,
        2,
        1300,
        0.003,
        0.15,
        1.66,
        1.61,
        0.0006,
        0.165,
        0.0284,
        0.1713,
        0.0062,
        0.7252,
        0.0237,
        0.125,
        3.7,
    )


def test_sp_ph1_solid_state_transformer_takes_parameters():
    sst = dpsimpy.sp.ph1.SolidStateTransformer("sst")
    sst.set_parameters(nom_v1=110e3, nom_v2=20e3, p_ref=1e6, q1_ref=0.0, q2_ref=0.0)


def test_sp_ph1_voltage_source_inverter_bus_type():
    inverter = dpsimpy.sp.ph1.VoltageSourceInverter("uid", "vsi")
    inverter.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PQ)
    dpsimpy.sp.ph1.VoltageSourceInverter("uid2", "vsi2", 1e6, 0.0)


# #### SSN components ####


def test_dp_ph1_ssn_variable_serial_rlc_runs():
    gnd = dpsimpy.dp.SimNode.gnd
    n1, n2 = dpsimpy.dp.SimNode("n1"), dpsimpy.dp.SimNode("n2")
    vs = dpsimpy.dp.ph1.VoltageSource("vs")
    vs.set_parameters(complex(100, 0))
    rlc = dpsimpy.dp.ph1.Variable_Serial_RLC("rlc")
    rlc.set_parameters(1.0, 0.001, 1e-6, OMEGA_NOMINAL)
    load = dpsimpy.dp.ph1.Resistor("r")
    load.set_parameters(10.0)
    vs.connect([gnd, n1])
    rlc.connect([n1, n2])
    load.connect([n2, gnd])
    _run("dp_ph1_var_rlc", dpsimpy.Domain.DP, [n1, n2], [vs, rlc, load])


def test_emt_ph3_ssn_inductor_runs():
    gnd = dpsimpy.emt.SimNode.gnd
    n1 = dpsimpy.emt.SimNode("n1", dpsimpy.PhaseType.ABC)
    n2 = dpsimpy.emt.SimNode("n2", dpsimpy.PhaseType.ABC)
    vs = dpsimpy.emt.ph3.VoltageSource("vs")
    vs.set_parameters(
        dpsimpy.Math.single_phase_variable_to_three_phase(complex(100, 0)), 50
    )
    inductor = dpsimpy.emt.ph3.SSN_Inductor("l")
    inductor.set_parameters(m3(0.001))
    load = dpsimpy.emt.ph3.Resistor("r")
    load.set_parameters(m3(10.0))
    vs.connect([gnd, n1])
    inductor.connect([n1, n2])
    load.connect([n2, gnd])
    _run("emt_ph3_ssn_inductor", dpsimpy.Domain.EMT, [n1, n2], [vs, inductor, load])


def test_emt_ph3_ssn_inductor_exposes_attribute():
    inductor = dpsimpy.emt.ph3.SSN_Inductor("l")
    inductor.set_parameters(m3(0.001))
    assert inductor.L is not None
