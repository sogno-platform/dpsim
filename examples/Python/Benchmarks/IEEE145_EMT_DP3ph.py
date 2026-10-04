#!/usr/bin/env python3
"""Paired IEEE-145 three-phase EMT / dynamic-phasor solver benchmark.

The static MATPOWER case is used as the common network definition. Generators
are represented by balanced ideal voltage sources at their solved MATPOWER bus
phasors; loads are constant-impedance equivalents at nominal bus voltage. This
keeps the EMT and DP topologies and element values identical while avoiding
invented machine dynamics (the IEEE-145 MATPOWER case contains no dynamic data).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "build"))
sys.path.insert(0, str(ROOT / "python" / "src" / "dpsim"))

import dpsimpy  # noqa: E402
import matpower  # noqa: E402

CASE_URL = (
    "https://raw.githubusercontent.com/dpsim-simulator/reference-results/"
    "master/Matpower/case145results.mat"
)
RMS3PH_TO_PEAK1PH = math.sqrt(2.0 / 3.0)


def load_case(path: Path, frequency: float):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        print(f"Fetching validated MATPOWER IEEE-145 data to {path}")
        urllib.request.urlretrieve(CASE_URL, path)
    reader = matpower.Reader(str(path), mpc_name="case145results")
    reader.process_mpc(frequency=frequency)
    return reader


def diagonal(value: float) -> np.ndarray:
    return np.eye(3, dtype=np.float64) * value


def phase_phasors(v_ll_rms: float, angle_deg: float) -> np.ndarray:
    va = v_ll_rms * np.exp(1j * math.radians(angle_deg))
    shift_b = np.exp(-1j * 2.0 * math.pi / 3.0)
    shift_c = np.exp(1j * 2.0 * math.pi / 3.0)
    return np.array([va, va * shift_b, va * shift_c], dtype=np.complex128)


def add_shunt(
    components, module, ground, g: float, b: float, omega: float, node, prefix: str
):
    if abs(g) > 1e-15:
        resistor = module.Resistor(f"{prefix}_G")
        resistor.set_parameters(diagonal(1.0 / g))
        resistor.connect([ground, node])
        components.append(resistor)

    if b > 1e-15:
        capacitor = module.Capacitor(f"{prefix}_B")
        capacitor.set_parameters(diagonal(b / omega))
        capacitor.connect([ground, node])
        components.append(capacitor)
    elif b < -1e-15:
        inductor = module.Inductor(f"{prefix}_B")
        inductor.set_parameters(diagonal(-1.0 / (omega * b)))
        inductor.connect([ground, node])
        components.append(inductor)


def add_load(components, domain_module, ground, bus_row, node, frequency: float):
    p_total = float(bus_row.Pd) * 1e6
    q_total = float(bus_row.Qd) * 1e6
    if abs(p_total) < 1e-9 and abs(q_total) < 1e-9:
        return

    v_nom_ll = float(bus_row.baseKV) * 1e3
    p_phase = p_total / 3.0
    q_phase = q_total / 3.0
    if domain_module is dpsimpy.emt.ph3:
        load = domain_module.RXLoad(f"Load_N{int(bus_row.bus_i)}")
        load.set_parameters(diagonal(p_phase), diagonal(q_phase), v_nom_ll)
        load.connect([node])
        components.append(load)
        return

    # DP::Ph3 has no RXLoad composite. Stamp the same per-phase parallel R/L/C
    # equivalent used internally by EMT::Ph3::RXLoad.
    denom = p_phase * p_phase + q_phase * q_phase
    v_phase_rms_sq = (v_nom_ll / math.sqrt(3.0)) ** 2
    if abs(p_phase) > 1e-12:
        resistance = v_phase_rms_sq * p_phase / denom
        resistor = domain_module.Resistor(f"Load_N{int(bus_row.bus_i)}_R")
        resistor.set_parameters(diagonal(resistance))
        resistor.connect([ground, node])
        components.append(resistor)

    if abs(q_phase) > 1e-12:
        reactance = v_phase_rms_sq * q_phase / denom
        if reactance > 0:
            inductor = domain_module.Inductor(f"Load_N{int(bus_row.bus_i)}_L")
            inductor.set_parameters(diagonal(reactance / (2.0 * math.pi * frequency)))
            inductor.connect([ground, node])
            components.append(inductor)
        else:
            capacitor = domain_module.Capacitor(f"Load_N{int(bus_row.bus_i)}_C")
            capacitor.set_parameters(
                diagonal(-1.0 / (2.0 * math.pi * frequency * reactance))
            )
            capacitor.connect([ground, node])
            components.append(capacitor)


def build_system(reader, domain: str, frequency: float):
    is_emt = domain == "EMT"
    module = dpsimpy.emt.ph3 if is_emt else dpsimpy.dp.ph3
    node_module = dpsimpy.emt if is_emt else dpsimpy.dp
    phase_type = dpsimpy.PhaseType.ABC
    omega = 2.0 * math.pi * frequency

    buses = reader.mpc_bus_data
    bus_by_id = {int(row.bus_i): row for row in buses.itertuples(index=False)}
    nodes = {}
    for bus_id, row in bus_by_id.items():
        node = node_module.SimNode(f"N{bus_id}", phase_type)
        v_ll = float(row.baseKV) * 1e3 * float(row.Vm)
        node.set_initial_voltage(phase_phasors(v_ll, float(row.Va)))
        nodes[bus_id] = node

    components = []

    # Each PV/slack generator bus is held at its solved MATPOWER voltage. This
    # creates the same ideal-source network in EMT and DP without fabricated
    # synchronous-machine parameters.
    generator_buses = set()
    for gen in reader.mpc_gen_data.itertuples(index=False):
        if int(gen.status) != 1:
            continue
        bus_id = int(gen.bus)
        if bus_id in generator_buses or bus_id not in nodes:
            continue
        generator_buses.add(bus_id)
        bus = bus_by_id[bus_id]
        v_ll_rms = float(bus.baseKV) * 1e3 * float(bus.Vm)
        # EMT::Ph3::VoltageSource converts its line-to-line RMS reference to
        # phase peak internally. DP::Ph3::VoltageSource expects phase-peak
        # phasors directly, so its reference needs the conversion here.
        source_voltage = v_ll_rms * (RMS3PH_TO_PEAK1PH if not is_emt else 1.0)
        voltage_ref = phase_phasors(source_voltage, float(bus.Va))
        source = module.NetworkInjection(f"Source_N{bus_id}")
        source.set_parameters(voltage_ref, frequency)
        source.connect([nodes[bus_id]])
        components.append(source)

    for bus_id, bus in bus_by_id.items():
        add_load(
            components, module, node_module.SimNode.gnd, bus, nodes[bus_id], frequency
        )
        gs = float(bus.Gs) * 1e6 / (float(bus.baseKV) * 1e3) ** 2
        bs = float(bus.Bs) * 1e6 / (float(bus.baseKV) * 1e3) ** 2
        if abs(gs) > 1e-15 or abs(bs) > 1e-15:
            if is_emt:
                shunt = module.Shunt(f"Shunt_N{bus_id}")
                shunt.set_parameters(gs, bs)
                shunt.connect([nodes[bus_id]])
                components.append(shunt)
            else:
                add_shunt(
                    components,
                    module,
                    node_module.SimNode.gnd,
                    gs,
                    bs,
                    omega,
                    nodes[bus_id],
                    f"Shunt_N{bus_id}",
                )

    for idx, branch in enumerate(reader.mpc_branch_data.itertuples(index=False), 1):
        if int(branch.status) != 1:
            continue
        fbus = int(branch.fbus)
        tbus = int(branch.tbus)
        fbase = float(bus_by_id[fbus].baseKV) * 1e3
        tbase = float(bus_by_id[tbus].baseKV) * 1e3
        tap = float(branch.ratio) if float(branch.ratio) != 0.0 else 1.0

        if float(branch.ratio) == 0.0 and math.isclose(
            fbase, tbase, rel_tol=0.0, abs_tol=1e-6
        ):
            z_base = tbase * tbase / reader.mpc_base_power_MVA
            resistance = float(branch.r) * z_base
            inductance = float(branch.x) * z_base / omega
            capacitance = float(branch.b) / z_base / omega
            line = module.PiLine(f"Line_{idx}_{fbus}_{tbus}")
            line.set_parameters(
                diagonal(resistance),
                diagonal(inductance),
                diagonal(capacitance),
                diagonal(0.0),
            )
            line.connect([nodes[tbus], nodes[fbus]])
            components.append(line)
        else:
            ratio_abs = tap * fbase / tbase
            z_base = tbase * tbase / reader.mpc_base_power_MVA
            if fbase > tbase:
                resistance = float(branch.r) * ratio_abs**2 * z_base
                reactance = float(branch.x) * ratio_abs**2 * z_base
            else:
                resistance = float(branch.r) * z_base * tap**2
                reactance = float(branch.x) * z_base * tap**2
            rate_a = float(branch.rateA) * 1e6
            rated_power = rate_a if rate_a > 0.0 else reader.mpc_base_power_MVA
            transformer = module.Transformer(f"Transformer_{idx}_{fbus}_{tbus}")
            transformer.set_parameters(
                fbase,
                tbase,
                rated_power,
                abs(ratio_abs),
                math.radians(float(branch.angle)),
                diagonal(resistance),
                diagonal(reactance / omega),
            )
            transformer.connect([nodes[fbus], nodes[tbus]])
            components.append(transformer)

    system = dpsimpy.SystemTopology(frequency, list(nodes.values()), components)
    return system, nodes


def run_domain(reader, args, domain: str) -> float:
    build_start = time.perf_counter()
    system, nodes = build_system(reader, domain, args.frequency)
    build_elapsed = time.perf_counter() - build_start
    name = f"IEEE145_{domain}_3ph_{args.solver}"
    sim = dpsimpy.Simulation(name, dpsimpy.LogLevel.off)
    sim.set_system(system)
    sim.set_domain(dpsimpy.Domain.EMT if domain == "EMT" else dpsimpy.Domain.DP)
    sim.set_solver(dpsimpy.Solver.MNA)
    sim.set_direct_solver_implementation(
        getattr(dpsimpy.DirectLinearSolverImpl, args.solver)
    )
    sim.set_time_step(args.timestep)
    sim.set_final_time(args.duration)
    sim.do_init_from_nodes_and_terminals(True)
    sim.do_system_matrix_recomputation(True)

    logger = None
    if args.record:
        dpsimpy.Logger.set_log_dir(str(args.output_dir / name))
        logger = dpsimpy.Logger(name)
        for bus_id, node in nodes.items():
            logger.log_attribute(f"N{bus_id}.V", "v", node)
        sim.add_logger(logger)

    # Split initialization/factorization from steady stepping. Simulation.run()
    # combines both, which obscures scaling behavior for large sparse networks.
    start = time.perf_counter()
    sim.start()
    init_elapsed = time.perf_counter() - start

    step_elapsed = 0.0
    steps = 0
    current_time = args.timestep
    while current_time < args.duration + dpsimpy.DOUBLE_EPSILON:
        start = time.perf_counter()
        current_time = sim.next()
        step_elapsed += time.perf_counter() - start
        steps += 1
    sim.stop()

    print(
        f"RESULT domain={domain} buses={len(nodes)} components={len(system.components)} "
        f"steps={steps} timestep={args.timestep:g} duration={args.duration:g} "
        f"solver={args.solver} build_seconds={build_elapsed:.6f} "
        f"init_seconds={init_elapsed:.6f} step_seconds={step_elapsed:.6f} "
        f"avg_step_seconds={step_elapsed / steps:.6f} "
        f"total_seconds={build_elapsed + init_elapsed + step_elapsed:.6f}"
    )
    if args.record:
        print(f"RECORDED {args.output_dir / name / (name + '.csv')}")
    return build_elapsed + init_elapsed + step_elapsed


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case-file",
        type=Path,
        default=ROOT / "build" / "data" / "Matpower" / "case145results.mat",
    )
    parser.add_argument("--domain", choices=["EMT", "DP", "both"], default="both")
    parser.add_argument(
        "--solver", choices=["KLU", "SparseLU", "DenseLU"], default="KLU"
    )
    parser.add_argument("--frequency", type=float, default=60.0)
    parser.add_argument("--timestep", type=float, default=20e-6)
    parser.add_argument("--duration", type=float, default=0.02)
    parser.add_argument(
        "--record",
        action="store_true",
        help="record all bus voltages; adds logger overhead",
    )
    parser.add_argument("--output-dir", type=Path, default=ROOT / "logs")
    return parser.parse_args()


def main():
    args = parse_args()
    reader = load_case(args.case_file, args.frequency)
    print(
        f"CASE IEEE-145 buses={len(reader.mpc_bus_data)} branches={len(reader.mpc_branch_data)} generators={len(reader.mpc_gen_data)}"
    )
    domains = ["EMT", "DP"] if args.domain == "both" else [args.domain]
    for domain in domains:
        run_domain(reader, args, domain)


if __name__ == "__main__":
    main()
