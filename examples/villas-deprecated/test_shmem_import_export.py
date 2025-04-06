# This example reads a sine signal from VILLASnode that modifies the active power set point
# of the PQ load. The n2 voltage is exported to a file via villas.

import sys
import os.path
import logging
import json

from datetime import datetime
from villas.node.node import Node as VILLASnode

import dpsimpy
import dpsimpyvillas

base = os.path.splitext(os.path.basename(sys.argv[0]))[0]
log = logging.getLogger(base)


def villas(intf):
    log_filename = datetime.now().strftime(f"{base}-villas-%y-%m-%d_%H_%M_%S.log")

    nodes = {
        "dpsim1": intf.get_config(),
        "file1": {"type": "file", "uri": f"{base}-results-%y-%m-%d_%H_%M_%S.csv"},
        "sine": {
            "type": "signal",
            "signal": "sine",
            "rate": 1,
            "frequency": 0.1,
            "amplitude": 50000,
            "offset": 100000,
        },
    }

    paths = [
        {"in": "dpsim1", "out": "file1"},
        {"in": "sine", "out": "dpsim1", "hooks": [{"type": "print"}]},
    ]

    config = {"nodes": nodes, "paths": paths}
    config["nodes"]["dpsim1"]["out"]["hooks"] = [{"type": "print"}]

    log.info("VILLASnode config: \n%s", json.dumps(config, indent=2))

    return VILLASnode(config=config, log_filename=log_filename)


def dpsim():
    # Parameters
    V_nom = 20e3
    p_load_nom = 100e3
    q_load_nom = 50e3
    line_resistance = 0.05
    line_inductance = 0.1
    line_capacitance = 0.1e-6
    name = "test_shmem_import_export"

    # Nodes and Components
    n1 = dpsimpy.sp.SimNode("n1", dpsimpy.PhaseType.Single)
    n2 = dpsimpy.sp.SimNode("n2", dpsimpy.PhaseType.Single)

    extnet = dpsimpy.sp.ph1.NetworkInjection("Slack")
    extnet.set_parameters(voltage_set_point=V_nom)
    extnet.set_base_voltage(V_nom)
    extnet.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.VD)

    line = dpsimpy.sp.ph1.PiLine("PiLine")
    line.set_parameters(R=line_resistance, L=line_inductance, C=line_capacitance)
    line.set_base_voltage(V_nom)

    load = dpsimpy.sp.ph1.Load("Load")
    load.set_parameters(
        active_power=p_load_nom, reactive_power=q_load_nom, nominal_voltage=V_nom
    )
    load.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PQ)

    extnet.connect([n1])
    line.connect([n1, n2])
    load.connect([n2])
    system = dpsimpy.SystemTopology(50, [n1, n2], [extnet, line, load])

    sim = dpsimpy.RealTimeSimulation(name, dpsimpy.LogLevel.debug)
    sim.set_system(system)
    sim.set_domain(dpsimpy.Domain.SP)
    sim.set_solver(dpsimpy.Solver.NRP)
    sim.set_time_step(1)
    sim.set_final_time(10)
    sim.do_init_from_nodes_and_terminals(False)

    logger = dpsimpy.Logger(name)
    sim.add_logger(logger)
    sim.log_attribute("n1.v", n1.attr("v"))
    sim.log_attribute("n2.v", n2.attr("v"))

    intf = dpsimpyvillas.InterfaceShmem()
    sim.add_interface(intf)
    sim.import_attribute(load.attr("P"), 0)
    sim.export_attribute(n2.attr("v").derive_coeff(0, 0).derive_mag(), 0)

    return sim, intf


def test_shmem_import_export():
    logging.basicConfig(
        format="[%(asctime)s %(name)s %(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
        level=logging.INFO,
    )

    sim, intf = dpsim()
    node = villas(intf)

    node.start()

    sim.run(1)

    node.stop()


if __name__ == "__main__":
    test_shmem_import_export()
