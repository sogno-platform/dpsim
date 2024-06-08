/* Simple test circuit for testing connection to a FPGA via VILLASnode
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <filesystem>
#include <fstream>

#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph1_CurrentSource.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-villas/InterfaceVillas.h>
#include <dpsim/Utils.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

const std::string buildFpgaConfig(CommandLineArgs &args) {
  std::filesystem::path fpgaIpPath =
      "/usr/local/etc/villas/node/etc/fpga/vc707-xbar-pcie-dino/"
      "vc707-xbar-pcie-dino.json";

  if (args.options.find("ips") != args.options.end()) {
    fpgaIpPath = std::filesystem::path(args.getOptionString("ips"));
  }
  std::string cardConfig = fmt::format(
      R"STRING("card": {{
      "interface": "pcie",
      "id": "10ee:7021",
      "slot": "0000:88:00.0",
      "do_reset": true,
      "ips": "{}",
      "polling": true
    }},
    "connect": [
      "0->3",
      "dino<-dma",
      "dino->dma"
    ],
    "low_latency_mode": true,
    "timestep": {})STRING",
      fpgaIpPath.string(), args.timeStep);
  std::string signalOutConfig = fmt::format(
      R"STRING("out": {{
      "signals": [{{
        "name": "from_dpsim",
        "type": "complex",
        "unit": "V"
      }}],
      "hooks": [{{
        "type": "dp",
        "signal": "from_dpsim",
        "f0": {},
        "dt": {},
        "harmonics": [0],
        "inverse": true
      }}]
    }})STRING",
      args.sysFreq, args.timeStep);
  std::string signalInConfig = fmt::format(
      R"STRING("in": {{
      "signals": [{{
        "name": "to_dpsim",
        "type": "float",
        "unit": "V",
        "builtin": false
      }}],
      "hooks": ["print", {{
        "type": "dp",
        "signal": "to_dpsim",
        "f0": {},
        "dt": {},
        "harmonics": [0],
        "inverse": false
      }}]
    }})STRING",
      args.sysFreq, args.timeStep);
  const std::string config = fmt::format(
      R"STRING({{
    "type": "fpga",
    {},
    {},
    {}
  }})STRING",
      cardConfig, signalOutConfig, signalInConfig);
  DPsim::Logger::get("FpgaExample")->debug("Config for Node:\n{}", config);
  return config;
}

SystemTopology loopbackTopology(CommandLineArgs &args,
                                std::shared_ptr<InterfaceVillas> intf,
                                std::shared_ptr<DataLogger> logger) {
  // Nodes
  auto n1 = SimNode::make("n1");

  // Components
  auto vs = VoltageSource::make("v_s");
  vs->setParameters(Complex(10, 0), args.sysFreq);
  auto rl = Resistor::make("r_l");
  rl->setParameters(1);

  // Topology
  vs->connect({SimNode::GND, n1});
  rl->connect({n1, SimNode::GND});

  // Interface
  intf->importAttribute(vs->mVoltageRef, 0, false, false, "from_dino", "A");
  intf->exportAttribute(n1->mVoltage->deriveCoeff<Complex>(0, 0), 0, true,
                        "to_dino", "V");
  intf->printVillasSignals();

  // Logger
  logger->logAttribute("v1", n1->mVoltage);
  logger->logAttribute("rl_i", rl->mIntfCurrent);

  return SystemTopology(args.sysFreq, SystemNodeList{SimNode::GND, n1},
                        SystemComponentList{vs, rl});
}

SystemTopology hilTopology(CommandLineArgs &args,
                           std::shared_ptr<InterfaceVillas> intf,
                           std::shared_ptr<DataLogger> logger) {
  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");

  // Components
  auto vs = VoltageSource::make("v_s");
  vs->setParameters(Complex(10, 0), args.sysFreq);
  auto rs = Resistor::make("r_s");
  rs->setParameters(1);

  auto cs = CurrentSource::make("i_l");
  cs->setParameters(Complex(0, 0));

  // Topology
  vs->connect({SimNode::GND, n1});
  rs->connect({n1, n2});
  cs->connect({n2, SimNode::GND});

  // Interface
  intf->importAttribute(cs->mCurrentRef, 0, false, false, "from_dino", "A");
  intf->exportAttribute(n2->mVoltage->deriveCoeff<Complex>(0, 0), 0, true,
                        "to_dino", "V");
  intf->printVillasSignals();

  // Logger
  logger->logAttribute("v1", n1->mVoltage);
  logger->logAttribute("v2", n2->mVoltage);
  logger->logAttribute("cs_i", cs->mIntfCurrent);

  return SystemTopology(args.sysFreq, SystemNodeList{SimNode::GND, n1, n2},
                        SystemComponentList{vs, rs, cs});
}

SystemTopology getTopology(CommandLineArgs &args,
                           std::shared_ptr<InterfaceVillas> intf,
                           std::shared_ptr<DataLogger> logger) {
  if (args.options.find("topology") != args.options.end()) {
    std::string topology = args.getOptionString("topology");
    if (topology == "hil") {
      return hilTopology(args, intf, logger);
    } else if (topology == "loopback") {
      return loopbackTopology(args, intf, logger);
    }
  }
  return hilTopology(args, intf, logger);
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv, "FpgaExample", 0.01, 10 * 60, 5.);
  CPS::Logger::setLogDir("logs/" + args.name);

  auto intf = std::make_shared<InterfaceVillas>(buildFpgaConfig(args));
  auto logger = DataLogger::make(args.name);

  auto sys = getTopology(args, intf, logger);

  RealTimeSimulation sim(args.name, args);
  sim.setSystem(sys);
  sim.addInterface(intf);
  sim.addLogger(logger);
  sim.run();

  //std::ofstream of("task_dependencies.svg");
  //sim.dependencyGraph().render(of);
}
