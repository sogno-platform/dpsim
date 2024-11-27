/* Simple test circuit for testing connection to a FPGA via VILLASnode
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <filesystem>
#include <fstream>

#include <DPsim.h>
#include <dpsim-models/Attribute.h>
#include <dpsim-models/DP/DP_Ph1_CurrentSource.h>
#include <dpsim-models/DP/DP_Ph1_ProfileVoltageSource.h>
#include <dpsim-models/DP/DP_Ph1_VoltageSource.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-villas/InterfaceVillas.h>
#include <dpsim-villas/InterfaceVillasQueueless.h>
#include <dpsim/Event.h>
#include <dpsim/RealTimeDataLogger.h>
#include <dpsim/Utils.h>
#include <memory>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

const std::string buildFpgaConfig(CommandLineArgs &args) {
  std::filesystem::path fpgaIpPath = "/usr/local/etc/villas/node/etc/fpga/vc707-xbar-pcie-dino/"
                                     "vc707-xbar-pcie-dino.json";

  if (args.options.find("ips") != args.options.end()) {
    fpgaIpPath = std::filesystem::path(args.getOptionString("ips"));
  }
  std::string cardConfig = fmt::format(
      R"STRING("card": {{
      "interface": "pcie",
      "id": "10ee:7021",
      "slot": "0000:89:00.0",
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
        "unit": "V",
        "builtin": false
      }}],
      "hooks": [{{
        "type": "cast",
        "new_type": "float",
        "signal": "from_dpsim"
      }}]
    }})STRING");
  std::string signalInConfig = fmt::format(
      R"STRING("in": {{
      "signals": [{{
        "name": "seqnum",
        "type": "integer",
        "unit": "",
        "builtin": false
      }},
      {{
        "name": "to_dpsim",
        "type": "float",
        "unit": "V",
        "builtin": false
      }}],
      "hooks": [{{
        "type": "cast",
        "new_type": "complex",
        "signal": "to_dpsim"
      }}]
    }})STRING");
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

SystemTopology loopbackTopology(CommandLineArgs &args, std::shared_ptr<Interface> intf, std::shared_ptr<DataLoggerInterface> logger) {
  // Nodes
  auto n1 = SimNode::make("n1");

  // Components
  auto vs = VoltageSource::make("v_s");
  vs->setParameters(10.);
  auto rl = Resistor::make("r_l");
  rl->setParameters(1);

  // Topology
  vs->connect({n1, SimNode::GND});
  rl->connect({n1, SimNode::GND});

  // Interface
  auto seqnumAttribute = CPS::AttributeStatic<Int>::make(0);
  intf->addImport(seqnumAttribute, true, true);
  intf->addImport(vs->mVoltageRef, true, true);
  intf->addExport(n1->mVoltage->deriveCoeff<Complex>(0, 0));

  // Logger
  if (logger) {
    logger->logAttribute("v1", n1->mVoltage);
    logger->logAttribute("rl_i", rl->mIntfCurrent);
  }

  return SystemTopology(args.sysFreq, SystemNodeList{SimNode::GND, n1},
                        SystemComponentList{vs, rl});
}

SystemTopology hilTopology(CommandLineArgs &args, std::shared_ptr<Interface> intf, std::shared_ptr<DataLoggerInterface> logger) {
  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");

  // Components
  auto vs = VoltageSource::make("v_s");
  vs->setParameters(Complex(1, 0), 50);
  auto rs = Resistor::make("r_s");
  rs->setParameters(1);

  auto cs = CurrentSource::make("i_l");
  cs->setParameters(Complex(0, 0));

  // Topology
  vs->connect({n1, SimNode::GND});
  cs->connect({n1, n2});
  rs->connect({n2, SimNode::GND});

  // Interface
  auto seqnumAttribute = CPS::AttributeStatic<Int>::make(0);
  intf->addImport(seqnumAttribute, true, true);
  intf->addImport(cs->mCurrentRef, true, true);
  intf->addExport(n1->mVoltage->deriveCoeff<Complex>(0, 0));

  // Logger
  if (logger) {
    logger->logAttribute("v1", n1->mVoltage);
    logger->logAttribute("v2", n2->mVoltage);
    logger->logAttribute("cs_i", cs->mIntfCurrent);
  }

  return SystemTopology(args.sysFreq, SystemNodeList{SimNode::GND, n1, n2}, SystemComponentList{vs, rs, cs});
}

SystemTopology profileTopology(CommandLineArgs &args, std::shared_ptr<Interface> intf, std::shared_ptr<DataLoggerInterface> logger) {
  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");

  // Components
  auto vs = ProfileVoltageSource::make("v_s", "data.bin");
  auto rs = Resistor::make("r_s");
  rs->setParameters(1);

  auto cs = CurrentSource::make("i_l");
  cs->setParameters(Complex(0, 0));

  // Topology
  vs->connect({n1, SimNode::GND});
  cs->connect({n1, n2});
  rs->connect({n2, SimNode::GND});

  // Interface
  auto seqnumAttribute = CPS::AttributeStatic<Int>::make(0);
  intf->addImport(seqnumAttribute, true, true);
  intf->addImport(cs->mCurrentRef, true, true);
  intf->addExport(n1->mVoltage->deriveCoeff<Complex>(0, 0));

  // Logger
  if (logger) {
    logger->logAttribute("v1", n1->mVoltage);
    logger->logAttribute("v2", n2->mVoltage);
    logger->logAttribute("cs_i", cs->mIntfCurrent);
  }

  return SystemTopology(args.sysFreq, SystemNodeList{SimNode::GND, n1, n2}, SystemComponentList{vs, rs, cs});
}

SystemTopology getTopology(CommandLineArgs &args, std::shared_ptr<Interface> intf, std::shared_ptr<DataLoggerInterface> logger) {
  if (args.options.find("topology") != args.options.end()) {
    std::string topology = args.getOptionString("topology");
    if (topology == "hil") {
      return hilTopology(args, intf, logger);
    } else if (topology == "loopback") {
      return loopbackTopology(args, intf, logger);
    } else if (topology == "profile") {
      return profileTopology(args, intf, logger);
    }
  }
  return hilTopology(args, intf, logger);
}

std::shared_ptr<Event> getEvent(CommandLineArgs &args, SystemTopology &sys) {
  std::string topology = "hil";
  if (args.options.find("topology") != args.options.end()) {
    topology = args.getOptionString("topology");
  }
  if (args.options.find("event") != args.options.end()) {
    std::string event = args.getOptionString("event");
    if (event == "frequencyDrop") {
      if (topology != "hil") {
        throw std::runtime_error("frequencyDrop event only supported for topology \"hil\".");
      }
      auto vs = std::dynamic_pointer_cast<VoltageSource>(sys.mComponents[0]);
      return AttributeEvent<Real>::make(3, vs->mSrcFreq, 45.);
    }
    if (event == "voltageDrop") {
      if (topology != "hil") {
        throw std::runtime_error("voltageDrop event only supported for topology \"hil\".");
      }
      auto vs = std::dynamic_pointer_cast<VoltageSource>(sys.mComponents[0]);
      return AttributeEvent<Real>::make(3, vs->mVoltageRef->deriveReal(), 0.7);
    }
  }
  return nullptr;
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv, "FpgaExample", 0.01, 10 * 60, 5.);
  CPS::Logger::setLogDir("logs/" + args.name);
  bool log = args.options.find("log") != args.options.end() && args.getOptionBool("log");

  auto intf = std::make_shared<InterfaceVillasQueueless>(
      buildFpgaConfig(args), "FpgaExample", spdlog::level::off);
  std::filesystem::path logFilename = "logs/" + args.name + "/FpgaExample.csv";
  std::shared_ptr<DataLoggerInterface> logger = nullptr;
  if (log) {
    logger = RealTimeDataLogger::make(logFilename, args.duration, args.timeStep);
  }

  auto sys = getTopology(args, intf, logger);

  Simulation sim(args.name, args);
  sim.setSystem(sys);
  sim.addInterface(intf);
  sim.setLogStepTimes(false);

  auto event = getEvent(args, sys);
  if (event) {
    sim.addEvent(event);
  }

  if (log) {
    sim.addLogger(logger);
  }
  sim.run();

  CPS::Logger::get("FpgaExample")->info("Simulation finished.");
  sim.logStepTimes("FpgaExample");

  //std::ofstream of("task_dependencies.svg");
  //sim.dependencyGraph().render(of);
}
