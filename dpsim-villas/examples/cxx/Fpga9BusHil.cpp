/* Simple test circuit for testing connection to a FPGA via VILLASnode
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dpsim-models/EMT/EMT_Ph3_RXLoad.h"
#include "dpsim/Definitions.h"
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
using namespace CPS::EMT;
using namespace CPS::EMT::Ph1;

const std::string buildFpgaConfig(CommandLineArgs &args) {
  std::filesystem::path fpgaIpPath = "/home/eiling/projects/villas-node/etc/fpga/vc707-xbar-pcie/"
                                     "vc707-xbar-pcie.json";

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
      "3<->dma"
    ],
    "low_latency_mode": true,
    "timestep": {})STRING",
      fpgaIpPath.string(), args.timeStep);
  std::string signalOutConfig = fmt::format(
      R"STRING("out": {{
      "signals": [{{
        "name": "from_dpsim",
        "type": "float",
        "unit": "V",
        "builtin": false
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
        "unit": "A",
        "builtin": false
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

SystemTopology hilTopology(CommandLineArgs &args, std::shared_ptr<Interface> intf, std::shared_ptr<DataLoggerInterface> logger) {
  std::string simName = "Fpga9BusHil";

  std::list<fs::path> filenames =
      Utils::findFiles({"WSCC-09_Dyn_Full_DI.xml", "WSCC-09_Dyn_Full_EQ.xml", "WSCC-09_Dyn_Full_SV.xml", "WSCC-09_Dyn_Full_TP.xml"}, "build/_deps/cim-data-src/WSCC-09/WSCC-09_Dyn_Full", "CIMPATH");

  // ----- POWERFLOW FOR INITIALIZATION -----
  // read original network topology
  String simNamePF = simName + "_PF";
  CPS::CIM::Reader reader(simNamePF);
  SystemTopology systemPF = reader.loadCIM(60, filenames, Domain::SP, PhaseType::Single, CPS::GeneratorType::PVNode);
  systemPF.component<CPS::SP::Ph1::SynchronGenerator>("GEN1")->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);

  // define logging
  auto loggerPF = DPsim::DataLogger::make(simNamePF);
  for (auto node : systemPF.mNodes) {
    loggerPF->logAttribute(node->name() + ".V", node->attribute("v"));
  }

  // run powerflow
  Simulation simPF(simNamePF, CPS::Logger::Level::debug);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(0.1);
  simPF.setFinalTime(2 * 0.1);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(true);
  simPF.addLogger(loggerPF);
  simPF.run();

  // ----- DYNAMIC SIMULATION -----
  CPS::CIM::Reader reader2(simName);
  SystemTopology sys = reader2.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC, CPS::GeneratorType::FullOrder);

  sys.initWithPowerflow(systemPF, CPS::Domain::EMT);

  // Extend topology with switch
  // auto sw = Ph1::Switch::make("StepLoad");
  // sw->setParameters(1e9, 0.1);
  // sw->connect({SimNode::GND, sys.node<SimNode>("BUS6")});
  // sw->open();
  // sys.addComponent(sw);

  // sys.component<CPS::SimPowerComp>("BUS1")->

  // auto cs = Ph1::CurrentSource::make("cs");
  // cs->setParameters(Complex(0, 0));
  // cs->connect({SimNode::GND, sys.node<SimNode>("BUS6")});

  // sys.addComponent(cs);

  sys.component<CPS::EMT::Ph3::RXLoad>("LOAD5")->setParameters(Matrix({{125e6, 0, 0}, {0, 125e6, 0}, {0, 0, 125e6}}), Matrix({{90e6, 0, 0}, {0, 90e6, 0}, {0, 0, 90e6}}), 230e3, true);

  sys.component<CPS::EMT::Ph3::RXLoad>("LOAD8")->setParameters(Matrix({{100e6, 0, 0}, {0, 100e6, 0}, {0, 0, 100e6}}), Matrix({{30e6, 0, 0}, {0, 30e6, 0}, {0, 0, 30e6}}), 230e3, true);

  sys.component<CPS::EMT::Ph3::RXLoad>("LOAD6")->setParameters(Matrix({{90e6, 0, 0}, {0, 90e6, 0}, {0, 0, 90e6}}), Matrix({{30e6, 0, 0}, {0, 30e6, 0}, {0, 0, 30e6}}), 230e3, true);

  // Interface
  auto seqnumAttribute = CPS::AttributeStatic<Int>::make(0);
  auto current = CPS::AttributeStatic<Real>::make(0);
  intf->addImport(seqnumAttribute, true, true);
  intf->addImport(current, true, true);
  // intf->addImport(cs->mCurrentRef->deriveReal(), true, true);
  intf->addExport(sys.node<SimNode>("BUS6")->mVoltage->deriveCoeff<Real>(0, 0));

  // Logger
  if (logger) {
    // logger->logAttribute("cs", cs->mCurrentRef->deriveReal());
    logger->logAttribute("v1", sys.node<SimNode>("BUS1")->attribute("v"));
    logger->logAttribute("v2", sys.node<SimNode>("BUS2")->attribute("v"));
    logger->logAttribute("v3", sys.node<SimNode>("BUS3")->attribute("v"));
    logger->logAttribute("v4", sys.node<SimNode>("BUS4")->attribute("v"));
    logger->logAttribute("v5", sys.node<SimNode>("BUS5")->attribute("v"));
    logger->logAttribute("v6", sys.node<SimNode>("BUS6")->attribute("v"));
    logger->logAttribute("v7", sys.node<SimNode>("BUS7")->attribute("v"));
    logger->logAttribute("v8", sys.node<SimNode>("BUS8")->attribute("v"));
    logger->logAttribute("v9", sys.node<SimNode>("BUS9")->attribute("v"));
    // logger->logAttribute("wr_1", sys.component<CPS::EMT::Ph3::SynchronGeneratorDQTrapez>("GEN1")->attribute("w_r"));
    // logger->logAttribute("wr_2", sys.component<CPS::EMT::Ph3::SynchronGeneratorDQTrapez>("GEN2")->attribute("w_r"));
    // logger->logAttribute("wr_3", sys.component<CPS::EMT::Ph3::SynchronGeneratorDQTrapez>("GEN3")->attribute("w_r"));
  }
  return sys;
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv, "Fpga9BusHil", 0.01, 10 * 60, 60., -1, CPS::Logger::Level::info, CPS::Logger::Level::off, false, false, false, CPS::Domain::EMT);
  CPS::Logger::setLogDir("logs/" + args.name);
  bool log = args.options.find("log") != args.options.end() && args.getOptionBool("log");

  auto intf = std::make_shared<InterfaceVillasQueueless>(buildFpgaConfig(args), "Fpga9BusHil", spdlog::level::off);
  std::filesystem::path logFilename = "logs/" + args.name + "/Fpga9BusHil.csv";
  std::shared_ptr<DataLoggerInterface> logger = nullptr;
  if (log) {
    logger = RealTimeDataLogger::make(logFilename, args.duration, args.timeStep);
  }

  auto sys = hilTopology(args, intf, logger);

  Simulation sim(args.name, args);
  sim.setSystem(sys);
  sim.addInterface(intf);
  sim.setLogStepTimes(false);

  if (log) {
    sim.addLogger(logger);
  }

  sim.run();

  CPS::Logger::get("FpgaExample")->info("Simulation finished.");
  sim.logStepTimes("FpgaExample");
  sys.renderToFile("Fpga9BusHil.svg");
  // std::ofstream of("task_dependencies.svg");
  // sim.dependencyGraph().render(of);
}
