/* Co-simulation example where RTDS simulates a WSCC 9 bus and the
 * load connected to bus 5 is simulated in DPSim.
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <dpsim-models/Logger.h>
#include <filesystem>
#include <fstream>

#include <DPsim.h>
#include <dpsim-models/Attribute.h>
#include <dpsim-models/EMT/EMT_Ph3_ControlledCurrentSource.h>
#include <dpsim-models/EMT/EMT_Ph3_ControlledVoltageSource.h>
#include <dpsim-models/EMT/EMT_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-villas/InterfaceVillas.h>
#include <dpsim-villas/InterfaceVillasQueueless.h>
#include <dpsim/Event.h>
#include <dpsim/RealTimeDataLogger.h>
#include <dpsim/Utils.h>
#include <memory>

using namespace DPsim;
using namespace CPS::EMT;

const std::string buildFpgaConfig(CommandLineArgs &args) {
  std::filesystem::path fpgaIpPath = "/usr/local/etc/villas/node/etc/fpga/vc707-xbar-pcie/"
                                     "vc707-xbar-pcie.json";
  ;

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
      "0<->1",
      "2<->dma"
    ],
    "low_latency_mode": true,
    "timestep": {})STRING",
      fpgaIpPath.string(), args.timeStep);
  std::string signalOutConfig = fmt::format(
      R"STRING("out": {{
      "signals": [{{
        "name": "seq_to_dpsim",
        "type": "integer",
        "unit": "",
        "builtin": false
      }},
      {{
        "name": "seq_to_rtds",
        "type": "integer",
        "unit": "",
        "builtin": false
      }},
      {{
        "name": "VA",
        "type": "float",
        "unit": "V",
        "builtin": false
      }},
      {{
        "name": "VB",
        "type": "float",
        "unit": "V",
        "builtin": false
      }},
      {{
        "name": "VC",
        "type": "float",
        "unit": "A",
        "builtin": false
      }},
      {{
        "name": "seqnum",
        "type": "integer",
        "unit": "",
        "builtin": false
      }}]
    }})STRING");
  std::string signalInConfig = fmt::format(
      R"STRING("in": {{
      "signals": [{{
        "name": "seq_from_dpsim",
        "type": "integer",
        "unit": "",
        "builtin": false
      }},
      {{
        "name": "seq_from_rtds",
        "type": "integer",
        "unit": "",
        "builtin": false
      }},
      {{
        "name": "IA",
        "type": "float",
        "unit": "A",
        "builtin": false
      }},
      {{
        "name": "IB",
        "type": "float",
        "unit": "A",
        "builtin": false
      }},
      {{
        "name": "IC",
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
  DPsim::Logger::get("FpgaCosim3PhInfiniteBus")->debug("Config for Node:\n{}", config);
  return config;
}

SystemTopology buildTopology(CommandLineArgs &args, std::shared_ptr<Interface> intfFpga, std::shared_ptr<DataLoggerInterface> logger) {
  // Nodes
  auto bus1 = SimNode::make("bus1", PhaseType::ABC);
  auto bus2 = SimNode::make("bus2", PhaseType::ABC);
  auto bus3 = SimNode::make("bus3", PhaseType::ABC);

  // Components
  auto vs = Ph3::VoltageSource::make("vs");
  vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(230e3 / sqrt(3), 0)), 50);

  auto r = Ph3::Resistor::make("R");
  r->setParameters(Matrix{{10.4275, 0, 0}, {0, 10.4275, 0}, {0, 0, 10.4275}});

  auto l = Ph3::Inductor::make("L");
  l->setParameters(Matrix{{0.325101, 0, 0}, {0, 0.325101, 0}, {0, 0, 0.325101}});

  auto cs = Ph3::ControlledCurrentSource::make("cs");
  cs->setParameters(Matrix{{0.0}, {0.0}, {0.0}});

  // Topology
  vs->connect({SimNode::GND, bus1});
  r->connect({bus1, bus2});
  l->connect({bus2, bus3});
  cs->connect({bus3, SimNode::GND});

  // Interface
  auto seqFromRTDSAttribute = CPS::AttributeStatic<Int>::make(0);
  auto seqFromDPsimAttribute = CPS::AttributeStatic<Int>::make(0);
  auto seqToDPsimAttribute = CPS::AttributeDynamic<Int>::make(0);
  auto updateFn = std::make_shared<CPS::AttributeUpdateTask<Int, Int>::Actor>([](std::shared_ptr<Int> &dependent, typename CPS::Attribute<Int>::Ptr dependency) { *dependent = *dependent + 1; });
  seqToDPsimAttribute->addTask(CPS::UpdateTaskKind::UPDATE_ON_GET, CPS::AttributeUpdateTask<Int, Int>::make(CPS::UpdateTaskKind::UPDATE_ON_GET, *updateFn, seqFromDPsimAttribute));
  auto seqNumForRTDS = CPS::AttributeDynamic<Int>::make(0);
  seqNumForRTDS->addTask(CPS::UpdateTaskKind::UPDATE_ON_GET, CPS::AttributeUpdateTask<Int, Int>::make(CPS::UpdateTaskKind::UPDATE_ON_GET, *updateFn, seqFromDPsimAttribute));

  intfFpga->addImport(seqFromRTDSAttribute, true, true);
  intfFpga->addImport(seqFromDPsimAttribute, true, true);
  intfFpga->addImport(cs->mCurrentRef->deriveCoeff<Real>(0, 0), true, true);
  intfFpga->addImport(cs->mCurrentRef->deriveCoeff<Real>(1, 0), true, true);
  intfFpga->addImport(cs->mCurrentRef->deriveCoeff<Real>(2, 0), true, true);

  intfFpga->addExport(seqToDPsimAttribute);
  intfFpga->addExport(seqFromRTDSAttribute);
  intfFpga->addExport(bus3->mVoltage->deriveCoeff<Real>(0, 0));
  intfFpga->addExport(bus3->mVoltage->deriveCoeff<Real>(1, 0));
  intfFpga->addExport(bus3->mVoltage->deriveCoeff<Real>(2, 0));
  intfFpga->addExport(seqNumForRTDS);

  // Logger
  if (logger) {
    logger->logAttribute("a", bus3->mVoltage->deriveCoeff<Real>(0, 0));
    logger->logAttribute("b", bus3->mVoltage->deriveCoeff<Real>(1, 0));
    logger->logAttribute("c", bus3->mVoltage->deriveCoeff<Real>(2, 0));
    logger->logAttribute("a_i", cs->mIntfCurrent->deriveCoeff<Real>(0, 0));
    logger->logAttribute("b_i", cs->mIntfCurrent->deriveCoeff<Real>(1, 0));
    logger->logAttribute("c_i", cs->mIntfCurrent->deriveCoeff<Real>(2, 0));
  }

  return SystemTopology(args.sysFreq, SystemNodeList{SimNode::GND, bus1, bus2, bus3}, SystemComponentList{vs, cs, r, l});
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv, "FpgaCosim3PhInfiniteBus", 0.01, 10 * 60, 50., -1, CPS::Logger::Level::info, CPS::Logger::Level::off, false, false, false, CPS::Domain::EMT);
  CPS::Logger::setLogDir("logs/" + args.name);
  bool log = args.options.find("log") != args.options.end() && args.getOptionBool("log");

  auto intfFpga = std::make_shared<InterfaceVillasQueueless>(buildFpgaConfig(args), "FpgaInterface", spdlog::level::off);

  std::filesystem::path logFilename = "logs/" + args.name + "/FpgaCosim3PhInfiniteBus.csv";
  std::shared_ptr<DataLoggerInterface> logger = nullptr;
  if (log) {
    logger = RealTimeDataLogger::make(logFilename, args.duration, args.timeStep);
  }

  auto sys = buildTopology(args, intfFpga, logger);

  Simulation sim(args.name, args);
  sim.setSystem(sys);
  sim.addInterface(intfFpga);
  sim.setLogStepTimes(false);

  if (log) {
    sim.addLogger(logger);
  }
  sim.run();

  CPS::Logger::get("FpgaCosim3PhInfiniteBus")->info("Simulation finished.");
  sim.logStepTimes("FpgaCosim3PhInfiniteBus");

  // std::ofstream of("task_dependencies.svg");
  // sim.dependencyGraph().render(of);
}
