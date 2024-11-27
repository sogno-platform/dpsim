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
#include <dpsim-models/EMT/EMT_Ph3_ControlledVoltageSource.h>
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
  std::filesystem::path fpgaIpPath = "/usr/local/etc/villas/node/etc/fpga/vc707-xbar-pcie-dino/"
                                     "vc707-xbar-pcie-dino-v2.json";
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
        "name": "I_A",
        "type": "float",
        "unit": "A",
        "builtin": false
      }},
      {{
        "name": "I_B",
        "type": "float",
        "unit": "A",
        "builtin": false
      }},
      {{
        "name": "I_C",
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
        "name": "A",
        "type": "float",
        "unit": "V",
        "builtin": false
      }},
      {{
        "name": "B",
        "type": "float",
        "unit": "V",
        "builtin": false
      }},
      {{
        "name": "C",
        "type": "float",
        "unit": "V",
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

SystemTopology buildTopology(CommandLineArgs &args, std::shared_ptr<Interface> intfFpga, std::shared_ptr<DataLoggerInterface> logger) {
  // Nodes
  auto bus = SimNode::make("bus", PhaseType::ABC);

  // Components
  auto vs = Ph3::ControlledVoltageSource::make("vs");
  auto voltageRef = Matrix({{230e3}, {230e3}, {230e3}});
  vs->setParameters(voltageRef);

  auto load = Ph3::RXLoad::make("load");
  auto active = Matrix({{125e6, 0, 0}, {0, 125e6, 0}, {0, 0, 125e6}});
  // auto reactive = Matrix::Zero(3, 3);
  auto reactive = Matrix({{50e6, 0, 0}, {0, 50e6, 0}, {0, 0, 50e6}});
  load->setParameters(active, reactive, 230e3, true);

  // Topology
  vs->connect({bus, SimNode::GND});
  load->connect({bus});

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
  intfFpga->addImport(vs->mVoltageRef->deriveCoeff<Real>(0, 0), true, true);
  intfFpga->addImport(vs->mVoltageRef->deriveCoeff<Real>(1, 0), true, true);
  intfFpga->addImport(vs->mVoltageRef->deriveCoeff<Real>(2, 0), true, true);

  intfFpga->addExport(seqToDPsimAttribute);
  intfFpga->addExport(seqFromRTDSAttribute);
  intfFpga->addExport(load->mIntfCurrent->deriveCoeff<Real>(0, 0));
  intfFpga->addExport(load->mIntfCurrent->deriveCoeff<Real>(1, 0));
  intfFpga->addExport(load->mIntfCurrent->deriveCoeff<Real>(2, 0));
  intfFpga->addExport(seqNumForRTDS);

  // Logger
  if (logger) {
    logger->logAttribute("a", vs->mVoltageRef->deriveCoeff<Real>(0, 0));
    logger->logAttribute("b", vs->mVoltageRef->deriveCoeff<Real>(1, 0));
    logger->logAttribute("c", vs->mVoltageRef->deriveCoeff<Real>(2, 0));
    logger->logAttribute("a_i", load->mIntfCurrent->deriveCoeff<Real>(0, 0));
    logger->logAttribute("b_i", load->mIntfCurrent->deriveCoeff<Real>(1, 0));
    logger->logAttribute("c_i", load->mIntfCurrent->deriveCoeff<Real>(2, 0));
  }

  return SystemTopology(args.sysFreq, SystemNodeList{SimNode::GND, bus}, SystemComponentList{vs, load});
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv, "FpgaCosimulation", 0.01, 10 * 60, 60., -1, CPS::Logger::Level::info, CPS::Logger::Level::off, false, false, false, CPS::Domain::EMT);
  CPS::Logger::setLogDir("logs/" + args.name);
  bool log = args.options.find("log") != args.options.end() && args.getOptionBool("log");

  auto intfFpga = std::make_shared<InterfaceVillasQueueless>(buildFpgaConfig(args), "FpgaInterface", spdlog::level::off);

  std::filesystem::path logFilename = "logs/" + args.name + "/FpgaCosimulation.csv";
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

  CPS::Logger::get("FpgaCosimulation")->info("Simulation finished.");
  sim.logStepTimes("FpgaCosimulation");

  // std::ofstream of("task_dependencies.svg");
  // sim.dependencyGraph().render(of);
}
