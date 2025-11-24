/* Simple test circuit for testing connection to a FPGA via VILLASnode
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */
#include <filesystem>
#include <fstream>

#include <DPsim.h>

#include <dpsim-villas/Interfaces.h>

#include "../examples/cxx/Examples.h"

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph1;

const CPS::CIM::Examples::Components::GovernorKundur::Parameters govKundur;
const CPS::CIM::Examples::Components::ExcitationSystemEremia::Parameters
    excEremia;

const std::string buildFpgaConfig(CommandLineArgs &args) {
  std::filesystem::path fpgaIpPath =
      "/usr/local/etc/villas/node/etc/fpga/vc707-xbar-pcie/"
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
  SPDLOG_LOGGER_DEBUG(DPsim::Logger::get("FpgaExample"), "Config for Node:\n{}",
                      config);
  return config;
}

std::pair<SystemTopology, std::shared_ptr<std::vector<Event::Ptr>>>
hilTopology(CommandLineArgs &args, std::shared_ptr<Interface> intf,
            std::shared_ptr<DataLoggerInterface> logger) {
  std::string simName = "Fpga9BusHil";
  auto events = std::make_shared<std::vector<Event::Ptr>>();
  std::list<fs::path> filenames = Utils::findFiles(
      {"WSCC-09_Dyn_Full_DI.xml", "WSCC-09_Dyn_Full_EQ.xml",
       "WSCC-09_Dyn_Full_SV.xml", "WSCC-09_Dyn_Full_TP.xml"},
      "build/_deps/cim-data-src/WSCC-09/WSCC-09_Dyn_Full", "CIMPATH");

  // ----- POWERFLOW FOR INITIALIZATION -----
  // read original network topology
  String simNamePF = simName + "_PF";
  CPS::CIM::Reader reader(simNamePF);
  SystemTopology systemPF = reader.loadCIM(
      60, filenames, Domain::SP, PhaseType::Single, CPS::GeneratorType::PVNode);
  systemPF.component<CPS::SP::Ph1::SynchronGenerator>("GEN1")
      ->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);

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
  SystemTopology sys =
      reader2.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                      CPS::GeneratorType::FullOrderVBR);

  sys.initWithPowerflow(systemPF, CPS::Domain::EMT);

  sys.component<CPS::EMT::Ph3::RXLoad>("LOAD5")->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(125e6),
      CPS::Math::singlePhaseParameterToThreePhase(90e6), 230e3, true);

  sys.component<CPS::EMT::Ph3::RXLoad>("LOAD8")->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(100e6),
      CPS::Math::singlePhaseParameterToThreePhase(30e6), 230e3, true);

  sys.component<CPS::EMT::Ph3::RXLoad>("LOAD6")->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(90e6),
      CPS::Math::singlePhaseParameterToThreePhase(30e6), 230e3, true);

  auto gen1 = sys.component<CPS::EMT::Ph3::SynchronGeneratorVBR>("GEN1");
  gen1->addGovernor(govKundur.Ta_t, govKundur.Tb, govKundur.Tc, govKundur.Fa,
                    govKundur.Fb, govKundur.Fc, govKundur.Kg, govKundur.Tsr,
                    govKundur.Tsm, 1, 1);
  gen1->addExciter(excEremia.Ta, excEremia.Ka, excEremia.Te, excEremia.Ke,
                   excEremia.Tf, excEremia.Kf, excEremia.Tr);
  auto gen2 = sys.component<CPS::EMT::Ph3::SynchronGeneratorVBR>("GEN2");
  gen2->addGovernor(govKundur.Ta_t, govKundur.Tb, govKundur.Tc, govKundur.Fa,
                    govKundur.Fb, govKundur.Fc, govKundur.Kg, govKundur.Tsr,
                    govKundur.Tsm, 1, 1);
  gen2->addExciter(excEremia.Ta, excEremia.Ka, excEremia.Te, excEremia.Ke,
                   excEremia.Tf, excEremia.Kf, excEremia.Tr);
  auto gen3 = sys.component<CPS::EMT::Ph3::SynchronGeneratorVBR>("GEN3");
  gen3->addGovernor(govKundur.Ta_t, govKundur.Tb, govKundur.Tc, govKundur.Fa,
                    govKundur.Fb, govKundur.Fc, govKundur.Kg, govKundur.Tsr,
                    govKundur.Tsm, 1, 1);
  gen3->addExciter(excEremia.Ta, excEremia.Ka, excEremia.Te, excEremia.Ke,
                   excEremia.Tf, excEremia.Kf, excEremia.Tr);

  auto cs = Ph1::CurrentSource::make("cs");
  cs->setParameters(Complex(0, 0));
  cs->connect({SimNode::GND, sys.node<SimNode>("BUS6")});

  sys.addComponent(cs);

  // Interface
  auto seqnumAttribute = CPS::AttributeStatic<Int>::make(0);
  // auto current = CPS::AttributeStatic<Real>::make(0);

  auto scaledOutputVoltage = CPS::AttributeDynamic<Real>::make(0);
  // We scale the voltage so we map the nominal voltage in the simulation (230kV) to a nominal real peak voltage
  // of 15V. The amplifier has a gain of 20, so the voltage before the amplifier is 1/20 of the voltage at the load.
  constexpr double voltage_scale = 15. * 1.414 / (230e3 * 20.);
  auto updateFn = std::make_shared<CPS::AttributeUpdateTask<Real, Real>::Actor>(
      [](std::shared_ptr<Real> &dependent,
         typename CPS::Attribute<Real>::Ptr dependency) {
        *dependent = *dependency * voltage_scale;
        if (*dependent > 1.) {
          *dependent = 1.;
        } else if (*dependent < -1.) {
          *dependent = -1.;
        }
      });
  scaledOutputVoltage->addTask(
      CPS::UpdateTaskKind::UPDATE_ON_GET,
      CPS::AttributeUpdateTask<Real, Real>::make(
          CPS::UpdateTaskKind::UPDATE_ON_GET, *updateFn,
          sys.node<SimNode>("BUS6")->mVoltage->deriveCoeff<Real>(0, 0)));
  auto voltageInterfaceActive = CPS::AttributeStatic<CPS::Bool>::make(false);
  auto voltageActiveFn =
      std::make_shared<CPS::AttributeUpdateTask<Real, Bool>::Actor>(
          [](std::shared_ptr<Real> &dependent,
             typename CPS::Attribute<Bool>::Ptr dependency) {
            if (!*dependency) {
              *dependent = 0.;
            }
          });
  scaledOutputVoltage->addTask(CPS::UpdateTaskKind::UPDATE_ON_GET,
                               CPS::AttributeUpdateTask<Real, Bool>::make(
                                   CPS::UpdateTaskKind::UPDATE_ON_GET,
                                   *voltageActiveFn, voltageInterfaceActive));
  // We activate the voltage interface after 2 seconds to allow the current sensors to stabilize.
  auto activeVoltageEvent =
      AttributeEvent<Bool>::make(2., voltageInterfaceActive, true);
  events->push_back(activeVoltageEvent);

  // We scale the current using a gain so that we get a load in the simulation in the MVA range.
  // Additionally, the current sensor (LXSR 6-NPS) has a sensitivity of 0.1042 V/A (from datasheet),
  // we are using 3 turns, and we measured an offset voltage of -2.5268 V.
  constexpr double current_scale = 100. / (3. * 0.1042);
  constexpr double current_offset = -2.483036;
  auto scaledCurrent = CPS::AttributeDynamic<Real>::make(0);
  auto closedLoop = CPS::AttributeStatic<CPS::Bool>::make(false);
  auto currentScaleFn =
      std::make_shared<CPS::AttributeUpdateTask<Real, Bool>::Actor>(
          [](std::shared_ptr<Real> &dependent,
             typename CPS::Attribute<Bool>::Ptr dependency) {
            if (!*dependency) {
              *dependent = 0.;
            } else {
              *dependent = (*dependent + current_offset) * current_scale;
            }
          });
  scaledCurrent->addTask(
      CPS::UpdateTaskKind::UPDATE_ON_SET,
      CPS::AttributeUpdateTask<Real, Bool>::make(
          CPS::UpdateTaskKind::UPDATE_ON_SET, *currentScaleFn, closedLoop));
  auto closeLoopEvent = AttributeEvent<Bool>::make(2., closedLoop, true);
  events->push_back(closeLoopEvent);

  auto currentCopyFn =
      std::make_shared<CPS::AttributeUpdateTask<Real, Complex>::Actor>(
          [](std::shared_ptr<Real> &dependent,
             typename CPS::Attribute<Complex>::Ptr dependency) {
            dependency->set(Complex(*dependent, 0));
          });
  scaledCurrent->addTask(
      CPS::UpdateTaskKind::UPDATE_ON_SET,
      CPS::AttributeUpdateTask<Real, Complex>::make(
          CPS::UpdateTaskKind::UPDATE_ON_SET, *currentCopyFn, cs->mCurrentRef));

  intf->addImport(seqnumAttribute, true, true);
  intf->addImport(scaledCurrent, true, true);
  intf->addExport(scaledOutputVoltage);

  // Logger
  if (logger) {
    logger->logAttribute("interfaceVoltage", scaledOutputVoltage);
    logger->logAttribute("interfaceCurrent", scaledCurrent);
    logger->logAttribute("v1", sys.node<SimNode>("BUS1")->attribute("v"));
    logger->logAttribute("v2", sys.node<SimNode>("BUS2")->attribute("v"));
    logger->logAttribute("v3", sys.node<SimNode>("BUS3")->attribute("v"));
    logger->logAttribute("v4", sys.node<SimNode>("BUS4")->attribute("v"));
    logger->logAttribute("v5", sys.node<SimNode>("BUS5")->attribute("v"));
    logger->logAttribute("v6", sys.node<SimNode>("BUS6")->attribute("v"));
    logger->logAttribute("v7", sys.node<SimNode>("BUS7")->attribute("v"));
    logger->logAttribute("v8", sys.node<SimNode>("BUS8")->attribute("v"));
    logger->logAttribute("v9", sys.node<SimNode>("BUS9")->attribute("v"));
  }
  return {sys, events};
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv, "Fpga9BusHil", 0.01, 10 * 60, 60., -1,
                       CPS::Logger::Level::info, CPS::Logger::Level::off, false,
                       false, false, CPS::Domain::EMT);
  CPS::Logger::setLogDir("logs/" + args.name);
  bool log = args.options.find("log") != args.options.end() &&
             args.getOptionBool("log");

  auto intf = std::make_shared<InterfaceVillasQueueless>(
      buildFpgaConfig(args), "Fpga9BusHil", spdlog::level::off);
  std::filesystem::path logFilename = "logs/" + args.name + "/Fpga9BusHil.csv";
  std::shared_ptr<DataLoggerInterface> logger = nullptr;
  if (log) {
    logger =
        RealTimeDataLogger::make(logFilename, args.duration, args.timeStep);
  }

  auto topo = hilTopology(args, intf, logger);

  Simulation sim(args.name, args);
  sim.setSystem(topo.first);
  sim.addInterface(intf);
  sim.setLogStepTimes(false);
  sim.doSystemMatrixRecomputation(true);
  for (auto event : *topo.second) {
    sim.addEvent(event);
  }

  if (log) {
    sim.addLogger(logger);
  }

  sim.run();

  SPDLOG_LOGGER_INFO(CPS::Logger::get("FpgaExample"), "Simulation finished.");
  sim.logStepTimes("FpgaExample");
  topo.first.renderToFile("Fpga9BusHil.svg");
}
