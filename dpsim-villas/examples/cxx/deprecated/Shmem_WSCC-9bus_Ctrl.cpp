// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim-villas/Interfaces.h>

using namespace DPsim;
using namespace CPS::DP::Ph1;
using namespace CPS::Signal;

int main(int argc, char *argv[]) {

  CommandLineArgs args(argc, argv);

  // Find CIM files
  std::list<fs::path> filenames;
  if (argc <= 1) {
    filenames =
        DPsim::Utils::findFiles({"WSCC-09_RX_DI.xml", "WSCC-09_RX_EQ.xml",
                                 "WSCC-09_RX_SV.xml", "WSCC-09_RX_TP.xml"},
                                "Examples/CIM/WSCC-09_RX", "CIMPATH");
  } else {
    filenames = args.positionalPaths();
  }

  String simName = "Shmem_WSCC-9bus_Ctrl";
  CPS::Logger::setLogDir("logs/" + simName);

  CPS::CIM::Reader reader(simName, CPS::Logger::Level::info,
                          CPS::Logger::Level::off);
  SystemTopology sys = reader.loadCIM(60, filenames);

  // Extend system with controllable load (Profile)
  auto load_profile = PQLoadCS::make("load_cs_profile");
  load_profile->connect({sys.node<CPS::DP::SimNode>("BUS6")});
  load_profile->setParameters(0, 0, 230000);
  sys.mComponents.push_back(load_profile);

  // Extend system with controllable load
  auto load = PQLoadCS::make("load_cs");
  load->connect({sys.node<CPS::DP::SimNode>("BUS5")});
  load->setParameters(0, 0, 230000);
  sys.mComponents.push_back(load);

  // Controllers and filter
  std::vector<Real> coefficients_profile = std::vector<Real>(2000, 1. / 2000);
  std::vector<Real> coefficients = std::vector<Real>(100, 1. / 100);

  auto filtP_profile = FIRFilter::make("filter_p_profile", coefficients_profile,
                                       0, CPS::Logger::Level::off);
  load_profile->mActivePower->setReference(filtP_profile->mOutput);

  sys.mComponents.push_back(filtP_profile);

  auto filtP =
      FIRFilter::make("filter_p", coefficients, 0, CPS::Logger::Level::off);
  load->mActivePower->setReference(filtP->mOutput);
  sys.mComponents.push_back(filtP);

#ifdef WITH_RT
  RealTimeSimulation sim(simName, CPS::Logger::Level::off);
#else
  Simulation sim(simName, CPS::Logger::Level::off);
#endif
  sim.setSystem(sys);
  sim.setTimeStep(args.timeStep);
  sim.setFinalTime(args.duration);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.doInitFromNodesAndTerminals(true);

  const std::string shmemConfig = R"STRING(
    {
      "type": "shmem",
      "in": {
        "name": "dpsim1-villas"
      },
      "out": {
        "name": "villas-dpsim1"
      },
      "queuelen": 1024
    })STRING";

  auto intf = std::make_shared<InterfaceVillas>(shmemConfig);

  auto logger = DataLogger::make(simName);

  // Register exportable node voltages
  UInt o = 0;
  for (auto n : sys.mNodes) {
    UInt i;
    if (sscanf(n->name().c_str(), "BUS%u", &i) != 1) {
      std::cerr << "Failed to determine bus no of bus: " << n->name()
                << std::endl;
      continue;
    }

    i--;

    auto n_dp = std::dynamic_pointer_cast<CPS::DP::SimNode>(n);
    auto v = n_dp->mVoltage->deriveCoeff<Complex>(0, 0);

    std::cout << "Signal " << (i * 2) + 0 << ": Mag  " << n->name()
              << std::endl;
    std::cout << "Signal " << (i * 2) + 1 << ": Phas " << n->name()
              << std::endl;

    intf->exportAttribute(v->deriveMag(), (i * 2) + 0, true);
    o++;
    intf->exportAttribute(v->derivePhase(), (i * 2) + 1, true);
    o++;

    logger->logAttribute(fmt::format("mag_{}", i), v->deriveMag());
    logger->logAttribute(fmt::format("phase_{}", i), v->derivePhase());
  }

  logger->logAttribute("v3", sys.node<CPS::DP::SimNode>("BUS3")->mVoltage);

  // TODO gain by 20e8
  auto filtPInput = CPS::AttributeDynamic<Real>::make(0.0);
  intf->importAttribute(filtPInput, 0, true);
  filtP->setInput(filtPInput);

  auto filtPProfileInput = CPS::AttributeDynamic<Real>::make(0.0);
  intf->importAttribute(filtPProfileInput, 1, true);
  filtP_profile->setInput(filtPProfileInput);

  intf->exportAttribute(load->attributeTyped<Real>("P"), o++, true);
  intf->exportAttribute(load_profile->attributeTyped<Real>("P"), o++, true);

  sim.addInterface(intf);
  sim.addLogger(logger);
  sim.run(args.startTime);

  return 0;
}
