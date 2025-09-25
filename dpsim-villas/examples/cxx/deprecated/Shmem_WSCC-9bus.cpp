// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim-villas/Interfaces.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {

  std::list<fs::path> filenames =
      DPsim::Utils::findFiles({"WSCC-09_RX_DI.xml", "WSCC-09_RX_EQ.xml",
                               "WSCC-09_RX_SV.xml", "WSCC-09_RX_TP.xml"},
                              "Examples/CIM/WSCC-09_RX", "CIMPATH");

  String simName = "Shmem_WSCC-9bus";

  CPS::CIM::Reader reader(simName, CPS::Logger::Level::info,
                          CPS::Logger::Level::info);
  SystemTopology sys = reader.loadCIM(60, filenames);

#ifdef WITH_RT
  RealTimeSimulation sim(simName, CPS::Logger::Level::debug);
#else
  Simulation sim(simName, CPS::Logger::Level::debug);
#endif
  sim.setSystem(sys);
  sim.setTimeStep(0.001);
  sim.setFinalTime(120);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.doInitFromNodesAndTerminals(true);

  const std::string shmemConfig = R"STRING(
    {
      "type": "shmem",
      "in": {
        "name": "dpsim-villas"
      },
      "out": {
        "name": "villas-dpsim"
      },
      "queuelen": 1024
    })STRING";

  auto intf = std::make_shared<InterfaceVillas>(shmemConfig);

  // Register exportable node voltages
  UInt o = 0;
  for (auto n : sys.mNodes) {
    auto v = n->attributeTyped<Complex>("v");

    intf->exportAttribute(v->deriveMag(), o + 0, true);
    intf->exportAttribute(v->derivePhase(), o + 1, true);

    o += 2;
  }

  sim.addInterface(intf);
#ifdef WITH_RT
  sim.run(10);
#else
  sim.run();
#endif

  return 0;
}
