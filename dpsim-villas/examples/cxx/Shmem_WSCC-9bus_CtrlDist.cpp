// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim-models/CIM/Reader.h>
#include <dpsim-villas/InterfaceShmem.h>

using namespace DPsim;
using namespace CPS;
using namespace CPS::DP::Ph1;
using namespace CPS::Signal;

int main(int argc, char *argv[]) {

  CommandLineArgs args(argc, argv, "Shmem_WSCC-9bus_CtrlDist", 0.001, 20, 60);

  if (args.scenario == 0) {
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

    CIM::Reader reader(args.name, CPS::Logger::Level::info,
                       CPS::Logger::Level::info);
    SystemTopology sys = reader.loadCIM(args.sysFreq, filenames);

    // Extend system with controllable load (Profile)
    auto load_profile = PQLoadCS::make("load_cs_profile", 0, 0, 230000,
                                       CPS::Logger::Level::info);
    load_profile->connect({sys.node<DP::SimNode>("BUS7")});
    sys.mComponents.push_back(load_profile);

    // Extend system with controllable load
    auto ecs =
        CurrentSource::make("i_intf", Complex(0, 0), CPS::Logger::Level::debug);
    ecs->connect({sys.node<DP::SimNode>("BUS4"), DP::SimNode::GND});
    sys.mComponents.push_back(ecs);

    RealTimeSimulation sim(args.name + "_1", CPS::Logger::Level::debug);
    sim.setSystem(sys);
    sim.setTimeStep(args.timeStep);
    sim.setFinalTime(args.duration);
    sim.setDomain(Domain::DP);
    sim.setSolverType(Solver::Type::MNA);
    sim.doInitFromNodesAndTerminals(true);

    InterfaceShmem intf1("/dpsim01", "/dpsim10", nullptr, false);
    InterfaceShmem intf2("/dpsim1-villas", "/villas-dpsim1", nullptr, false);
    sim.addInterface(std::shared_ptr<Interface>(&intf1));
    sim.addInterface(std::shared_ptr<Interface>(&intf2));

    // Controllers and filter
    std::vector<Real> coefficients_profile = std::vector<Real>(2000, 1. / 2000);

    auto filtP_profile = FIRFilter::make(
        "filter_p_profile", coefficients_profile, 0, CPS::Logger::Level::info);
    load_profile->mActivePower->setReference(filtP_profile->mOutput);
    sys.mComponents.push_back(filtP_profile);

    // Register interface current source and voltage drop
    intf1.importAttribute(ecs->mCurrentRef, 0);
    intf1.exportAttribute(ecs->mIntfVoltage->deriveCoeff<Complex>(0, 0), 0);

    // TODO: gain by 20e8
    filtP_profile->setInput(intf2.importReal(0));

    // Register exportable node voltages
    for (auto n : sys.mNodes) {
      UInt i;
      if (sscanf(n->name().c_str(), "BUS%u", &i) != 1) {
        std::cerr << "Failed to determine bus no of bus: " << n->name()
                  << std::endl;
        continue;
      }

      i--;

      auto v = n->attributeTyped<Complex>("v");

      std::cout << "Signal " << (i * 2) + 0 << ": Mag " << n->name()
                << std::endl;
      std::cout << "Signal " << (i * 2) + 1 << ": Phas " << n->name()
                << std::endl;

      intf2.exportReal(v->deriveMag(), (i * 2) + 0);
      intf2.exportReal(v->derivePhase(), (i * 2) + 1);
    }

    sim.run(args.startTime);
  }

  if (args.scenario == 1) {
    // Nodes
    auto n1 = DP::SimNode::make(
        "n1", PhaseType::Single,
        std::vector<Complex>({Complex(02.180675e+05, -1.583367e+04)}));

    // Add interface voltage source
    auto evs = VoltageSource::make("v_intf", CPS::Logger::Level::debug);
    evs->setParameters(Complex(0, 0));
    evs->connect({DP::SimNode::GND, n1});

    // Extend system with controllable load
    auto load = PQLoadCS::make("load_cs", 0, 0, 230000);
    load->connect({n1});

    // Controllers and filter
    std::vector<Real> coefficients = std::vector<Real>(100, 1. / 100);
    auto filtP =
        FIRFilter::make("filter_p", coefficients, 0, CPS::Logger::Level::info);
    load->mActivePower->setReference(filtP->mOutput);

    auto sys = SystemTopology(args.sysFreq, SystemNodeList{n1},
                              SystemComponentList{evs, load, filtP});
    RealTimeSimulation sim(args.name + "_2");
    sim.setSystem(sys);
    sim.setTimeStep(args.timeStep);
    sim.setFinalTime(args.duration);

    InterfaceShmem intf1("/dpsim10", "/dpsim01", nullptr, false);
    sim.addInterface(std::shared_ptr<Interface>(&intf1));

    InterfaceShmem intf2("/dpsim2-villas", "/villas-dpsim2", nullptr, false);
    sim.addInterface(std::shared_ptr<Interface>(&intf2));

    // Register voltage source reference and current flowing through source
    // multiply with -1 to consider passive sign convention
    intf1.importAttribute(evs->mVoltageRef, 0);
    // TODO: invalid sign
    intf1.exportAttribute(evs->mIntfCurrent->deriveCoeff<Complex>(0, 0), 0);

    // Register controllable load
    filtP->setInput(intf2.importReal(0));
    intf2.exportReal(load->mActivePower, 0);
    intf2.exportComplex(load->mIntfVoltage->deriveCoeff<Complex>(0, 0), 1);
    intf2.exportComplex(load->mIntfCurrent->deriveCoeff<Complex>(0, 0), 2);

    sim.run(args.startTime);
  }

  return 0;
}
