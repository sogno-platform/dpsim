// SPDX-License-Identifier: Apache-2.0

#include <DPsim.h>
#include <dpsim-villas/InterfaceShmem.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
  // Testing the interface with a simple circuit,
  // but the load is simulated in a different instance.
  // Values are exchanged using the ideal transformator model: an ideal
  // current source on the supply side and an ideal voltage source on the
  // supply side, whose values are received from the respective other circuit.
  // Here, the two instances directly communicate with each other without using
  // VILLASnode in between.

  if (argc < 2) {
    std::cerr << "not enough arguments (either 0 or 1 for the test number)"
              << std::endl;
    std::exit(1);
  }

  String in, out;

  if (String(argv[1]) == "0") {
    in = "/dpsim10";
    out = "/dpsim01";
  } else if (String(argv[1]) == "1") {
    in = "/dpsim01";
    out = "/dpsim10";
  } else {
    std::cerr << "invalid test number" << std::endl;
    std::exit(1);
  }

  Real timeStep = 0.001;
  Real finalTime = 0.1;

  if (String(argv[1]) == "0") {
    String simName = "ShmemDistributedDirect_1";
    CPS::Logger::setLogDir("logs/" + simName);

    // Nodes
    auto n1 = SimNode::make("n1", PhaseType::Single, std::vector<Complex>{10});
    auto n2 = SimNode::make("n2", PhaseType::Single, std::vector<Complex>{5});

    // Components
    auto evs = VoltageSource::make("v_intf", CPS::Logger::Level::debug);
    evs->setParameters(Complex(5, 0));
    auto vs1 = VoltageSource::make("vs_1", CPS::Logger::Level::debug);
    vs1->setParameters(Complex(10, 0));
    auto r12 = Resistor::make("r_12", CPS::Logger::Level::debug);
    r12->setParameters(1);

    // Connections
    evs->connect({SimNode::GND, n2});
    vs1->connect({SimNode::GND, n1});
    r12->connect({n1, n2});

    auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                              SystemComponentList{evs, vs1, r12});

    Simulation sim(simName);
    sim.setSystem(sys);
    sim.setTimeStep(timeStep);
    sim.setFinalTime(finalTime);

    // Logging
    auto logger = DataLogger::make(simName);
    logger->logAttribute("v1", n1->mVoltage);
    logger->logAttribute("v2", n2->mVoltage);
    logger->logAttribute("r12", r12->mIntfCurrent);
    logger->logAttribute("ievs", evs->mIntfCurrent);
    logger->logAttribute("vevs", evs->mIntfVoltage);
    sim.addLogger(logger);

    // Map attributes to interface entries
    InterfaceShmem intf(in, out);
    intf.importAttribute(evs->mVoltageRef, 0);
    auto evsAttrMinus = evs->mIntfCurrent->deriveCoeff<Complex>(0, 0);
    intf.exportAttribute(evsAttrMinus, 0);
    sim.addInterface(std::shared_ptr<Interface>(&intf));

    MatrixComp initialEvsCurrent = MatrixComp::Zero(1, 1);
    initialEvsCurrent(0, 0) = Complex(5, 0);
    evs->setIntfCurrent(initialEvsCurrent);

    sim.run();
  } else if (String(argv[1]) == "1") {
    String simName = "ShmemDistributedDirect_2";
    CPS::Logger::setLogDir("logs/" + simName);

    // Nodes
    auto n2 = SimNode::make("n2", PhaseType::Single, std::vector<Complex>{5});

    // Components
    auto ecs = CurrentSource::make("i_intf", CPS::Logger::Level::debug);
    ecs->setParameters(Complex(5, 0));
    auto r02 = Resistor::make("r_02", CPS::Logger::Level::debug);
    r02->setParameters(1);

    // Connections
    ecs->connect({SimNode::GND, n2});
    r02->connect({SimNode::GND, n2});

    auto sys =
        SystemTopology(50, SystemNodeList{n2}, SystemComponentList{ecs, r02});

    Simulation sim(simName);
    sim.setSystem(sys);
    sim.setTimeStep(timeStep);
    sim.setFinalTime(finalTime);

    // Logging
    auto logger = DataLogger::make(simName);
    logger->logAttribute("v2", n2->mVoltage);
    logger->logAttribute("r02", r02->mIntfCurrent);
    logger->logAttribute("vecs", ecs->mIntfVoltage);
    logger->logAttribute("iecs", ecs->mIntfCurrent);
    sim.addLogger(logger);

    // Map attributes to interface entries
    InterfaceShmem intf(in, out);
    intf.importAttribute(ecs->mCurrentRef, 0);
    //intf.exportComplex(ecs->mIntfVoltage->coeff(0, 0), 0);
    intf.exportAttribute(
        ecs->mIntfVoltage->deriveCoeff<Complex>(0, 0)->deriveScaled(
            Complex(-1., 0)),
        0);
    sim.addInterface(std::shared_ptr<Interface>(&intf));

    sim.run();
  }
}
