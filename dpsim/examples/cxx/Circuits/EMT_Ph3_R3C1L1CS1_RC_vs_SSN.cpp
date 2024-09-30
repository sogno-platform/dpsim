#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;

void EMT_PH3_R3_C1_L1_CS1() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simNameRC = "EMT_Ph3_R3C1L1CS1_RC_vs_SSN_RC";

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

  // Components

  Matrix param = Matrix::Zero(3, 3);
  param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;

  auto cs0 = Ph3::CurrentSource::make("CS0");
  cs0->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0, 0.0)),
      50.0);

  auto r1 = Ph3::Resistor::make("R1");
  r1->setParameters(10 * param);

  auto r2 = Ph3::Resistor::make("R2");
  r2->setParameters(param);

  auto r3 = Ph3::Resistor::make("R3");
  r3->setParameters(5 * param);

  auto l1 = Ph3::Inductor::make("L1");
  l1->setParameters(0.02 * param);

  auto c1 = Ph3::Capacitor::make("C1");
  c1->setParameters(0.001 * param);

  // Topology
  cs0->connect(SimNode::List{n1, SimNode::GND});

  r1->connect(SimNode::List{n2, n1});
  r2->connect(SimNode::List{n2, SimNode::GND});
  r3->connect(SimNode::List{n2, SimNode::GND});

  l1->connect(SimNode::List{n2, SimNode::GND});

  c1->connect(SimNode::List{n1, n2});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{cs0, r1, r2, r3, l1, c1});

  // Logging
  Logger::setLogDir("logs/" + simNameRC);
  auto logger = DataLogger::make(simNameRC);
  logger->logAttribute("V1", n1->attribute("v"));
  logger->logAttribute("V2", n2->attribute("v"));
  logger->logAttribute("V_C1", c1->attribute("v_intf"));
  logger->logAttribute("I_L1", l1->attribute("i_intf"));

  Simulation sim(simNameRC, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_PH3_SSN_R3_C1_L1_CS1() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simNameSSN = "EMT_Ph3_R3C1L1CS1_RC_vs_SSN_SSN";
  Logger::setLogDir("logs/" + simNameSSN);

  // Nodes
  auto n1 = CPS::EMT::SimNode::make("n1", PhaseType::ABC);
  auto n2 = CPS::EMT::SimNode::make("n2", PhaseType::ABC);

  // Components

  Matrix param = Matrix::Zero(3, 3);
  param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;

  auto cs0 = Ph3::CurrentSource::make("CS0");
  cs0->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0, 0.0)),
      50.0);

  auto r1 = Ph3::Resistor::make("R1", Logger::Level::debug);
  r1->setParameters(10 * param);

  auto r2 = Ph3::Resistor::make("R2", Logger::Level::debug);
  r2->setParameters(param);

  auto r3 = Ph3::Resistor::make("R3", Logger::Level::debug);
  r3->setParameters(5 * param);

  auto l1 = Ph3::SSN::Inductor::make("L1");
  l1->setParameters(0.02 * param);

  auto c1 = Ph3::SSN::Capacitor::make("C1");
  c1->setParameters(0.001 * param);

  // Topology
  cs0->connect(CPS::EMT::SimNode::List{n1, CPS::EMT::SimNode::GND});

  r1->connect(CPS::EMT::SimNode::List{n2, n1});
  r2->connect(CPS::EMT::SimNode::List{n2, CPS::EMT::SimNode::GND});
  r3->connect(CPS::EMT::SimNode::List{n2, CPS::EMT::SimNode::GND});

  l1->connect(CPS::EMT::SimNode::List{n2, CPS::EMT::SimNode::GND});

  c1->connect(CPS::EMT::SimNode::List{n1, n2});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{cs0, r1, r2, r3, l1, c1});

  // Logging
  auto logger = DataLogger::make(simNameSSN);
  logger->logAttribute("V1_SSN", n1->attribute("v"));
  logger->logAttribute("V2_SSN", n2->attribute("v"));
  logger->logAttribute("V_C1_SSN", c1->attribute("v_intf"));
  logger->logAttribute("I_L1_SSN", l1->attribute("i_intf"));

  Simulation sim(simNameSSN, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

int main(int argc, char *argv[]) {
  EMT_PH3_R3_C1_L1_CS1();
  EMT_PH3_SSN_R3_C1_L1_CS1();
}
