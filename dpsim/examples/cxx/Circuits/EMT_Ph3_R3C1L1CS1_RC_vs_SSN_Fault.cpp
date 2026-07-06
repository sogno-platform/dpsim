// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;

// EMT counterpart of DP_Ph3_R3C1L1CS1_RC_vs_SSN_Fault.cpp, used as the
// cross-domain reference for the DP-SSN three-phase fault validation: same
// topology, parameters and fault timing, run in the EMT (time-domain) frame
// instead of DP (dynamic phasor / SFA).
static const Real switchOpenR = 1e6;
static const Real switchClosedR = 0.001;
static const Real startTimeFault = 0.03;
static const Real endTimeFault = 0.06;

void EMT_Ph3_R3_C1_L1_CS1_Fault_RC() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simNameRC = "EMT_Ph3_R3C1L1CS1_RC_vs_SSN_Fault_RC";

  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

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

  auto fault = Ph3::SeriesSwitch::make("fault");
  fault->setParameters(switchOpenR, switchClosedR);
  fault->open();

  cs0->connect(SimNode::List{n1, SimNode::GND});
  r1->connect(SimNode::List{n2, n1});
  r2->connect(SimNode::List{n2, SimNode::GND});
  r3->connect(SimNode::List{n2, SimNode::GND});
  l1->connect(SimNode::List{n2, SimNode::GND});
  c1->connect(SimNode::List{n1, n2});
  fault->connect(SimNode::List{n2, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2},
                     SystemComponentList{cs0, r1, r2, r3, l1, c1, fault});

  Logger::setLogDir("logs/" + simNameRC);
  auto logger = DataLogger::make(simNameRC);
  logger->logAttribute("V2_RC", n2->attribute("v"));
  logger->logAttribute("I_L1_RC", l1->attribute("i_intf"));

  Simulation sim(simNameRC, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doSystemMatrixRecomputation(true);
  sim.addEvent(SwitchEvent::make(startTimeFault, fault, true));
  sim.addEvent(SwitchEvent::make(endTimeFault, fault, false));
  sim.run();
}

void EMT_Ph3_R3_C1_L1_CS1_Fault_SSN() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simNameSSN = "EMT_Ph3_R3C1L1CS1_RC_vs_SSN_Fault_SSN";

  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

  Matrix param = Matrix::Zero(3, 3);
  param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;
  Matrix identity3 = Matrix::Identity(3, 3);

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

  Matrix inductance = 0.02 * param;
  auto l1 = Ph3::GenericTwoTerminalVTypeSSN::make("L1_genVSSN");
  l1->setParameters(Matrix::Zero(3, 3), inductance.inverse(), identity3,
                    Matrix::Zero(3, 3));

  Matrix capacitance = 0.001 * param;
  auto c1 = Ph3::GenericTwoTerminalITypeSSN::make("C1_genISSN");
  c1->setParameters(Matrix::Zero(3, 3), capacitance.inverse(), identity3,
                    Matrix::Zero(3, 3));

  auto fault = Ph3::SeriesSwitch::make("fault");
  fault->setParameters(switchOpenR, switchClosedR);
  fault->open();

  cs0->connect(SimNode::List{n1, SimNode::GND});
  r1->connect(SimNode::List{n2, n1});
  r2->connect(SimNode::List{n2, SimNode::GND});
  r3->connect(SimNode::List{n2, SimNode::GND});
  l1->connect(SimNode::List{n2, SimNode::GND});
  c1->connect(SimNode::List{n1, n2});
  fault->connect(SimNode::List{n2, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2},
                     SystemComponentList{cs0, r1, r2, r3, l1, c1, fault});

  Logger::setLogDir("logs/" + simNameSSN);
  auto logger = DataLogger::make(simNameSSN);
  logger->logAttribute("V2_SSN", n2->attribute("v"));
  logger->logAttribute("I_L1_SSN", l1->attribute("i_intf"));

  Simulation sim(simNameSSN, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doSystemMatrixRecomputation(true);
  sim.addEvent(SwitchEvent::make(startTimeFault, fault, true));
  sim.addEvent(SwitchEvent::make(endTimeFault, fault, false));
  sim.run();
}

int main(int argc, char *argv[]) {
  EMT_Ph3_R3_C1_L1_CS1_Fault_RC();
  EMT_Ph3_R3_C1_L1_CS1_Fault_SSN();
}
