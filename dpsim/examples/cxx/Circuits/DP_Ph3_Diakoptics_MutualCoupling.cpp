// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

static void runCase(const String &name, Bool tearLine) {
  constexpr Real timeStep = 1e-4;
  constexpr Real finalTime = 0.02;

  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

  auto source = Ph3::VoltageSource::make("source");
  source->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0, 0.0)));

  Matrix rLine(3, 3);
  rLine << 0.40, 0.04, 0.02, 0.04, 0.50, 0.03, 0.02, 0.03, 0.45;
  Matrix lLine(3, 3);
  lLine << 4e-3, 4e-4, 2e-4, 4e-4, 5e-3, 3e-4, 2e-4, 3e-4, 4.5e-3;
  Matrix cLine(3, 3);
  cLine << 2e-6, 0.15e-6, 0.10e-6, 0.15e-6, 2.2e-6, 0.12e-6, 0.10e-6, 0.12e-6,
      1.8e-6;
  Matrix gLine = 1e-6 * Matrix::Identity(3, 3);

  auto line = Ph3::PiLine::make("line");
  line->setParameters(rLine, lLine, cLine, gLine);

  Matrix rLoad(3, 3);
  rLoad << 2.0, 0.10, 0.03, 0.10, 2.2, 0.08, 0.03, 0.08, 1.8;
  auto load = Ph3::Resistor::make("load");
  load->setParameters(rLoad);

  source->connect({n1, SimNode::GND});
  line->connect({n1, n2});
  load->connect({n2, SimNode::GND});

  SystemTopology system;
  if (tearLine) {
    // A torn component is registered separately and omitted from the subnet
    // component list, matching the existing Diakoptics examples.
    system = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{source, load});
    system.addTearComponent(line);
  } else {
    system = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{source, line, load});
  }

  Logger::setLogDir("logs/" + name);
  auto logger = DataLogger::make(name);
  logger->logAttribute("v_n2", n2->attribute("v"));
  logger->logAttribute("i_line", line->attribute("i_intf"));

  Simulation sim(name, Logger::Level::info);
  sim.setSystem(system);
  if (tearLine)
    sim.setTearingComponents(system.mTearComponents);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

int main() {
  runCase("DP_Ph3_MutualCoupling_Direct", false);
  runCase("DP_Ph3_MutualCoupling_Torn", true);
}
