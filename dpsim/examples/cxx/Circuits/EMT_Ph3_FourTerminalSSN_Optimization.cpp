// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <iostream>

using namespace DPsim;
using namespace CPS::EMT;

int main(int argc, char *[]) {
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 10.0;
  const String simName = "EMT_Ph3_FourTerminalSSN_Optimization";

  SystemNodeList nodes;
  CPS::SimNode<Real>::List buses;
  SystemComponentList components;
  for (Int terminal = 0; terminal < 4; ++terminal) {
    auto node = CPS::SimNode<Real>::make("bus" + std::to_string(terminal),
                                         CPS::PhaseType::ABC);
    auto source = Ph3::VoltageSource::make("source" + std::to_string(terminal));
    source->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                              Complex(100.0 + 5.0 * terminal, 0.0)),
                          50.0);
    auto shunt = Ph3::Resistor::make("shunt" + std::to_string(terminal));
    shunt->setParameters(CPS::Math::singlePhaseParameterToThreePhase(100.0));
    source->connect({CPS::SimNode<Real>::GND, node});
    shunt->connect({node, CPS::SimNode<Real>::GND});
    nodes.push_back(node);
    buses.push_back(node);
    components.push_back(source);
    components.push_back(shunt);
  }

  auto ssn = Ph3::GenericFourTerminalVTypeSSN::make("four_terminal_ssn");
  Matrix A = Matrix::Zero(2, 2);
  A.diagonal() << -50.0, -80.0;
  Matrix B = Matrix::Constant(2, 6, 0.01);
  Matrix C = Matrix::Constant(6, 2, 0.02);
  Matrix D = 0.1 * Matrix::Identity(6, 6);
  ssn->setParameters(A, B, C, D);
  ssn->connect({buses[0], buses[1], buses[2], buses[3]});
  components.push_back(ssn);

  auto system = SystemTopology(50, nodes, components);
  std::shared_ptr<DataLoggerInterface> logger;
  if (argc == 1) {
    Logger::setLogDir("logs/" + simName);
    auto dataLogger = DataLogger::make(simName);
    for (std::size_t index = 0; index < nodes.size(); ++index)
      dataLogger->logAttribute("bus" + std::to_string(index),
                               nodes[index]->attribute("v"));
    dataLogger->logAttribute("ssn_voltage", ssn->attribute("v_intf"));
    dataLogger->logAttribute("ssn_current", ssn->attribute("i_intf"));
    dataLogger->logAttribute("ssn_state", ssn->attribute("x"));
    logger = dataLogger;
  }

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  if (logger)
    simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  std::cout << "simulation_wall_s=" << elapsed << '\n';
  return 0;
}
