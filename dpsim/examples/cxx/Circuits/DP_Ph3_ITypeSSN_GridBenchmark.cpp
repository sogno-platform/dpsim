// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

using namespace CPS;
using namespace DPsim;

int main(int argc, char *argv[]) {
  constexpr UInt branchCount = 128;
  constexpr Real resistance = 10.0;
  constexpr Real capacitance = 2e-3;
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 0.05;
  const String runName = argc > 1 ? argv[1] : "run";
  const String simName = "DP_Ph3_ITypeSSN_Grid_" + runName;

  const Matrix A = Matrix::Identity(3, 3) * (-1. / (resistance * capacitance));
  const Matrix B = Matrix::Identity(3, 3) * (1. / capacitance);
  const Matrix C = Matrix::Identity(3, 3);
  const Matrix D = Matrix::Zero(3, 3);
  std::vector<SimNode<Complex>::Ptr> nodes;
  std::vector<std::shared_ptr<DP::Ph3::GenericTwoTerminalITypeSSN>> ssnComps;
  SystemComponentList components;
  nodes.reserve(branchCount);
  ssnComps.reserve(branchCount);
  components.reserve(2 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    const Real currentMagnitude = 0.01 + 0.0001 * index;
    auto node =
        SimNode<Complex>::make("bus" + std::to_string(index), PhaseType::ABC);
    auto source =
        DP::Ph3::CurrentSource::make("source" + std::to_string(index));
    source->setParameters(
        Math::singlePhaseVariableToThreePhase(Complex(currentMagnitude, 0.0)));
    source->connect({DP::SimNode::GND, node});

    auto ssn = DP::Ph3::GenericTwoTerminalITypeSSN::make("itype_ssn" +
                                                         std::to_string(index));
    ssn->setParameters(A, B, C, D);
    ssn->connect({node, DP::SimNode::GND});

    nodes.push_back(node);
    ssnComps.push_back(ssn);
    components.push_back(source);
    components.push_back(ssn);
  }

  SystemTopology system(50.0, SystemNodeList(nodes.begin(), nodes.end()),
                        components);
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  for (UInt index : {0u, branchCount / 2, branchCount - 1}) {
    logger->logAttribute("bus" + std::to_string(index) + ".v",
                         nodes[index]->attribute("v"));
    logger->logAttribute("itype_ssn" + std::to_string(index) + ".v",
                         ssnComps[index]->attribute("v_intf"));
    logger->logAttribute("itype_ssn" + std::to_string(index) + ".i",
                         ssnComps[index]->attribute("i_intf"));
  }

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setSolverType(Solver::Type::MNA);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const Real wallTime =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
          .count();

  for (UInt index = 0; index < branchCount; ++index)
    for (Int phase = 0; phase < 3; ++phase)
      if (!Math::isFinite(nodes[index]->voltage()(phase, 0)) ||
          !Math::isFinite(ssnComps[index]->intfVoltage()(phase, 0)) ||
          !Math::isFinite(ssnComps[index]->intfCurrent()(phase, 0))) {
        std::cerr << "DP three-phase I-type SSN grid produced a non-finite "
                     "result.\n";
        return 1;
      }

  std::cout << "branches=" << branchCount << '\n'
            << "steps="
            << static_cast<UInt>(std::llround(finalTime / timeStep)) + 1 << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
