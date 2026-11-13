// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

namespace {
constexpr Int stateCount = 48;
constexpr Real stepSize = 10e-6;
constexpr Real finalTime = 0.5;

void makeModelMatrices(Matrix &A, Matrix &B, Matrix &C, Matrix &D, Bool iType) {
  A = Matrix::Zero(stateCount, stateCount);
  B = Matrix::Zero(stateCount, 1);
  C = Matrix::Zero(1, stateCount);
  D = Matrix::Zero(1, 1);

  for (Int row = 0; row < stateCount; ++row) {
    A(row, row) = -2.0 - 0.02 * row;
    B(row, 0) = 0.1 * (row + 1) / stateCount;
    C(0, row) = 0.1 * (row + 1) / stateCount;
    for (Int col = 0; col < stateCount; ++col) {
      if (row != col)
        A(row, col) = 1e-4 * std::cos(Real(row + 1) * Real(col + 1));
    }
  }

  // Keep both Norton/Thevenin equivalents well-conditioned and dissipative.
  D(0, 0) = iType ? 1.0 : 0.05;
}

int runCase(Bool iType, Bool changeC) {
  const String kind = iType ? "IType" : "VType";
  const String updateSuffix = changeC ? "_ChangeC" : "";
  const String simName = "EMT_Ph1_LargeStateSSNHistory_" + kind + updateSuffix;
  Logger::setLogDir("logs/" + simName);

  auto sourceNode = CPS::SimNode<Real>::make("source", PhaseType::Single);
  auto loadNode = CPS::SimNode<Real>::make("load", PhaseType::Single);
  auto source = EMT::Ph1::VoltageSource::make("source");
  source->setParameters(Math::polar(1.0, Math::degToRad(-90.0)), 50.0);
  auto load = EMT::Ph1::Resistor::make("load");
  load->setParameters(10.0);

  SimPowerComp<Real>::Ptr ssn;
  Attribute<Matrix>::Ptr outputMatrix;
  Matrix A, B, C, D;
  makeModelMatrices(A, B, C, D, iType);
  if (iType) {
    auto component = EMT::Ph1::SSNTypeI2T::make("large_state_ssn");
    component->setParameters(A, B, C, D);
    outputMatrix = component->mC;
    ssn = component;
  } else {
    auto component = EMT::Ph1::SSNTypeV2T::make("large_state_ssn");
    component->setParameters(A, B, C, D);
    outputMatrix = component->mC;
    ssn = component;
  }

  source->connect({CPS::SimNode<Real>::GND, sourceNode});
  ssn->connect({sourceNode, loadNode});
  load->connect({loadNode, CPS::SimNode<Real>::GND});
  auto system =
      SystemTopology(50.0, {sourceNode, loadNode}, {source, ssn, load});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_ssn", ssn->attribute("v_intf"));
  logger->logAttribute("i_ssn", ssn->attribute("i_intf"));

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(stepSize);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  if (!changeC) {
    simulation.run();
  } else {
    simulation.initialize();
    simulation.start();
    Bool matrixChanged = false;
    while (simulation.time() < simulation.finalTime()) {
      if (!matrixChanged && simulation.time() >= 0.25 - 0.5 * stepSize) {
        **outputMatrix *= 1.5;
        matrixChanged = true;
      }
      simulation.step();
    }
    simulation.stop();
  }
  return 0;
}
} // namespace

int main(int argc, char *argv[]) {
  const Bool iType = argc > 1 && String(argv[1]) == "i-type";
  const Bool changeC = argc > 2 && String(argv[2]) == "change-c";
  return runCase(iType, changeC);
}
