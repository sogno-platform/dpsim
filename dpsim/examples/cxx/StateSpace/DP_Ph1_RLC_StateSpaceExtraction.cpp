// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#include <DPsim.h>

#include <cmath>
#include <iostream>

using namespace DPsim;
using namespace CPS::DP;

namespace {

void printEigenvalues(const String &title, const CPS::VectorComp &eigenvalues) {
  std::cout << "\n" << title << "\n";
  for (Eigen::Index idx = 0; idx < eigenvalues.rows(); ++idx)
    std::cout << "  [" << idx << "] " << eigenvalues(idx) << "\n";
}

Complex mapContinuousToDiscrete(const Complex &lambda, Real timeStep) {
  const Complex two(2.0, 0.0);
  return (two + timeStep * lambda) / (two - timeStep * lambda);
}

} // namespace

class DPPh1RLCStateSpaceExtractionExample {
public:
  DPPh1RLCStateSpaceExtractionExample()
      : mTimeStep(0.0001), mFinalTime(0.001), mFrequency(50.0),
        mSourceVoltage(CPS::Math::polar(1.0, 0.0)), mBranchResistance(0.2),
        mBranchInductance(0.02), mLoadResistance(1.0), mLoadInductance(0.05),
        mLoadCapacitance(100e-6) {}

  void runExampleWithMNAComponents() const {
    const String simName = "DP_Ph1_RLC_StateSpaceExtraction_Native";

    auto n1 = SimNode::make("n1");
    auto n2 = SimNode::make("n2");
    auto n3 = SimNode::make("n3");
    auto n4 = SimNode::make("n4");
    auto n5 = SimNode::make("n5");

    auto vs = Ph1::VoltageSource::make("VS");
    vs->setParameters(mSourceVoltage, mFrequency);

    auto rBranch = Ph1::Resistor::make("RBranch");
    rBranch->setParameters(mBranchResistance);

    auto lBranch = Ph1::Inductor::make("LBranch");
    lBranch->setParameters(mBranchInductance);

    auto rLoad = Ph1::Resistor::make("RLoad");
    rLoad->setParameters(mLoadResistance);

    auto lLoad = Ph1::Inductor::make("LLoad");
    lLoad->setParameters(mLoadInductance);

    auto cLoad = Ph1::Capacitor::make("CLoad");
    cLoad->setParameters(mLoadCapacitance);

    vs->connect(SimNode::List{SimNode::GND, n1});
    rBranch->connect(SimNode::List{n1, n2});
    lBranch->connect(SimNode::List{n2, n3});
    rLoad->connect(SimNode::List{n3, n4});
    lLoad->connect(SimNode::List{n4, n5});
    cLoad->connect(SimNode::List{n5, SimNode::GND});

    auto system = SystemTopology(
        mFrequency, SystemNodeList{n1, n2, n3, n4, n5},
        SystemComponentList{vs, rBranch, lBranch, rLoad, lLoad, cLoad});

    Simulation sim(simName, Logger::Level::warn);
    sim.setSystem(system);
    sim.setDomain(Domain::DP);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.doStateSpaceExtraction(true);
    sim.run();

    printExtractionResults("Native DP Ph1 components: RL branch + RLC load",
                           sim);
  }

  void runExampleWithRLCLoadSSN() const {
    const String simName = "DP_Ph1_RLC_StateSpaceExtraction_SSNLoad";

    auto n1 = SimNode::make("n1");
    auto n2 = SimNode::make("n2");
    auto n3 = SimNode::make("n3");

    Matrix aMatrix;
    Matrix bMatrix;
    Matrix cMatrix;
    Matrix dMatrix;
    createStateSpaceMatricesForRLCLoad(aMatrix, bMatrix, cMatrix, dMatrix);

    auto vs = Ph1::VoltageSource::make("VS");
    vs->setParameters(mSourceVoltage, mFrequency);

    auto rBranch = Ph1::Resistor::make("RBranch");
    rBranch->setParameters(mBranchResistance);

    auto lBranch = Ph1::Inductor::make("LBranch");
    lBranch->setParameters(mBranchInductance);

    auto rlcLoad =
        Ph1::GenericTwoTerminalVTypeSSN::make("RLCLoad", Logger::Level::warn);
    rlcLoad->setParameters(aMatrix, bMatrix, cMatrix, dMatrix);

    vs->connect(SimNode::List{SimNode::GND, n1});
    rBranch->connect(SimNode::List{n1, n2});
    lBranch->connect(SimNode::List{n2, n3});
    rlcLoad->connect(SimNode::List{n3, SimNode::GND});

    auto system =
        SystemTopology(mFrequency, SystemNodeList{n1, n2, n3},
                       SystemComponentList{vs, rBranch, lBranch, rlcLoad});

    Simulation sim(simName, Logger::Level::warn);
    sim.setSystem(system);
    sim.setDomain(Domain::DP);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.doStateSpaceExtraction(true);
    sim.run();

    printExtractionResults("Mixed model: native RL branch + DP SSN RLC load",
                           sim);
  }

  void runExampleWithRLBranchSSN() const {
    const String simName = "DP_Ph1_RLC_StateSpaceExtraction_SSNBranch";

    auto n1 = SimNode::make("n1");
    auto n3 = SimNode::make("n3");
    auto n4 = SimNode::make("n4");
    auto n5 = SimNode::make("n5");

    Matrix aMatrix;
    Matrix bMatrix;
    Matrix cMatrix;
    Matrix dMatrix;
    createStateSpaceMatricesForRLBranch(aMatrix, bMatrix, cMatrix, dMatrix);

    auto vs = Ph1::VoltageSource::make("VS");
    vs->setParameters(mSourceVoltage, mFrequency);

    auto rlBranch =
        Ph1::GenericTwoTerminalVTypeSSN::make("RLBranch", Logger::Level::warn);
    rlBranch->setParameters(aMatrix, bMatrix, cMatrix, dMatrix);

    auto rLoad = Ph1::Resistor::make("RLoad");
    rLoad->setParameters(mLoadResistance);

    auto lLoad = Ph1::Inductor::make("LLoad");
    lLoad->setParameters(mLoadInductance);

    auto cLoad = Ph1::Capacitor::make("CLoad");
    cLoad->setParameters(mLoadCapacitance);

    vs->connect(SimNode::List{SimNode::GND, n1});
    rlBranch->connect(SimNode::List{n1, n3});
    rLoad->connect(SimNode::List{n3, n4});
    lLoad->connect(SimNode::List{n4, n5});
    cLoad->connect(SimNode::List{n5, SimNode::GND});

    auto system =
        SystemTopology(mFrequency, SystemNodeList{n1, n3, n4, n5},
                       SystemComponentList{vs, rlBranch, rLoad, lLoad, cLoad});

    Simulation sim(simName, Logger::Level::warn);
    sim.setSystem(system);
    sim.setDomain(Domain::DP);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.doStateSpaceExtraction(true);
    sim.run();

    printExtractionResults("Mixed model: DP SSN RL branch + native RLC load",
                           sim);
  }

  void printExpectedEigenvalues() const {
    const Real totalResistance = mBranchResistance + mLoadResistance;
    const Real totalInductance = mBranchInductance + mLoadInductance;
    const Real alpha = totalResistance / (2.0 * totalInductance);
    const Real omega0 = std::sqrt(1.0 / (totalInductance * mLoadCapacitance));
    const Real omegaD = std::sqrt(omega0 * omega0 - alpha * alpha);
    const Real omegaShift = 2.0 * PI * mFrequency;

    const Complex lambdaPhysical1(-alpha, omegaD);
    const Complex lambdaPhysical2(-alpha, -omegaD);

    const Complex lambdaDP1 = lambdaPhysical1 - Complex(0.0, omegaShift);
    const Complex lambdaDP2 = lambdaPhysical2 - Complex(0.0, omegaShift);

    const Complex z1 = mapContinuousToDiscrete(lambdaDP1, mTimeStep);
    const Complex z2 = mapContinuousToDiscrete(lambdaDP2, mTimeStep);
    const Complex redundantZ(-1.0, 0.0);

    std::cout
        << "\n============================================================\n";
    std::cout << "Expected eigenvalues for the native DP Ph1 model\n";
    std::cout
        << "============================================================\n";
    std::cout
        << "\nPhysical continuous-time modes before frequency shifting:\n";
    std::cout << "  lambda_1 = " << lambdaPhysical1 << "\n";
    std::cout << "  lambda_2 = " << lambdaPhysical2 << "\n";
    std::cout << "\nNative DP continuous-time modes in the shifted frame:\n";
    std::cout << "  lambda_DP_1 = " << lambdaDP1 << "\n";
    std::cout << "  lambda_DP_2 = " << lambdaDP2 << "\n";
    std::cout << "\nCorresponding trapezoidal discrete-time modes:\n";
    std::cout << "  z_1 = " << z1 << "\n";
    std::cout << "  z_2 = " << z2 << "\n";
    std::cout << "\nThe real-valued augmented extraction also contains the "
                 "conjugates "
                 "of these modes.\n";
    std::cout << "\nAdditional redundant trapezoidal mode:\n";
    std::cout << "  z_3 = " << redundantZ << "\n";
    std::cout << "\nThe redundant z = -1 mode maps to infinity under the "
                 "trapezoidal z-to-s mapping.\n";
  }

private:
  void printExtractionResults(const String &caseName,
                              const Simulation &sim) const {
    const auto &extractor = sim.getStateSpaceExtractor();
    StateSpaceModalAnalysis modalAnalysis(extractor);
    modalAnalysis.update();

    std::cout
        << "\n============================================================\n";
    std::cout << caseName << "\n";
    std::cout
        << "============================================================\n";
    std::cout << "Number of extraction states: " << extractor.getStateCount()
              << "\n";
    printEigenvalues("Extracted discrete-time eigenvalues z:",
                     modalAnalysis.getDiscreteEigenvalues());
    printEigenvalues("Extracted continuous-time native DP eigenvalues lambda:",
                     modalAnalysis.getContinuousEigenvalues());
  }

  void createStateSpaceMatricesForRLCLoad(Matrix &aMatrix, Matrix &bMatrix,
                                          Matrix &cMatrix,
                                          Matrix &dMatrix) const {
    // x = [uC; i], input = v, output = i.
    aMatrix = Matrix::Zero(2, 2);
    aMatrix(0, 1) = 1.0 / mLoadCapacitance;
    aMatrix(1, 0) = -1.0 / mLoadInductance;
    aMatrix(1, 1) = -mLoadResistance / mLoadInductance;

    bMatrix = Matrix::Zero(2, 1);
    bMatrix(1, 0) = 1.0 / mLoadInductance;

    cMatrix = Matrix::Zero(1, 2);
    cMatrix(0, 1) = 1.0;

    dMatrix = Matrix::Zero(1, 1);
  }

  void createStateSpaceMatricesForRLBranch(Matrix &aMatrix, Matrix &bMatrix,
                                           Matrix &cMatrix,
                                           Matrix &dMatrix) const {
    // x = i, input = v, output = i.
    aMatrix = Matrix::Zero(1, 1);
    aMatrix(0, 0) = -mBranchResistance / mBranchInductance;

    bMatrix = Matrix::Zero(1, 1);
    bMatrix(0, 0) = 1.0 / mBranchInductance;

    cMatrix = Matrix::Identity(1, 1);
    dMatrix = Matrix::Zero(1, 1);
  }

  Real mTimeStep;
  Real mFinalTime;
  Real mFrequency;
  Complex mSourceVoltage;
  Real mBranchResistance;
  Real mBranchInductance;
  Real mLoadResistance;
  Real mLoadInductance;
  Real mLoadCapacitance;
};

int main() {
  DPPh1RLCStateSpaceExtractionExample example;
  example.runExampleWithMNAComponents();
  example.runExampleWithRLCLoadSSN();
  example.runExampleWithRLBranchSSN();
  example.printExpectedEigenvalues();
  return 0;
}
