#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;

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

class EMTPh3RLCStateSpaceExtractionExample {
public:
  EMTPh3RLCStateSpaceExtractionExample()
      : mTimeStep(0.0001), mFinalTime(0.001), mFrequency(50.0),
        mSourceVoltage(CPS::Math::polar(1.0, 0.0)),
        mBranchResistance(CPS::Math::singlePhaseParameterToThreePhase(0.2)),
        mBranchInductance(CPS::Math::singlePhaseParameterToThreePhase(0.02)),
        mLoadResistance(CPS::Math::singlePhaseParameterToThreePhase(1.0)),
        mLoadInductance(CPS::Math::singlePhaseParameterToThreePhase(0.05)),
        mLoadCapacitance(CPS::Math::singlePhaseParameterToThreePhase(100e-6)) {}

  void runExampleWithMNAComponents() const {
    const String simName = "EMT_Ph3_RLC_StateSpaceExtraction_Native";

    auto n1 = SimNode::make("n1", PhaseType::ABC);
    auto n2 = SimNode::make("n2", PhaseType::ABC);
    auto n3 = SimNode::make("n3", PhaseType::ABC);
    auto n4 = SimNode::make("n4", PhaseType::ABC);
    auto n5 = SimNode::make("n5", PhaseType::ABC);

    auto vs = Ph3::VoltageSource::make("VS");
    vs->setParameters(
        CPS::Math::singlePhaseVariableToThreePhase(mSourceVoltage), mFrequency);

    auto rBranch = Ph3::Resistor::make("RBranch");
    rBranch->setParameters(mBranchResistance);

    auto lBranch = Ph3::Inductor::make("LBranch");
    lBranch->setParameters(mBranchInductance);

    auto rLoad = Ph3::Resistor::make("RLoad");
    rLoad->setParameters(mLoadResistance);

    auto lLoad = Ph3::Inductor::make("LLoad");
    lLoad->setParameters(mLoadInductance);

    auto cLoad = Ph3::Capacitor::make("CLoad");
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
    sim.setDomain(Domain::EMT);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.doStateSpaceExtraction(true);

    sim.run();

    printExtractionResults("Native EMT components: RL branch + RLC load", sim);
  }

  void runExampleWithRLCLoadSSN() const {
    const String simName = "EMT_Ph3_RLC_StateSpaceExtraction_SSNLoad";

    auto n1 = SimNode::make("n1", PhaseType::ABC);
    auto n2 = SimNode::make("n2", PhaseType::ABC);
    auto n3 = SimNode::make("n3", PhaseType::ABC);

    Matrix aMatrix;
    Matrix bMatrix;
    Matrix cMatrix;
    Matrix dMatrix;
    createStateSpaceMatricesForRLCLoad(aMatrix, bMatrix, cMatrix, dMatrix);

    auto vs = Ph3::VoltageSource::make("VS");
    vs->setParameters(
        CPS::Math::singlePhaseVariableToThreePhase(mSourceVoltage), mFrequency);

    auto rBranch = Ph3::Resistor::make("RBranch");
    rBranch->setParameters(mBranchResistance);

    auto lBranch = Ph3::Inductor::make("LBranch");
    lBranch->setParameters(mBranchInductance);

    auto rlcLoad =
        Ph3::GenericTwoTerminalVTypeSSN::make("RLCLoad", Logger::Level::warn);
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
    sim.setDomain(Domain::EMT);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.doStateSpaceExtraction(true);

    sim.run();

    printExtractionResults("Mixed model: native RL branch + SSN RLC load", sim);
  }

  void runExampleWithRLBranchSSN() const {
    const String simName = "EMT_Ph3_RLC_StateSpaceExtraction_SSNBranch";

    auto n1 = SimNode::make("n1", PhaseType::ABC);
    auto n3 = SimNode::make("n3", PhaseType::ABC);
    auto n4 = SimNode::make("n4", PhaseType::ABC);
    auto n5 = SimNode::make("n5", PhaseType::ABC);

    Matrix aMatrix;
    Matrix bMatrix;
    Matrix cMatrix;
    Matrix dMatrix;
    createStateSpaceMatricesForRLBranch(aMatrix, bMatrix, cMatrix, dMatrix);

    auto vs = Ph3::VoltageSource::make("VS");
    vs->setParameters(
        CPS::Math::singlePhaseVariableToThreePhase(mSourceVoltage), mFrequency);

    auto rlBranch =
        Ph3::GenericTwoTerminalVTypeSSN::make("RLBranch", Logger::Level::warn);
    rlBranch->setParameters(aMatrix, bMatrix, cMatrix, dMatrix);

    auto rLoad = Ph3::Resistor::make("RLoad");
    rLoad->setParameters(mLoadResistance);

    auto lLoad = Ph3::Inductor::make("LLoad");
    lLoad->setParameters(mLoadInductance);

    auto cLoad = Ph3::Capacitor::make("CLoad");
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
    sim.setDomain(Domain::EMT);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.doStateSpaceExtraction(true);

    sim.run();

    printExtractionResults("Mixed model: SSN RL branch + native RLC load", sim);
  }

  void printExpectedEigenvalues() const {
    const Real totalResistance =
        mBranchResistance(0, 0) + mLoadResistance(0, 0);
    const Real totalInductance =
        mBranchInductance(0, 0) + mLoadInductance(0, 0);
    const Real loadCapacitance = mLoadCapacitance(0, 0);

    const Real alpha = totalResistance / (2.0 * totalInductance);
    const Real omega0 = std::sqrt(1.0 / (totalInductance * loadCapacitance));
    const Real omegaD = std::sqrt(omega0 * omega0 - alpha * alpha);

    const Complex lambda1(-alpha, omegaD);
    const Complex lambda2(-alpha, -omegaD);

    const Complex z1 = mapContinuousToDiscrete(lambda1, mTimeStep);
    const Complex z2 = mapContinuousToDiscrete(lambda2, mTimeStep);
    const Complex redundantZ(-1.0, 0.0);

    std::cout
        << "\n============================================================\n";
    std::cout << "Expected eigenvalues per phase\n";
    std::cout
        << "============================================================\n";

    std::cout << "\nPhysical continuous-time modes:\n";
    std::cout << "  lambda_1 = " << lambda1 << "\n";
    std::cout << "  lambda_2 = " << lambda2 << "\n";

    std::cout << "\nCorresponding trapezoidal discrete-time modes:\n";
    std::cout << "  z_1 = " << z1 << "\n";
    std::cout << "  z_2 = " << z2 << "\n";

    std::cout << "\nAdditional redundant trapezoidal mode:\n";
    std::cout << "  z_3 = " << redundantZ << "\n";

    std::cout << "\nEach mode is expected three times for the balanced EMT Ph3 "
                 "system.\n";
    std::cout << "The redundant z = -1 mode maps to infinity under the "
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

    printEigenvalues("Extracted continuous-time eigenvalues lambda:",
                     modalAnalysis.getContinuousEigenvalues());
  }

  void createStateSpaceMatricesForRLCLoad(Matrix &aMatrix, Matrix &bMatrix,
                                          Matrix &cMatrix,
                                          Matrix &dMatrix) const {
    const Matrix inverseInductance = mLoadInductance.inverse();
    const Matrix inverseCapacitance = mLoadCapacitance.inverse();

    // x = [uC_abc; i_abc], input = v_abc, output = i_abc.
    aMatrix = Matrix::Zero(6, 6);
    aMatrix.block(0, 3, 3, 3) = inverseCapacitance;
    aMatrix.block(3, 0, 3, 3) = -inverseInductance;
    aMatrix.block(3, 3, 3, 3) = -inverseInductance * mLoadResistance;

    bMatrix = Matrix::Zero(6, 3);
    bMatrix.block(3, 0, 3, 3) = inverseInductance;

    cMatrix = Matrix::Zero(3, 6);
    cMatrix.block(0, 3, 3, 3) = Matrix::Identity(3, 3);

    dMatrix = Matrix::Zero(3, 3);
  }

  void createStateSpaceMatricesForRLBranch(Matrix &aMatrix, Matrix &bMatrix,
                                           Matrix &cMatrix,
                                           Matrix &dMatrix) const {
    const Matrix inverseInductance = mBranchInductance.inverse();

    // x = i_abc, input = v_abc, output = i_abc.
    aMatrix = -inverseInductance * mBranchResistance;
    bMatrix = inverseInductance;
    cMatrix = Matrix::Identity(3, 3);
    dMatrix = Matrix::Zero(3, 3);
  }

  Real mTimeStep;
  Real mFinalTime;
  Real mFrequency;
  Complex mSourceVoltage;

  Matrix mBranchResistance;
  Matrix mBranchInductance;

  Matrix mLoadResistance;
  Matrix mLoadInductance;
  Matrix mLoadCapacitance;
};

int main() {
  EMTPh3RLCStateSpaceExtractionExample example;

  example.runExampleWithMNAComponents();
  example.runExampleWithRLCLoadSSN();
  example.runExampleWithRLBranchSSN();

  example.printExpectedEigenvalues();

  return 0;
}
