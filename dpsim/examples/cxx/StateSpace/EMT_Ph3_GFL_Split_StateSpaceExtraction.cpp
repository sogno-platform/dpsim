// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <dpsim-models/EMT/EMT_Ph3_SSN_GFL.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_GFL_Split.h>

using namespace CPS;
using namespace DPsim;

namespace {

constexpr UInt ControllerStateCount = 8;
constexpr UInt NetworkStateCount = 6;
constexpr UInt GridStateCount = 3;
constexpr UInt DelayStateCount = 3;
constexpr UInt DelayedStateCount =
    ControllerStateCount + NetworkStateCount + GridStateCount + DelayStateCount;
constexpr UInt NoDelayStateCount =
    ControllerStateCount + NetworkStateCount + GridStateCount;

struct ExtractionResult {
  VectorComp discreteEigenvalues;
  Real extractionTime;
  UInt stateCount;
};

struct EigenvalueRecord {
  Real timeStep;
  String model;
  String representation;
  VectorComp values;
};

VectorComp eigenvalues(const Matrix &matrix) {
  Eigen::EigenSolver<Matrix> solver(matrix);
  if (solver.info() != Eigen::Success)
    throw std::runtime_error("Eigenvalue computation failed.");
  return solver.eigenvalues();
}

Complex mapContinuousToDiscrete(const Complex &lambda, Real timeStep) {
  const Complex two(2.0, 0.0);
  return (two + timeStep * lambda) / (two - timeStep * lambda);
}

VectorComp mapContinuousToDiscrete(const VectorComp &lambda, Real timeStep) {
  VectorComp result(lambda.rows());
  for (Eigen::Index idx = 0; idx < lambda.rows(); ++idx)
    result(idx) = mapContinuousToDiscrete(lambda(idx), timeStep);
  return result;
}

Real maxNearestDistance(const VectorComp &reference, const VectorComp &actual) {
  Real maximum = 0.0;

  for (Eigen::Index refIdx = 0; refIdx < reference.rows(); ++refIdx) {
    Real nearest = std::numeric_limits<Real>::max();
    for (Eigen::Index actualIdx = 0; actualIdx < actual.rows(); ++actualIdx)
      nearest =
          std::min(nearest, std::abs(reference(refIdx) - actual(actualIdx)));
    maximum = std::max(maximum, nearest);
  }

  return maximum;
}

void appendEigenvaluesToCsv(std::ofstream &stream,
                            const EigenvalueRecord &record) {
  for (Eigen::Index idx = 0; idx < record.values.rows(); ++idx) {
    stream << record.timeStep << ',' << 1e6 * record.timeStep << ','
           << record.model << ',' << record.representation << ',' << idx << ','
           << record.values(idx).real() << ',' << record.values(idx).imag()
           << '\n';
  }
}

void writeEigenvaluesCsv(const std::filesystem::path &path,
                         const std::vector<EigenvalueRecord> &records) {
  std::filesystem::create_directories(path.parent_path());

  std::ofstream stream(path);
  if (!stream.is_open())
    throw std::runtime_error("Could not open eigenvalue CSV output: " +
                             path.string());

  stream << std::setprecision(std::numeric_limits<Real>::max_digits10);
  stream << "time_step_s,time_step_us,model,representation,index,real,imag\n";

  for (const auto &record : records)
    appendEigenvaluesToCsv(stream, record);

  if (!stream)
    throw std::runtime_error("Could not write eigenvalue CSV output: " +
                             path.string());
}

Matrix parkTransformDq0(Real theta) {
  const Real k = std::sqrt(2.0 / 3.0);
  const Real k0 = 1.0 / std::sqrt(3.0);

  Matrix transform(3, 3);
  transform.row(0) << k * std::cos(theta), k * std::cos(theta - 2.0 * PI / 3.0),
      k * std::cos(theta + 2.0 * PI / 3.0);
  transform.row(1) << -k * std::sin(theta),
      -k * std::sin(theta - 2.0 * PI / 3.0),
      -k * std::sin(theta + 2.0 * PI / 3.0);
  transform.row(2) << k0, k0, k0;
  return transform;
}

void stampParkBlock(Matrix &transform, UInt offset, Real theta) {
  transform.block(offset, offset, 3, 3) = parkTransformDq0(theta);
}

void stampFrameDerivative(Matrix &matrix, UInt offset, Real omega) {
  matrix(offset, offset + 1) += omega;
  matrix(offset + 1, offset) -= omega;
}

} // namespace

class EMTPh3SplitGFLStateSpaceExtractionExample {
public:
  EMTPh3SplitGFLStateSpaceExtractionExample()
      : mFrequency(50.0), mOmega(2.0 * PI * mFrequency),
        mGridVoltageRmsLineToLine(400.0), mGridResistance(0.3),
        mGridInductance(0.1e-3), mLf(2e-3), mCf(10e-6), mRf(0.2), mRc(0.2),
        mKpPLL(0.25), mKiPLL(0.2), mOmegaCutoff(mOmega), mPRef(10000.0),
        mQRef(5000.0), mKpPowerCtrl(0.05), mKiPowerCtrl(0.2), mKpCurrCtrl(0.25),
        mKiCurrCtrl(1.0) {}

  void run() const {
    const String simName = "EMT_Ph3_GFL_Split_StateSpaceExtraction";
    const SystemTopology powerFlow = runPowerFlow(simName + "_PF");

    const std::array<Real, 4> timeSteps{1e-6, 10e-6, 100e-6, 1e-3};
    const std::array<String, 4> timeStepLabels{"1us", "10us", "100us", "1ms"};
    std::vector<EigenvalueRecord> records;

    std::cout
        << "\n============================================================\n"
        << "EMT Ph3 GFL state-space extraction comparison\n"
        << "============================================================\n"
        << "Topology: ideal source -> R/L grid -> SSN GFL\n"
        << "Analysis frame: global dq0\n"
        << "Time steps: 1 us, 10 us, 100 us, 1 ms\n";

    for (UInt caseIdx = 0; caseIdx < timeSteps.size(); ++caseIdx) {
      const Real timeStep = timeSteps[caseIdx];
      const String suffix = "_dt_" + timeStepLabels[caseIdx];

      auto splitInverter =
          EMT::Ph3::SSN_GFL_Split::make("InverterSplit", Logger::Level::warn);
      setInverterParameters(splitInverter);
      const ExtractionResult extractedSplit =
          runExtraction(powerFlow, splitInverter, simName + "_Split" + suffix,
                        timeStep, DelayedStateCount);

      auto noDelayInverter =
          EMT::Ph3::SSN_GFL::make("InverterNoDelay", Logger::Level::warn);
      setInverterParameters(noDelayInverter);
      const ExtractionResult extractedNoDelay = runExtraction(
          powerFlow, noDelayInverter, simName + "_NoDelay" + suffix, timeStep,
          NoDelayStateCount);

      const Matrix analyticalDelayedAd = buildAnalyticalDelayedDiscreteMatrix(
          *splitInverter, extractedSplit.extractionTime, timeStep);
      const VectorComp analyticalDelayedZ = eigenvalues(analyticalDelayedAd);

      const Matrix analyticalNoDelayA = buildAnalyticalNoDelayContinuousMatrix(
          *splitInverter, extractedSplit.extractionTime + timeStep);
      const VectorComp analyticalNoDelayLambda =
          eigenvalues(analyticalNoDelayA);
      const VectorComp analyticalNoDelayZ =
          mapContinuousToDiscrete(analyticalNoDelayLambda, timeStep);

      records.push_back({timeStep, "extracted_ssn_gfl_split", "discrete_z",
                         extractedSplit.discreteEigenvalues});
      records.push_back({timeStep, "extracted_ssn_gfl", "discrete_z",
                         extractedNoDelay.discreteEigenvalues});
      records.push_back(
          {timeStep, "analytical_delayed", "discrete_z", analyticalDelayedZ});
      records.push_back(
          {timeStep, "analytical_no_delay", "discrete_z", analyticalNoDelayZ});
      records.push_back({timeStep, "analytical_no_delay", "continuous_exact",
                         analyticalNoDelayLambda});

      const Real delayedDifference =
          std::max(maxNearestDistance(analyticalDelayedZ,
                                      extractedSplit.discreteEigenvalues),
                   maxNearestDistance(extractedSplit.discreteEigenvalues,
                                      analyticalDelayedZ));
      const Real noDelayDifference =
          std::max(maxNearestDistance(analyticalNoDelayZ,
                                      extractedNoDelay.discreteEigenvalues),
                   maxNearestDistance(extractedNoDelay.discreteEigenvalues,
                                      analyticalNoDelayZ));

      std::cout << "\nDelta t = " << 1e6 * timeStep << " us\n"
                << "  split states: " << extractedSplit.stateCount
                << " (8 controller + 6 plant + 3 grid + 3 delay)\n"
                << "  no-delay states: " << extractedNoDelay.stateCount
                << " (8 controller + 6 plant + 3 grid)\n"
                << "  split extraction vs analytical delayed max error: "
                << delayedDifference << "\n"
                << "  no-delay extraction vs analytical no-delay max error: "
                << noDelayDifference << "\n";

      if (delayedDifference > 1e-7)
        throw std::runtime_error(
            "Split-GFL extraction does not match the analytical delayed "
            "model at time step " +
            timeStepLabels[caseIdx] + ".");
    }

    const std::filesystem::path eigenvalueCsvPath =
        std::filesystem::path("logs") / simName / "eigenvalues.csv";
    writeEigenvaluesCsv(eigenvalueCsvPath, records);
    std::cout << "\nEigenvalue CSV: " << eigenvalueCsvPath.string() << "\n";
  }

private:
  template <typename InverterType>
  void
  setInverterParameters(const std::shared_ptr<InverterType> &inverter) const {
    inverter->setParameters(mLf, mCf, mRf, mRc, mOmega, mKpPLL, mKiPLL,
                            mOmegaCutoff, mPRef, mQRef, mKpPowerCtrl,
                            mKiPowerCtrl, mKpCurrCtrl, mKiCurrCtrl);
  }

  template <typename InverterType>
  ExtractionResult runExtraction(const SystemTopology &powerFlow,
                                 const std::shared_ptr<InverterType> &inverter,
                                 const String &simName, Real timeStep,
                                 UInt expectedStateCount) const {
    Simulation simulation(simName, Logger::Level::warn);
    simulation.setSystem(createEmtSystem(powerFlow, inverter));
    simulation.setDomain(Domain::EMT);
    simulation.setSolverType(Solver::Type::MNA);
    simulation.setTimeStep(timeStep);
    // Extract the first-step map at the power-flow initialized operating point.
    // This keeps the delayed and no-delay cases comparable across time steps.
    simulation.setFinalTime(timeStep);
    simulation.doStateSpaceExtraction(true);
    simulation.doInitFromNodesAndTerminals(true);
    simulation.run();

    const auto &extractor = simulation.getStateSpaceExtractor();
    if (extractor.getStateCount() != expectedStateCount)
      throw std::runtime_error("Unexpected extracted state count in " +
                               simName + ".");

    StateSpaceModalAnalysis modalAnalysis(extractor);
    modalAnalysis.setAnalysisFrame(StateSpaceAnalysisFrame::GlobalDQ0);
    modalAnalysis.setGlobalDq0Frame(mOmega);
    modalAnalysis.update();

    return {modalAnalysis.getDiscreteEigenvalues(),
            extractor.getLastExtractionTime(), extractor.getStateCount()};
  }

  SystemTopology runPowerFlow(const String &simName) const {
    auto nGrid = SimNode<Complex>::make("nGrid", PhaseType::Single);
    auto nSeries = SimNode<Complex>::make("nSeries", PhaseType::Single);
    auto nPcc = SimNode<Complex>::make("nPcc", PhaseType::Single);

    auto slack = SP::Ph1::NetworkInjection::make("Slack");
    slack->setParameters(mGridVoltageRmsLineToLine);
    slack->setBaseVoltage(mGridVoltageRmsLineToLine);
    slack->modifyPowerFlowBusType(PowerflowBusType::VD);

    auto resistance = SP::Ph1::PiLine::make("GridResistancePF");
    resistance->setParameters(mGridResistance, 0.0, 0.0, 0.0);
    resistance->setBaseVoltage(mGridVoltageRmsLineToLine);

    auto inductance = SP::Ph1::PiLine::make("GridInductancePF");
    inductance->setParameters(0.0, mGridInductance, 0.0, 0.0);
    inductance->setBaseVoltage(mGridVoltageRmsLineToLine);

    auto inverterInjection = SP::Ph1::Load::make("InverterPF");
    inverterInjection->setParameters(-mPRef, -mQRef, mGridVoltageRmsLineToLine);
    inverterInjection->modifyPowerFlowBusType(PowerflowBusType::PQ);

    slack->connect({nGrid});
    resistance->connect({nPcc, nSeries});
    inductance->connect({nSeries, nGrid});
    inverterInjection->connect({nPcc});

    SystemTopology system(
        mFrequency, SystemNodeList{nGrid, nSeries, nPcc},
        SystemComponentList{slack, resistance, inductance, inverterInjection});

    Simulation simulation(simName, Logger::Level::warn);
    simulation.setSystem(system);
    simulation.setDomain(Domain::SP);
    simulation.setSolverType(Solver::Type::NRP);
    simulation.setSolverAndComponentBehaviour(
        Solver::Behaviour::Initialization);
    simulation.setTimeStep(1.0);
    simulation.setFinalTime(2.0);
    simulation.doInitFromNodesAndTerminals(false);
    simulation.run();

    return system;
  }

  template <typename InverterType>
  SystemTopology
  createEmtSystem(const SystemTopology &powerFlow,
                  const std::shared_ptr<InverterType> &inverter) const {
    auto nGrid = SimNode<Real>::make("nGrid", PhaseType::ABC);
    auto nSeries = SimNode<Real>::make("nSeries", PhaseType::ABC);
    auto nPcc = SimNode<Real>::make("nPcc", PhaseType::ABC);

    auto slack = EMT::Ph3::NetworkInjection::make("Slack");
    auto resistance = EMT::Ph3::Resistor::make("GridResistance");
    resistance->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridResistance));
    auto inductance = EMT::Ph3::Inductor::make("GridInductance");
    inductance->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridInductance));

    slack->connect({nGrid});
    inductance->connect({nGrid, nSeries});
    resistance->connect({nSeries, nPcc});
    inverter->connect({EMT::SimNode::GND, nPcc});

    SystemTopology system(
        mFrequency, SystemNodeList{nGrid, nSeries, nPcc},
        SystemComponentList{slack, inductance, resistance, inverter});
    system.initWithPowerflow(powerFlow, Domain::EMT);
    return system;
  }

  Matrix
  buildAnalyticalDelayedDiscreteMatrix(const EMT::Ph3::SSN_GFL_Split &inverter,
                                       Real timeStart, Real timeStep) const {
    const Matrix controllerAdNext = inverter.getControllerDiscreteA();
    const Matrix controllerBdNext = inverter.getControllerDiscreteB();
    const Matrix controllerBdUsed = inverter.getControllerDiscreteBUsed();
    const Matrix controllerC = inverter.getControllerC();
    const Matrix controllerD = inverter.getControllerD();
    const Matrix networkAd = inverter.getDiscreteA();
    const Matrix networkBd = inverter.getDiscreteB();
    const Matrix networkBvd = inverter.getDiscreteControllerOutputB();

    Matrix terminalFromPlant = Matrix::Zero(3, 9);
    terminalFromPlant.block(0, 0, 3, 3) = Matrix::Identity(3, 3);
    terminalFromPlant.block(0, 6, 3, 3) = -mRc * Matrix::Identity(3, 3);

    Matrix measurementFromPlant = Matrix::Zero(6, 9);
    measurementFromPlant.block(0, 0, 3, 3) = Matrix::Identity(3, 3);
    measurementFromPlant.block(3, 6, 3, 3) = Matrix::Identity(3, 3);

    const Real denominator =
        1.0 + 0.5 * timeStep * mGridResistance / mGridInductance;
    const Real gridAdScalar =
        (1.0 - 0.5 * timeStep * mGridResistance / mGridInductance) /
        denominator;
    const Real gridBdScalar = (0.5 * timeStep / mGridInductance) / denominator;

    Matrix terminalInput = Matrix::Zero(9, 3);
    terminalInput.block(0, 0, 6, 3) = networkBd;
    terminalInput.block(6, 0, 3, 3) = gridBdScalar * Matrix::Identity(3, 3);

    Matrix delayedInput = Matrix::Zero(9, 3);
    delayedInput.block(0, 0, 6, 3) = 2.0 * networkBvd;

    const Matrix lhs =
        Matrix::Identity(9, 9) - terminalInput * terminalFromPlant;
    const Matrix physicalFromHistory =
        lhs.fullPivLu().solve(Matrix::Identity(9, 9));
    const Matrix physicalFromDelay = lhs.fullPivLu().solve(delayedInput);

    Matrix historyAd = Matrix::Zero(9, 9);
    historyAd.block(0, 0, 6, 6) = networkAd;
    historyAd.block(6, 6, 3, 3) = gridAdScalar * Matrix::Identity(3, 3);

    Matrix historyTerminalInput = Matrix::Zero(9, 3);
    historyTerminalInput.block(0, 0, 6, 3) =
        (networkAd + Matrix::Identity(6, 6)) * networkBd;
    historyTerminalInput.block(6, 0, 3, 3) =
        (gridAdScalar + 1.0) * gridBdScalar * Matrix::Identity(3, 3);

    Matrix historyDelayDirect = Matrix::Zero(9, 3);
    historyDelayDirect.block(0, 0, 6, 3) = 2.0 * networkAd * networkBvd;

    const Matrix terminalFromHistory = terminalFromPlant * physicalFromHistory;
    const Matrix terminalFromDelay = terminalFromPlant * physicalFromDelay;
    const Matrix historyClosedAd =
        historyAd + historyTerminalInput * terminalFromHistory;
    const Matrix historyClosedDelay =
        historyDelayDirect + historyTerminalInput * terminalFromDelay;

    const Matrix measurementFromHistory =
        measurementFromPlant * physicalFromHistory;
    const Matrix measurementFromDelay =
        measurementFromPlant * physicalFromDelay;
    const Matrix controllerInputUpdate =
        controllerAdNext * controllerBdUsed + controllerBdNext;

    Matrix result = Matrix::Zero(DelayedStateCount, DelayedStateCount);
    const UInt controllerOffset = 0;
    const UInt plantOffset = ControllerStateCount;
    const UInt delayOffset = ControllerStateCount + 9;

    result.block(plantOffset, plantOffset, 9, 9) = historyClosedAd;
    result.block(plantOffset, delayOffset, 9, 3) = historyClosedDelay;

    result.block(controllerOffset, controllerOffset, 8, 8) = controllerAdNext;
    result.block(controllerOffset, plantOffset, 8, 9) =
        controllerInputUpdate * measurementFromHistory;
    result.block(controllerOffset, delayOffset, 8, 3) =
        controllerInputUpdate * measurementFromDelay;

    const Matrix delayMeasurementGain =
        controllerC * controllerBdUsed + controllerD;
    result.block(delayOffset, controllerOffset, 3, 8) = controllerC;
    result.block(delayOffset, plantOffset, 3, 9) =
        delayMeasurementGain * measurementFromHistory;
    result.block(delayOffset, delayOffset, 3, 3) =
        delayMeasurementGain * measurementFromDelay;

    Matrix transformStart =
        Matrix::Identity(DelayedStateCount, DelayedStateCount);
    Matrix transformEnd = transformStart;
    const Real thetaStart = mOmega * timeStart;
    const Real thetaEnd = thetaStart + mOmega * timeStep;

    for (const UInt offset :
         {ControllerStateCount, ControllerStateCount + 3,
          ControllerStateCount + 6, ControllerStateCount + 9}) {
      stampParkBlock(transformStart, offset, thetaStart);
      stampParkBlock(transformEnd, offset, thetaEnd);
    }

    return transformEnd * result * transformStart.transpose();
  }

  Matrix buildAnalyticalNoDelayContinuousMatrix(
      const EMT::Ph3::SSN_GFL_Split &inverter, Real time) const {
    const Matrix controllerA = inverter.getControllerA();
    const Matrix controllerB = inverter.getControllerB();
    const Matrix controllerC = inverter.getControllerC();
    const Matrix controllerD = inverter.getControllerD();
    const Matrix networkA = inverter.getNetworkA();
    const Matrix networkBFull = inverter.getNetworkB();
    const Matrix networkB = networkBFull.leftCols(3);
    const Matrix networkBv = networkBFull.rightCols(3);

    Matrix terminalFromPlant = Matrix::Zero(3, 9);
    terminalFromPlant.block(0, 0, 3, 3) = Matrix::Identity(3, 3);
    terminalFromPlant.block(0, 6, 3, 3) = -mRc * Matrix::Identity(3, 3);

    Matrix measurementFromPlant = Matrix::Zero(6, 9);
    measurementFromPlant.block(0, 0, 3, 3) = Matrix::Identity(3, 3);
    measurementFromPlant.block(3, 6, 3, 3) = Matrix::Identity(3, 3);

    Matrix result = Matrix::Zero(NoDelayStateCount, NoDelayStateCount);
    const UInt controllerOffset = 0;
    const UInt plantOffset = ControllerStateCount;

    result.block(controllerOffset, controllerOffset, 8, 8) = controllerA;
    result.block(controllerOffset, plantOffset, 8, 9) =
        controllerB * measurementFromPlant;

    result.block(plantOffset, controllerOffset, 6, 8) = networkBv * controllerC;
    result.block(plantOffset, plantOffset, 6, 6) = networkA;
    result.block(plantOffset, plantOffset, 6, 9) +=
        networkB * terminalFromPlant +
        networkBv * controllerD * measurementFromPlant;

    result.block(plantOffset + 6, plantOffset, 3, 9) =
        (1.0 / mGridInductance) * terminalFromPlant;
    result.block(plantOffset + 6, plantOffset + 6, 3, 3) +=
        (-mGridResistance / mGridInductance) * Matrix::Identity(3, 3);

    Matrix transform = Matrix::Identity(NoDelayStateCount, NoDelayStateCount);
    const Real theta = mOmega * time;
    for (const UInt offset : {ControllerStateCount, ControllerStateCount + 3,
                              ControllerStateCount + 6})
      stampParkBlock(transform, offset, theta);

    Matrix transformed = transform * result * transform.transpose();
    for (const UInt offset : {ControllerStateCount, ControllerStateCount + 3,
                              ControllerStateCount + 6})
      stampFrameDerivative(transformed, offset, mOmega);

    return transformed;
  }

  Real mFrequency;
  Real mOmega;
  Real mGridVoltageRmsLineToLine;
  Real mGridResistance;
  Real mGridInductance;
  Real mLf;
  Real mCf;
  Real mRf;
  Real mRc;
  Real mKpPLL;
  Real mKiPLL;
  Real mOmegaCutoff;
  Real mPRef;
  Real mQRef;
  Real mKpPowerCtrl;
  Real mKiPowerCtrl;
  Real mKpCurrCtrl;
  Real mKiCurrCtrl;
};

int main() {
  EMTPh3SplitGFLStateSpaceExtractionExample example;
  example.run();
  return 0;
}
