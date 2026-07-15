// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#include <DPsim.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace DPsim;
using namespace CPS::EMT;

namespace {

struct ExtractionResult {
  UInt stateCount = 0;
  Matrix discreteStateMatrix;
  std::vector<String> stateNames;
  CPS::VectorComp discreteEigenvalues;
  CPS::VectorComp continuousEigenvalues;
};

ExtractionResult collectExtractionResult(const Simulation &simulation) {
  const auto &extractor = simulation.getStateSpaceExtractor();

  StateSpaceModalAnalysis modalAnalysis(extractor);
  modalAnalysis.update();

  ExtractionResult result;
  result.stateCount = extractor.getStateCount();
  result.discreteStateMatrix = extractor.getDiscreteStateMatrix();
  result.stateNames = modalAnalysis.getStateNames();
  result.discreteEigenvalues = modalAnalysis.getDiscreteEigenvalues();
  result.continuousEigenvalues = modalAnalysis.getContinuousEigenvalues();

  return result;
}

void printStateNames(const String &title,
                     const std::vector<String> &stateNames) {
  std::cout << "\n" << title << "\n";

  for (UInt idx = 0; idx < stateNames.size(); ++idx)
    std::cout << "  [" << idx << "] " << stateNames[idx] << "\n";
}

void printEigenvalues(const String &title, const CPS::VectorComp &eigenvalues) {
  std::cout << "\n" << title << "\n";

  for (Eigen::Index idx = 0; idx < eigenvalues.rows(); ++idx)
    std::cout << "  [" << idx << "] " << eigenvalues(idx) << "\n";
}

Complex mapContinuousToDiscrete(const Complex &lambda, Real timeStep) {
  const Complex two(2.0, 0.0);
  return (two + timeStep * lambda) / (two - timeStep * lambda);
}

Real maximumEigenvalueMatchingError(const CPS::VectorComp &actual,
                                    const CPS::VectorComp &expected) {
  if (actual.rows() != expected.rows())
    return std::numeric_limits<Real>::infinity();

  std::vector<bool> used(expected.rows(), false);
  Real maximumError = 0.0;

  for (Eigen::Index actualIdx = 0; actualIdx < actual.rows(); ++actualIdx) {
    Real bestError = std::numeric_limits<Real>::infinity();
    Eigen::Index bestExpectedIdx = -1;

    for (Eigen::Index expectedIdx = 0; expectedIdx < expected.rows();
         ++expectedIdx) {
      if (used[expectedIdx])
        continue;

      const Real error = std::abs(actual(actualIdx) - expected(expectedIdx));
      if (error < bestError) {
        bestError = error;
        bestExpectedIdx = expectedIdx;
      }
    }

    if (bestExpectedIdx < 0)
      return std::numeric_limits<Real>::infinity();

    used[bestExpectedIdx] = true;
    maximumError = std::max(maximumError, bestError);
  }

  return maximumError;
}

} // namespace

class EMTPh3SwitchStateSpaceExtractionExample {
public:
  EMTPh3SwitchStateSpaceExtractionExample()
      : mTimeStep(100e-6), mFinalTime(5.0 * mTimeStep),
        mSwitchTime(2.0 * mTimeStep), mFrequency(50.0),
        mSourceVoltage(CPS::Math::polar(1.0, 0.0)), mLineResistanceValue(0.5),
        mLineInductanceValue(0.05), mLoad1ResistanceValue(20.0),
        mLoad2ResistanceValue(10.0), mSwitchOpenResistanceValue(1e4),
        mSwitchClosedResistanceValue(0.1),
        mLineResistance(
            CPS::Math::singlePhaseParameterToThreePhase(mLineResistanceValue)),
        mLineInductance(
            CPS::Math::singlePhaseParameterToThreePhase(mLineInductanceValue)),
        mLoad1Resistance(
            CPS::Math::singlePhaseParameterToThreePhase(mLoad1ResistanceValue)),
        mLoad2Resistance(
            CPS::Math::singlePhaseParameterToThreePhase(mLoad2ResistanceValue)),
        mSwitchOpenResistance(CPS::Math::singlePhaseParameterToThreePhase(
            mSwitchOpenResistanceValue)),
        mSwitchClosedResistance(CPS::Math::singlePhaseParameterToThreePhase(
            mSwitchClosedResistanceValue)) {}

  void run() const {
    const auto openSwitch = runSwitchModel("Open", false, false, false);
    const auto openReference =
        runResistorReference("Open", mSwitchOpenResistance);

    const auto closedSwitch = runSwitchModel("Closed", true, false, false);
    const auto closedReference =
        runResistorReference("Closed", mSwitchClosedResistance);

    const auto switchedPrecomputed =
        runSwitchModel("Event_Precomputed", false, true, false);
    const auto switchedRecomputed =
        runSwitchModel("Event_Recomputed", false, true, true);

    validateResults(openSwitch, openReference, closedSwitch, closedReference,
                    switchedPrecomputed, switchedRecomputed);
  }

private:
  ExtractionResult runSwitchModel(const String &caseName, Bool initiallyClosed,
                                  Bool addClosingEvent,
                                  Bool recomputeSystemMatrix) const {
    const String simulationName =
        "EMT_Ph3_Switch_StateSpaceExtraction_" + caseName;

    auto sourceNode = SimNode::make("n1", PhaseType::ABC);
    auto lineInternalNode = SimNode::make("n2", PhaseType::ABC);
    auto loadNode = SimNode::make("nLoad", PhaseType::ABC);
    auto switchedLoadNode = SimNode::make("nSwitchedLoad", PhaseType::ABC);

    auto source = Ph3::VoltageSource::make("VS");
    source->setParameters(
        CPS::Math::singlePhaseVariableToThreePhase(mSourceVoltage), mFrequency);

    auto lineResistance = Ph3::Resistor::make("RLine");
    lineResistance->setParameters(mLineResistance);

    auto lineInductance = Ph3::Inductor::make("LLine");
    lineInductance->setParameters(mLineInductance);

    auto load1 = Ph3::Resistor::make("RLoad1");
    load1->setParameters(mLoad1Resistance);

    auto loadSwitch = Ph3::Switch::make("LoadSwitch");
    loadSwitch->setParameters(mSwitchOpenResistance, mSwitchClosedResistance,
                              initiallyClosed);

    auto load2 = Ph3::Resistor::make("RLoad2");
    load2->setParameters(mLoad2Resistance);

    source->connect({SimNode::GND, sourceNode});
    lineResistance->connect({sourceNode, lineInternalNode});
    lineInductance->connect({lineInternalNode, loadNode});
    load1->connect({loadNode, SimNode::GND});
    loadSwitch->connect({loadNode, switchedLoadNode});
    load2->connect({switchedLoadNode, SimNode::GND});

    auto system = SystemTopology(mFrequency,
                                 SystemNodeList{sourceNode, lineInternalNode,
                                                loadNode, switchedLoadNode},
                                 SystemComponentList{source, lineResistance,
                                                     lineInductance, load1,
                                                     loadSwitch, load2});

    Simulation simulation(simulationName, Logger::Level::warn);
    simulation.setSystem(system);
    simulation.setDomain(Domain::EMT);
    simulation.setSolverType(Solver::Type::MNA);
    simulation.setTimeStep(mTimeStep);
    simulation.setFinalTime(mFinalTime);
    simulation.doStateSpaceExtraction(true);

    if (recomputeSystemMatrix)
      simulation.doSystemMatrixRecomputation(true);

    if (addClosingEvent)
      simulation.addEvent(SwitchEvent3Ph::make(mSwitchTime, loadSwitch, true));

    simulation.run();

    return collectExtractionResult(simulation);
  }

  ExtractionResult runResistorReference(const String &caseName,
                                        const Matrix &switchResistance) const {
    const String simulationName = "EMT_Ph3_Switch_StateSpaceExtraction_" +
                                  caseName + "_ResistorReference";

    auto sourceNode = SimNode::make("n1", PhaseType::ABC);
    auto lineInternalNode = SimNode::make("n2", PhaseType::ABC);
    auto loadNode = SimNode::make("nLoad", PhaseType::ABC);
    auto switchedLoadNode = SimNode::make("nSwitchedLoad", PhaseType::ABC);

    auto source = Ph3::VoltageSource::make("VS");
    source->setParameters(
        CPS::Math::singlePhaseVariableToThreePhase(mSourceVoltage), mFrequency);

    auto lineResistance = Ph3::Resistor::make("RLine");
    lineResistance->setParameters(mLineResistance);

    auto lineInductance = Ph3::Inductor::make("LLine");
    lineInductance->setParameters(mLineInductance);

    auto load1 = Ph3::Resistor::make("RLoad1");
    load1->setParameters(mLoad1Resistance);

    auto switchReference = Ph3::Resistor::make("LoadSwitch");
    switchReference->setParameters(switchResistance);

    auto load2 = Ph3::Resistor::make("RLoad2");
    load2->setParameters(mLoad2Resistance);

    source->connect({SimNode::GND, sourceNode});
    lineResistance->connect({sourceNode, lineInternalNode});
    lineInductance->connect({lineInternalNode, loadNode});
    load1->connect({loadNode, SimNode::GND});
    switchReference->connect({loadNode, switchedLoadNode});
    load2->connect({switchedLoadNode, SimNode::GND});

    auto system = SystemTopology(mFrequency,
                                 SystemNodeList{sourceNode, lineInternalNode,
                                                loadNode, switchedLoadNode},
                                 SystemComponentList{source, lineResistance,
                                                     lineInductance, load1,
                                                     switchReference, load2});

    Simulation simulation(simulationName, Logger::Level::warn);
    simulation.setSystem(system);
    simulation.setDomain(Domain::EMT);
    simulation.setSolverType(Solver::Type::MNA);
    simulation.setTimeStep(mTimeStep);
    simulation.setFinalTime(mFinalTime);
    simulation.doStateSpaceExtraction(true);
    simulation.run();

    return collectExtractionResult(simulation);
  }

  Real equivalentLoadResistance(Real switchResistance) const {
    const Real switchedBranchResistance =
        switchResistance + mLoad2ResistanceValue;

    return mLoad1ResistanceValue * switchedBranchResistance /
           (mLoad1ResistanceValue + switchedBranchResistance);
  }

  CPS::VectorComp expectedDiscreteEigenvalues(Real switchResistance) const {
    const Real equivalentLoad = equivalentLoadResistance(switchResistance);

    const Complex lambda(
        -(mLineResistanceValue + equivalentLoad) / mLineInductanceValue, 0.0);

    const Complex z = mapContinuousToDiscrete(lambda, mTimeStep);

    CPS::VectorComp expected(3);
    expected << z, z, z;
    return expected;
  }

  void validateEquivalent(const String &label, const ExtractionResult &actual,
                          const ExtractionResult &reference,
                          Real tolerance) const {
    if (actual.stateCount != reference.stateCount)
      throw std::runtime_error(label + ": state counts differ.");

    if (actual.stateNames != reference.stateNames)
      throw std::runtime_error(label + ": state names differ.");

    if (actual.discreteStateMatrix.rows() !=
            reference.discreteStateMatrix.rows() ||
        actual.discreteStateMatrix.cols() !=
            reference.discreteStateMatrix.cols()) {
      throw std::runtime_error(label + ": state-matrix dimensions differ.");
    }

    const Real matrixError =
        (actual.discreteStateMatrix - reference.discreteStateMatrix)
            .cwiseAbs()
            .maxCoeff();

    const Real eigenvalueError = maximumEigenvalueMatchingError(
        actual.discreteEigenvalues, reference.discreteEigenvalues);

    std::cout << label << "\n";
    std::cout << "  Maximum absolute Ad difference: " << matrixError << "\n";
    std::cout << "  Maximum eigenvalue difference:  " << eigenvalueError
              << "\n";

    if (matrixError > tolerance)
      throw std::runtime_error(label + ": state matrices differ.");

    if (eigenvalueError > tolerance)
      throw std::runtime_error(label + ": eigenvalues differ.");
  }

  void validateAnalyticalEigenvalues(const String &label,
                                     const ExtractionResult &result,
                                     Real switchResistance,
                                     Real tolerance) const {
    const auto expected = expectedDiscreteEigenvalues(switchResistance);

    const Real error =
        maximumEigenvalueMatchingError(result.discreteEigenvalues, expected);

    std::cout << label << " analytical discrete-eigenvalue error: " << error
              << "\n";

    if (error > tolerance)
      throw std::runtime_error(label +
                               ": extracted eigenvalues do not match the "
                               "analytical reference.");
  }

  void validateResults(const ExtractionResult &openSwitch,
                       const ExtractionResult &openReference,
                       const ExtractionResult &closedSwitch,
                       const ExtractionResult &closedReference,
                       const ExtractionResult &switchedPrecomputed,
                       const ExtractionResult &switchedRecomputed) const {
    constexpr UInt expectedStateCount = 3;
    constexpr Real tolerance = DOUBLE_EPSILON;
    constexpr Real minimumConfigurationDifference = 1e-6;

    const std::vector<String> expectedStateNames{"LLine_a", "LLine_b",
                                                 "LLine_c"};

    std::cout
        << "\n============================================================\n"
        << "EMT Ph3 switch state-space extraction validation\n"
        << "============================================================\n"
        << std::scientific << std::setprecision(12);

    if (openSwitch.stateCount != expectedStateCount)
      throw std::runtime_error(
          "Switch model did not produce the expected 3 states.");

    if (openSwitch.stateNames != expectedStateNames) {
      printStateNames("Actual extraction states:", openSwitch.stateNames);
      throw std::runtime_error("Switch model produced unexpected state names.");
    }

    validateEquivalent("Open switch vs. open-resistance reference", openSwitch,
                       openReference, tolerance);
    validateEquivalent("Closed switch vs. closed-resistance reference",
                       closedSwitch, closedReference, tolerance);
    validateEquivalent("Open-to-closed event with precomputed switch matrices",
                       switchedPrecomputed, closedSwitch, tolerance);
    validateEquivalent("Open-to-closed event with system-matrix recomputation",
                       switchedRecomputed, closedSwitch, tolerance);

    const Real configurationDifference =
        (openSwitch.discreteStateMatrix - closedSwitch.discreteStateMatrix)
            .cwiseAbs()
            .maxCoeff();

    std::cout << "Open/closed maximum absolute Ad difference: "
              << configurationDifference << "\n";

    if (configurationDifference < minimumConfigurationDifference) {
      throw std::runtime_error("Open and closed switch configurations are not "
                               "sufficiently distinguishable.");
    }

    validateAnalyticalEigenvalues("Open configuration", openSwitch,
                                  mSwitchOpenResistanceValue, tolerance);
    validateAnalyticalEigenvalues("Closed configuration", closedSwitch,
                                  mSwitchClosedResistanceValue, tolerance);

    printStateNames("Extraction states:", openSwitch.stateNames);
    printEigenvalues("Open-switch discrete-time eigenvalues:",
                     openSwitch.discreteEigenvalues);
    printEigenvalues("Closed-switch discrete-time eigenvalues:",
                     closedSwitch.discreteEigenvalues);
    printEigenvalues("Closed-switch continuous-time eigenvalues:",
                     closedSwitch.continuousEigenvalues);

    std::cout << "\nPASS: EMT Ph3 switch extraction matches equivalent "
                 "open/closed resistor models and updates after switching.\n";
  }

  Real mTimeStep;
  Real mFinalTime;
  Real mSwitchTime;
  Real mFrequency;
  Complex mSourceVoltage;

  Real mLineResistanceValue;
  Real mLineInductanceValue;
  Real mLoad1ResistanceValue;
  Real mLoad2ResistanceValue;
  Real mSwitchOpenResistanceValue;
  Real mSwitchClosedResistanceValue;

  Matrix mLineResistance;
  Matrix mLineInductance;
  Matrix mLoad1Resistance;
  Matrix mLoad2Resistance;
  Matrix mSwitchOpenResistance;
  Matrix mSwitchClosedResistance;
};

int main() {
  try {
    EMTPh3SwitchStateSpaceExtractionExample example;
    example.run();
  } catch (const std::exception &exception) {
    std::cerr << "\nValidation failed: " << exception.what() << "\n";
    return 1;
  }

  return 0;
}
