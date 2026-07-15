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
using namespace CPS::DP;

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

class DPPh1SwitchStateSpaceExtractionExample {
public:
  DPPh1SwitchStateSpaceExtractionExample()
      : mTimeStep(100e-6), mFinalTime(5.0 * mTimeStep),
        mSwitchTime(2.0 * mTimeStep), mFrequency(50.0),
        mSourceVoltage(CPS::Math::polar(1.0, 0.0)), mLineResistance(0.5),
        mLineInductance(0.05), mLoad1Resistance(20.0), mLoad2Resistance(10.0),
        mSwitchOpenResistance(1e4), mSwitchClosedResistance(0.1) {}

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
        "DP_Ph1_Switch_StateSpaceExtraction_" + caseName;

    auto sourceNode = SimNode::make("n1");
    auto lineInternalNode = SimNode::make("n2");
    auto loadNode = SimNode::make("nLoad");
    auto switchedLoadNode = SimNode::make("nSwitchedLoad");

    auto source = Ph1::VoltageSource::make("VS");
    source->setParameters(mSourceVoltage, mFrequency);

    auto lineResistance = Ph1::Resistor::make("RLine");
    lineResistance->setParameters(mLineResistance);

    auto lineInductance = Ph1::Inductor::make("LLine");
    lineInductance->setParameters(mLineInductance);

    auto load1 = Ph1::Resistor::make("RLoad1");
    load1->setParameters(mLoad1Resistance);

    auto loadSwitch = Ph1::Switch::make("LoadSwitch");
    loadSwitch->setParameters(mSwitchOpenResistance, mSwitchClosedResistance,
                              initiallyClosed);

    auto load2 = Ph1::Resistor::make("RLoad2");
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
    simulation.setDomain(Domain::DP);
    simulation.setSolverType(Solver::Type::MNA);
    simulation.setTimeStep(mTimeStep);
    simulation.setFinalTime(mFinalTime);
    simulation.doStateSpaceExtraction(true);

    if (recomputeSystemMatrix)
      simulation.doSystemMatrixRecomputation(true);

    if (addClosingEvent)
      simulation.addEvent(SwitchEvent::make(mSwitchTime, loadSwitch, true));

    simulation.run();

    return collectExtractionResult(simulation);
  }

  ExtractionResult runResistorReference(const String &caseName,
                                        Real switchResistance) const {
    const String simulationName =
        "DP_Ph1_Switch_StateSpaceExtraction_" + caseName + "_ResistorReference";

    auto sourceNode = SimNode::make("n1");
    auto lineInternalNode = SimNode::make("n2");
    auto loadNode = SimNode::make("nLoad");
    auto switchedLoadNode = SimNode::make("nSwitchedLoad");

    auto source = Ph1::VoltageSource::make("VS");
    source->setParameters(mSourceVoltage, mFrequency);

    auto lineResistance = Ph1::Resistor::make("RLine");
    lineResistance->setParameters(mLineResistance);

    auto lineInductance = Ph1::Inductor::make("LLine");
    lineInductance->setParameters(mLineInductance);

    auto load1 = Ph1::Resistor::make("RLoad1");
    load1->setParameters(mLoad1Resistance);

    auto switchReference = Ph1::Resistor::make("LoadSwitch");
    switchReference->setParameters(switchResistance);

    auto load2 = Ph1::Resistor::make("RLoad2");
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
    simulation.setDomain(Domain::DP);
    simulation.setSolverType(Solver::Type::MNA);
    simulation.setTimeStep(mTimeStep);
    simulation.setFinalTime(mFinalTime);
    simulation.doStateSpaceExtraction(true);
    simulation.run();

    return collectExtractionResult(simulation);
  }

  Real equivalentLoadResistance(Real switchResistance) const {
    const Real switchedBranchResistance = switchResistance + mLoad2Resistance;

    return mLoad1Resistance * switchedBranchResistance /
           (mLoad1Resistance + switchedBranchResistance);
  }

  CPS::VectorComp expectedDiscreteEigenvalues(Real switchResistance) const {
    const Real equivalentLoad = equivalentLoadResistance(switchResistance);
    const Real damping = (mLineResistance + equivalentLoad) / mLineInductance;
    const Real omegaShift = 2.0 * PI * mFrequency;

    const Complex lambdaDP(-damping, -omegaShift);
    const Complex z = mapContinuousToDiscrete(lambdaDP, mTimeStep);

    CPS::VectorComp expected(2);
    expected << z, std::conj(z);
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
    constexpr UInt expectedStateCount = 2;
    constexpr Real tolerance = DOUBLE_EPSILON;
    constexpr Real minimumConfigurationDifference = 1e-6;

    const std::vector<String> expectedStateNames{"LLine_re", "LLine_im"};

    std::cout
        << "\n============================================================\n"
        << "DP Ph1 switch state-space extraction validation\n"
        << "============================================================\n"
        << std::scientific << std::setprecision(12);

    if (openSwitch.stateCount != expectedStateCount)
      throw std::runtime_error(
          "Switch model did not produce the expected 2 states.");

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
                                  mSwitchOpenResistance, tolerance);
    validateAnalyticalEigenvalues("Closed configuration", closedSwitch,
                                  mSwitchClosedResistance, tolerance);

    printStateNames("Extraction states:", openSwitch.stateNames);
    printEigenvalues("Open-switch discrete-time eigenvalues:",
                     openSwitch.discreteEigenvalues);
    printEigenvalues("Closed-switch discrete-time eigenvalues:",
                     closedSwitch.discreteEigenvalues);
    printEigenvalues("Closed-switch continuous-time native DP eigenvalues:",
                     closedSwitch.continuousEigenvalues);

    std::cout << "\nPASS: DP Ph1 switch extraction matches equivalent "
                 "open/closed resistor models and updates after switching.\n";
  }

  Real mTimeStep;
  Real mFinalTime;
  Real mSwitchTime;
  Real mFrequency;
  Complex mSourceVoltage;

  Real mLineResistance;
  Real mLineInductance;
  Real mLoad1Resistance;
  Real mLoad2Resistance;
  Real mSwitchOpenResistance;
  Real mSwitchClosedResistance;
};

int main() {
  try {
    DPPh1SwitchStateSpaceExtractionExample example;
    example.run();
  } catch (const std::exception &exception) {
    std::cerr << "\nValidation failed: " << exception.what() << "\n";
    return 1;
  }

  return 0;
}
