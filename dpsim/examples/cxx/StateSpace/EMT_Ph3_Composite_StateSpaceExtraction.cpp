// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#include <DPsim.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
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

} // namespace

class EMTPh3CompositeStateSpaceExtractionExample {
public:
  EMTPh3CompositeStateSpaceExtractionExample()
      : mFrequency(50.0), mTimeStep(50e-6), mFinalTime(2.0 * mTimeStep),
        mNominalVoltage(20e3),
        mLineResistance(CPS::Math::singlePhaseParameterToThreePhase(0.05)),
        mLineInductance(CPS::Math::singlePhaseParameterToThreePhase(0.1)),
        mLineCapacitance(CPS::Math::singlePhaseParameterToThreePhase(0.1e-6)),
        mLineConductance(CPS::Math::singlePhaseParameterToThreePhase(1e-6)),
        mLoadActivePower(CPS::Math::singlePhasePowerToThreePhase(100e3)),
        mLoadReactivePower(CPS::Math::singlePhasePowerToThreePhase(50e3)) {}

  void run() const {
    const auto compositeResult = runCompositeModel();
    const auto explicitResult = runExplicitModel();

    validateResults(compositeResult, explicitResult);
  }

private:
  ExtractionResult runCompositeModel() const {
    const String simulationName =
        "EMT_Ph3_Composite_StateSpaceExtraction_Composite";

    auto sourceNode = SimNode::make("n1", PhaseType::ABC);
    auto loadNode = SimNode::make("n2", PhaseType::ABC);

    auto source = Ph3::NetworkInjection::make("Slack");
    source->setParameters(
        CPS::Math::singlePhaseVariableToThreePhase(mNominalVoltage),
        mFrequency);

    auto line = Ph3::PiLine::make("Line");
    line->setParameters(mLineResistance, mLineInductance, mLineCapacitance,
                        mLineConductance);

    auto load = Ph3::RXLoad::make("Load");
    load->setParameters(mLoadActivePower, mLoadReactivePower, mNominalVoltage,
                        false); // Parallel R and X branches

    source->connect({sourceNode});
    line->connect({sourceNode, loadNode});
    load->connect({loadNode});

    auto system =
        SystemTopology(mFrequency, SystemNodeList{sourceNode, loadNode},
                       SystemComponentList{source, line, load});

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

  ExtractionResult runExplicitModel() const {
    const String simulationName =
        "EMT_Ph3_Composite_StateSpaceExtraction_Explicit";

    auto sourceNode = SimNode::make("n1", PhaseType::ABC);
    auto loadNode = SimNode::make("n2", PhaseType::ABC);
    auto lineInternalNode = SimNode::make("Line_internal", PhaseType::ABC);

    auto source = Ph3::NetworkInjection::make("Slack");
    source->setParameters(
        CPS::Math::singlePhaseVariableToThreePhase(mNominalVoltage),
        mFrequency);

    // Explicit PiLine series branch.
    auto lineResistance = Ph3::Resistor::make("Line_res");
    lineResistance->setParameters(mLineResistance);

    auto lineInductance = Ph3::Inductor::make("Line_ind");
    lineInductance->setParameters(mLineInductance);

    // PiLine splits its total shunt capacitance and conductance equally
    // between both terminals.
    const Matrix halfLineCapacitance = mLineCapacitance / 2.0;
    const Matrix lineShuntResistance = 2.0 * mLineConductance.inverse();

    auto lineConductance0 = Ph3::Resistor::make("Line_con0");
    lineConductance0->setParameters(lineShuntResistance);

    auto lineConductance1 = Ph3::Resistor::make("Line_con1");
    lineConductance1->setParameters(lineShuntResistance);

    auto lineCapacitance0 = Ph3::Capacitor::make("Line_cap0");
    lineCapacitance0->setParameters(halfLineCapacitance);

    auto lineCapacitance1 = Ph3::Capacitor::make("Line_cap1");
    lineCapacitance1->setParameters(halfLineCapacitance);

    // Explicit equivalent of the parallel RXLoad.
    const Real phaseVoltage = mNominalVoltage / std::sqrt(3.0);
    const Real omega = 2.0 * PI * mFrequency;

    const Matrix loadResistanceValue =
        std::pow(phaseVoltage, 2) * mLoadActivePower.inverse();

    const Matrix loadReactanceValue =
        std::pow(phaseVoltage, 2) * mLoadReactivePower.inverse();

    const Matrix loadInductanceValue = loadReactanceValue / omega;

    auto loadResistance = Ph3::Resistor::make("Load_res");
    loadResistance->setParameters(loadResistanceValue);

    auto loadInductance = Ph3::Inductor::make("Load_ind");
    loadInductance->setParameters(loadInductanceValue);

    source->connect({sourceNode});

    lineResistance->connect({sourceNode, lineInternalNode});
    lineInductance->connect({lineInternalNode, loadNode});

    // Use the same terminal orientation as the PiLine internals.
    lineConductance0->connect({SimNode::GND, sourceNode});
    lineConductance1->connect({SimNode::GND, loadNode});
    lineCapacitance0->connect({SimNode::GND, sourceNode});
    lineCapacitance1->connect({SimNode::GND, loadNode});

    // RXLoad parallel branches use ground as terminal 0.
    loadResistance->connect({SimNode::GND, loadNode});
    loadInductance->connect({SimNode::GND, loadNode});

    // Keep the component ordering consistent with the internal
    // subcomponent ordering of PiLine and RXLoad. This gives the same
    // extraction-state ordering in both models.
    auto system = SystemTopology(
        mFrequency, SystemNodeList{sourceNode, loadNode, lineInternalNode},
        SystemComponentList{
            source,
            lineResistance,
            lineInductance,
            lineConductance0,
            lineConductance1,
            lineCapacitance0,
            lineCapacitance1,
            loadResistance,
            loadInductance,
        });

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

  void validateResults(const ExtractionResult &compositeResult,
                       const ExtractionResult &explicitResult) const {
    constexpr UInt expectedStateCount = 12;
    constexpr Real tolerance = DOUBLE_EPSILON;

    std::cout
        << "\n============================================================\n"
        << "EMT Ph3 composite state-space extraction validation\n"
        << "============================================================\n";

    std::cout << "Composite state count: " << compositeResult.stateCount
              << "\n";
    std::cout << "Explicit state count:  " << explicitResult.stateCount << "\n";

    // PiLine: 3 inductor + 6 capacitor states.
    // RXLoad: 3 inductor states.
    if (compositeResult.stateCount != expectedStateCount) {
      throw std::runtime_error(
          "Composite model did not produce the expected 12 states.");
    }

    if (explicitResult.stateCount != expectedStateCount) {
      throw std::runtime_error(
          "Explicit model did not produce the expected 12 states.");
    }

    if (compositeResult.stateNames != explicitResult.stateNames) {
      printStateNames("Composite state names:", compositeResult.stateNames);
      printStateNames("Explicit state names:", explicitResult.stateNames);

      throw std::runtime_error(
          "Composite and explicit state ordering do not match.");
    }

    if (compositeResult.discreteStateMatrix.rows() !=
            explicitResult.discreteStateMatrix.rows() ||
        compositeResult.discreteStateMatrix.cols() !=
            explicitResult.discreteStateMatrix.cols()) {
      throw std::runtime_error(
          "Composite and explicit state-matrix dimensions differ.");
    }

    const Matrix difference = compositeResult.discreteStateMatrix -
                              explicitResult.discreteStateMatrix;

    const Real maximumAbsoluteDifference = difference.cwiseAbs().maxCoeff();

    const Real referenceScale = std::max<Real>(
        1.0, explicitResult.discreteStateMatrix.cwiseAbs().maxCoeff());

    const Real relativeDifference = maximumAbsoluteDifference / referenceScale;

    std::cout << std::scientific << std::setprecision(12);
    std::cout << "Maximum absolute Ad difference: " << maximumAbsoluteDifference
              << "\n";
    std::cout << "Relative Ad difference:         " << relativeDifference
              << "\n";

    printStateNames("Extraction states:", compositeResult.stateNames);

    printEigenvalues("Composite discrete-time eigenvalues:",
                     compositeResult.discreteEigenvalues);

    if (relativeDifference > tolerance) {
      std::cout << "\nComposite Ad:\n"
                << compositeResult.discreteStateMatrix << "\n";
      std::cout << "\nExplicit Ad:\n"
                << explicitResult.discreteStateMatrix << "\n";
      std::cout << "\nDifference:\n" << difference << "\n";

      throw std::runtime_error(
          "Composite and explicit state matrices do not match.");
    }

    std::cout << "\nPASS: PiLine and RXLoad composite extraction matches "
                 "the explicit EMT model.\n";
  }

  Real mFrequency;
  Real mTimeStep;
  Real mFinalTime;
  Real mNominalVoltage;

  Matrix mLineResistance;
  Matrix mLineInductance;
  Matrix mLineCapacitance;
  Matrix mLineConductance;

  Matrix mLoadActivePower;
  Matrix mLoadReactivePower;
};

int main() {
  try {
    EMTPh3CompositeStateSpaceExtractionExample example;
    example.run();
  } catch (const std::exception &exception) {
    std::cerr << "\nValidation failed: " << exception.what() << "\n";
    return 1;
  }

  return 0;
}
