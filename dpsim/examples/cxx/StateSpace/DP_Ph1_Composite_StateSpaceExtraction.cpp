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
using namespace CPS::DP;

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

class DPPh1CompositeStateSpaceExtractionExample {
public:
  DPPh1CompositeStateSpaceExtractionExample()
      : mFrequency(50.0), mTimeStep(50e-6), mFinalTime(2.0 * mTimeStep),
        mNominalVoltage(20e3),
        mSourceVoltage(CPS::Math::polar(mNominalVoltage, 0.0)),
        mLineResistance(0.05), mLineInductance(0.1), mLineCapacitance(0.1e-6),
        mLineConductance(1e-6), mLoadActivePower(100e3),
        mLoadReactivePower(50e3) {}

  void run() const {
    const auto compositeResult = runCompositeModel();
    const auto explicitResult = runExplicitModel();

    validateResults(compositeResult, explicitResult);
  }

private:
  ExtractionResult runCompositeModel() const {
    const String simulationName =
        "DP_Ph1_Composite_StateSpaceExtraction_Composite";

    auto sourceNode = SimNode::make("n1");
    auto loadNode = SimNode::make("n2");

    auto source = Ph1::NetworkInjection::make("Slack");
    source->setParameters(mSourceVoltage, mFrequency);

    auto line = Ph1::PiLine::make("Line");
    line->setParameters(mLineResistance, mLineInductance, mLineCapacitance,
                        mLineConductance);

    auto load = Ph1::RXLoad::make("Load");
    load->setParameters(mLoadActivePower, mLoadReactivePower, mNominalVoltage);

    source->connect({sourceNode});
    line->connect({sourceNode, loadNode});
    load->connect({loadNode});

    auto system =
        SystemTopology(mFrequency, SystemNodeList{sourceNode, loadNode},
                       SystemComponentList{source, line, load});

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

  ExtractionResult runExplicitModel() const {
    const String simulationName =
        "DP_Ph1_Composite_StateSpaceExtraction_Explicit";

    auto sourceNode = SimNode::make("n1");
    auto loadNode = SimNode::make("n2");
    auto lineInternalNode = SimNode::make("Line_internal");

    auto source = Ph1::NetworkInjection::make("Slack");
    source->setParameters(mSourceVoltage, mFrequency);

    // Explicit PiLine series branch.
    auto lineResistance = Ph1::Resistor::make("Line_res");
    lineResistance->setParameters(mLineResistance);

    auto lineInductance = Ph1::Inductor::make("Line_ind");
    lineInductance->setParameters(mLineInductance);

    // PiLine splits its total shunt capacitance and conductance equally
    // between both terminals.
    const Real halfLineCapacitance = mLineCapacitance / 2.0;
    const Real lineShuntResistance = 2.0 / mLineConductance;

    auto lineConductance0 = Ph1::Resistor::make("Line_con0");
    lineConductance0->setParameters(lineShuntResistance);

    auto lineConductance1 = Ph1::Resistor::make("Line_con1");
    lineConductance1->setParameters(lineShuntResistance);

    auto lineCapacitance0 = Ph1::Capacitor::make("Line_cap0");
    lineCapacitance0->setParameters(halfLineCapacitance);

    auto lineCapacitance1 = Ph1::Capacitor::make("Line_cap1");
    lineCapacitance1->setParameters(halfLineCapacitance);

    // Explicit equivalent of the parallel RXLoad.
    const Real omega = 2.0 * PI * mFrequency;
    const Real loadResistanceValue =
        std::pow(mNominalVoltage, 2) / mLoadActivePower;
    const Real loadReactanceValue =
        std::pow(mNominalVoltage, 2) / mLoadReactivePower;
    const Real loadInductanceValue = loadReactanceValue / omega;

    auto loadResistance = Ph1::Resistor::make("Load_res");
    loadResistance->setParameters(loadResistanceValue);

    auto loadInductance = Ph1::Inductor::make("Load_ind");
    loadInductance->setParameters(loadInductanceValue);

    source->connect({sourceNode});

    lineResistance->connect({sourceNode, lineInternalNode});
    lineInductance->connect({lineInternalNode, loadNode});

    // Use the same terminal orientation as the PiLine internals.
    lineConductance0->connect({SimNode::GND, sourceNode});
    lineConductance1->connect({SimNode::GND, loadNode});
    lineCapacitance0->connect({SimNode::GND, sourceNode});
    lineCapacitance1->connect({SimNode::GND, loadNode});

    // RXLoad branches are connected in parallel between ground and the load
    // node.
    loadResistance->connect({SimNode::GND, loadNode});
    loadInductance->connect({SimNode::GND, loadNode});

    // Keep the component order consistent with the internal subcomponent
    // order of PiLine and RXLoad. This gives the same extraction-state order
    // in both models.
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
    simulation.setDomain(Domain::DP);
    simulation.setSolverType(Solver::Type::MNA);
    simulation.setTimeStep(mTimeStep);
    simulation.setFinalTime(mFinalTime);
    simulation.doStateSpaceExtraction(true);
    simulation.run();

    return collectExtractionResult(simulation);
  }

  void validateResults(const ExtractionResult &compositeResult,
                       const ExtractionResult &explicitResult) const {
    constexpr UInt expectedStateCount = 8;
    constexpr Real tolerance = 1e-10;

    std::cout
        << "\n============================================================\n"
        << "DP Ph1 composite state-space extraction validation\n"
        << "============================================================\n";

    std::cout << "Composite state count: " << compositeResult.stateCount
              << "\n";
    std::cout << "Explicit state count:  " << explicitResult.stateCount << "\n";

    // PiLine: one complex inductor state and two complex capacitor states.
    // RXLoad: one complex inductor state.
    // Each complex state is represented by real and imaginary parts.
    if (compositeResult.stateCount != expectedStateCount) {
      throw std::runtime_error(
          "Composite model did not produce the expected 8 states.");
    }

    if (explicitResult.stateCount != expectedStateCount) {
      throw std::runtime_error(
          "Explicit model did not produce the expected 8 states.");
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

    const Real referenceScale =
        std::max(1.0, explicitResult.discreteStateMatrix.cwiseAbs().maxCoeff());

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

    std::cout
        << "\nPASS: DP Ph1 PiLine and RXLoad composite extraction matches "
           "the explicit DP model.\n";
  }

  Real mFrequency;
  Real mTimeStep;
  Real mFinalTime;
  Real mNominalVoltage;
  Complex mSourceVoltage;

  Real mLineResistance;
  Real mLineInductance;
  Real mLineCapacitance;
  Real mLineConductance;

  Real mLoadActivePower;
  Real mLoadReactivePower;
};

int main() {
  try {
    DPPh1CompositeStateSpaceExtractionExample example;
    example.run();
  } catch (const std::exception &exception) {
    std::cerr << "\nValidation failed: " << exception.what() << "\n";
    return 1;
  }

  return 0;
}
