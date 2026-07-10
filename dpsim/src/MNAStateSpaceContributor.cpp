// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#include <dpsim/MNAStateSpaceContributor.h>

#include <dpsim-models/DP/DP_Ph1_Capacitor.h>
#include <dpsim-models/DP/DP_Ph1_Inductor.h>
#include <dpsim-models/DP/DP_Ph1_Resistor.h>
#include <dpsim-models/DP/DP_Ph1_TwoTerminalVTypeSSNComp.h>
#include <dpsim-models/DP/DP_Ph1_VoltageSource.h>
#include <dpsim-models/DP/DP_VTypeSSNComp.h>
#include <dpsim-models/EMT/EMT_Ph3_Capacitor.h>
#include <dpsim-models/EMT/EMT_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeSSNComp.h>
#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeVariableSSNComp.h>
#include <dpsim-models/EMT/EMT_Ph3_VoltageSource.h>
#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>
#include <dpsim-models/SimPowerComp.h>
#include <stdexcept>
#include <string>
#include <utility>

using namespace CPS;

namespace DPsim {
namespace {

using SimPowerCompReal = CPS::SimPowerComp<CPS::Real>;
using SimPowerCompComplex = CPS::SimPowerComp<CPS::Complex>;

/// Builds K for vIntf = K xMNA, with
/// vIntf = v_terminal1 - v_terminal0.
Matrix buildTwoTerminalInterfaceVoltageMapping(SimPowerCompReal &component,
                                               UInt mnaVectorSize) {
  Matrix K = Matrix::Zero(3, mnaVectorSize);

  if (component.terminalNotGrounded(1)) {
    for (UInt phase = 0; phase < 3; ++phase)
      K(phase, component.matrixNodeIndex(1, phase)) = 1.0;
  }

  if (component.terminalNotGrounded(0)) {
    for (UInt phase = 0; phase < 3; ++phase)
      K(phase, component.matrixNodeIndex(0, phase)) = -1.0;
  }

  return K;
}

/// Builds K for [Re(vIntf), Im(vIntf)]^T = K xMNA, with
/// vIntf = v_terminal1 - v_terminal0.
Matrix
buildSinglePhaseComplexInterfaceVoltageMapping(SimPowerCompComplex &component,
                                               UInt mnaVectorSize) {
  if (mnaVectorSize % 2 != 0) {
    throw std::logic_error(
        "DP MNA state-space extraction requires a real-imaginary stacked "
        "MNA vector with even size.");
  }

  const UInt complexOffset = mnaVectorSize / 2;
  Matrix K = Matrix::Zero(2, mnaVectorSize);

  if (component.terminalNotGrounded(1)) {
    const UInt nodeIdx = component.matrixNodeIndex(1);
    K(0, nodeIdx) = 1.0;
    K(1, nodeIdx + complexOffset) = 1.0;
  }

  if (component.terminalNotGrounded(0)) {
    const UInt nodeIdx = component.matrixNodeIndex(0);
    K(0, nodeIdx) = -1.0;
    K(1, nodeIdx + complexOffset) = -1.0;
  }

  return K;
}

/// Stamps -K^T * outputMatrix into CdMna.
///
/// The sign follows the two-terminal convention vIntf = v1 - v0 and the
/// corresponding current-source stamping: terminal 0: +y, terminal 1: -y.
void stampTwoTerminalCurrentInjectionMapping(const Matrix &K, Matrix &CdMna,
                                             UInt stateOffset,
                                             const Matrix &outputMatrix) {
  CdMna.block(0, stateOffset, CdMna.rows(), outputMatrix.cols()) +=
      -K.transpose() * outputMatrix;
}

Matrix realAugment(const MatrixComp &matrix) {
  const Matrix::Index rows = matrix.rows();
  const Matrix::Index cols = matrix.cols();

  Matrix result = Matrix::Zero(2 * rows, 2 * cols);
  result.topLeftCorner(rows, cols) = matrix.real();
  result.topRightCorner(rows, cols) = -matrix.imag();
  result.bottomLeftCorner(rows, cols) = matrix.imag();
  result.bottomRightCorner(rows, cols) = matrix.real();

  return result;
}

Matrix realAugment(const Complex &value) {
  Matrix result = Matrix::Zero(2, 2);
  result << value.real(), -value.imag(), value.imag(), value.real();
  return result;
}

Complex calculateDPInductorPreviousCurrentFactor(const Complex &conductance) {
  const Real omegaDtHalf = -conductance.imag() / conductance.real();
  return Complex(1.0, -omegaDtHalf) / Complex(1.0, omegaDtHalf);
}

void setStateName(StateSpaceMetadata &metadata, UInt stateIndex,
                  const String &name) {
  if (stateIndex >= metadata.stateNames.size())
    throw std::runtime_error(
        "MNA state-space contributor tried to set a state name outside the "
        "extracted state vector.");

  metadata.stateNames[stateIndex] = name;
}

void addThreePhaseAbcStateMetadata(StateSpaceMetadata &metadata,
                                   UInt stateOffset, const String &baseName) {
  setStateName(metadata, stateOffset + 0, baseName + "_a");
  setStateName(metadata, stateOffset + 1, baseName + "_b");
  setStateName(metadata, stateOffset + 2, baseName + "_c");

  metadata.abcStateBlocks.push_back(
      {{stateOffset + 0, stateOffset + 1, stateOffset + 2}, baseName});
}

void addSinglePhaseComplexStateMetadata(StateSpaceMetadata &metadata,
                                        UInt stateOffset,
                                        const String &baseName) {
  setStateName(metadata, stateOffset + 0, baseName + "_re");
  setStateName(metadata, stateOffset + 1, baseName + "_im");
}

void addComplexStateMetadata(StateSpaceMetadata &metadata, UInt stateOffset,
                             UInt complexStateCount,
                             const String &componentName) {
  for (UInt idx = 0; idx < complexStateCount; ++idx) {
    const String stateName = componentName + ".x" + std::to_string(idx);
    setStateName(metadata, stateOffset + idx, stateName + "_re");
    setStateName(metadata, stateOffset + complexStateCount + idx,
                 stateName + "_im");
  }
}

class EMTPh3InductorStateSpaceContributor final
    : public MNAStateSpaceContributor {
public:
  explicit EMTPh3InductorStateSpaceContributor(
      std::shared_ptr<EMT::Ph3::Inductor> component)
      : mComponent(std::move(component)) {}

  UInt getStateCount() const override { return 3; }

  void stamp(Matrix &AdLocal, Matrix &BdMna, Matrix &CdMna, UInt stateOffset,
             UInt mnaVectorSize) const override {
    const Matrix &conductance = mComponent->getMNAConductance();

    const Matrix K =
        buildTwoTerminalInterfaceVoltageMapping(*mComponent, mnaVectorSize);

    // History state h:
    //   i[k+1] = G vIntf[k+1] + h[k]
    //   h[k+1] = h[k] + 2 G vIntf[k+1]
    // Therefore: AdLocal = I, BdMna = 2 G K, CdMna = -K^T.
    AdLocal.block(stateOffset, stateOffset, 3, 3) += Matrix::Identity(3, 3);

    BdMna.block(stateOffset, 0, 3, mnaVectorSize) += 2.0 * conductance * K;

    stampTwoTerminalCurrentInjectionMapping(K, CdMna, stateOffset,
                                            Matrix::Identity(3, 3));
  }

  void contributeMetadata(StateSpaceMetadata &metadata,
                          UInt stateOffset) const override {
    addThreePhaseAbcStateMetadata(metadata, stateOffset, mComponent->name());
  }

private:
  std::shared_ptr<EMT::Ph3::Inductor> mComponent;
};

class EMTPh3CapacitorStateSpaceContributor final
    : public MNAStateSpaceContributor {
public:
  explicit EMTPh3CapacitorStateSpaceContributor(
      std::shared_ptr<EMT::Ph3::Capacitor> component)
      : mComponent(std::move(component)) {}

  UInt getStateCount() const override { return 3; }

  void stamp(Matrix &AdLocal, Matrix &BdMna, Matrix &CdMna, UInt stateOffset,
             UInt mnaVectorSize) const override {
    const Matrix &conductance = mComponent->getMNAConductance();

    const Matrix K =
        buildTwoTerminalInterfaceVoltageMapping(*mComponent, mnaVectorSize);

    // History state h:
    //   i[k+1] = G vIntf[k+1] + h[k]
    //   h[k+1] = -h[k] - 2 G vIntf[k+1]
    // Therefore: AdLocal = -I, BdMna = -2 G K, CdMna = -K^T.
    AdLocal.block(stateOffset, stateOffset, 3, 3) -= Matrix::Identity(3, 3);

    BdMna.block(stateOffset, 0, 3, mnaVectorSize) -= 2.0 * conductance * K;

    stampTwoTerminalCurrentInjectionMapping(K, CdMna, stateOffset,
                                            Matrix::Identity(3, 3));
  }

  void contributeMetadata(StateSpaceMetadata &metadata,
                          UInt stateOffset) const override {
    addThreePhaseAbcStateMetadata(metadata, stateOffset, mComponent->name());
  }

private:
  std::shared_ptr<EMT::Ph3::Capacitor> mComponent;
};

class EMTPh3TwoTerminalVTypeSSNStateSpaceContributor final
    : public MNAStateSpaceContributor {
public:
  EMTPh3TwoTerminalVTypeSSNStateSpaceContributor(
      std::shared_ptr<EMT::VTypeSSNComp> component, Bool isVariable)
      : mComponent(std::move(component)), mIsVariable(isVariable) {}

  UInt getStateCount() const override { return mComponent->getStateCount(); }

  Bool isVariable() const override { return mIsVariable; }

  void stamp(Matrix &AdLocal, Matrix &BdMna, Matrix &CdMna, UInt stateOffset,
             UInt mnaVectorSize) const override {
    const UInt localStateCount = getStateCount();

    const Matrix &discreteA = mComponent->getDiscreteA();
    const Matrix &discreteB = mComponent->getDiscreteB();
    const Matrix &outputC = mComponent->getC();

    const Matrix K =
        buildTwoTerminalInterfaceVoltageMapping(*mComponent, mnaVectorSize);

    // History-coordinate state s = discreteA x + discreteB vIntf:
    //   s[k+1] = discreteA s[k] + (discreteA + I) discreteB vIntf[k+1]
    //   yHist[k] = C s[k]
    // Therefore: AdLocal = discreteA, BdMna = (discreteA + I) discreteB K,
    // CdMna = -K^T C.
    AdLocal.block(stateOffset, stateOffset, localStateCount, localStateCount) +=
        discreteA;

    const Matrix inputUpdate =
        (discreteA + Matrix::Identity(localStateCount, localStateCount)) *
        discreteB;

    BdMna.block(stateOffset, 0, localStateCount, mnaVectorSize) +=
        inputUpdate * K;

    stampTwoTerminalCurrentInjectionMapping(K, CdMna, stateOffset, outputC);
  }

  void contributeMetadata(StateSpaceMetadata &metadata,
                          UInt stateOffset) const override {
    const UInt localStateCount = getStateCount();
    const String componentName = mComponent->name();

    const auto localStateNames = mComponent->getLocalStateNames();

    if (!localStateNames.empty() && localStateNames.size() != localStateCount) {
      throw std::runtime_error(
          "SSN component returned an invalid number of local state names.");
    }

    for (UInt idx = 0; idx < localStateCount; ++idx) {
      if (!localStateNames.empty()) {
        setStateName(metadata, stateOffset + idx,
                     componentName + "." + localStateNames[idx]);
      }
    }

    for (auto abcBlock : mComponent->getLocalAbcStateBlocks()) {
      if (abcBlock.name.empty()) {
        throw std::runtime_error(
            "SSN component returned an abc state block with an empty name.");
      }

      for (auto &idx : abcBlock.indices) {
        if (idx >= localStateCount) {
          throw std::runtime_error(
              "SSN component returned an invalid abc state index.");
        }

        idx += stateOffset;
      }

      metadata.abcStateBlocks.push_back(
          {abcBlock.indices, componentName + "." + abcBlock.name});
    }
  }

private:
  std::shared_ptr<EMT::VTypeSSNComp> mComponent;
  Bool mIsVariable = false;
};

class DPPh1InductorStateSpaceContributor final
    : public MNAStateSpaceContributor {
public:
  explicit DPPh1InductorStateSpaceContributor(
      std::shared_ptr<DP::Ph1::Inductor> component)
      : mComponent(std::move(component)) {}

  UInt getStateCount() const override { return 2; }

  void stamp(Matrix &AdLocal, Matrix &BdMna, Matrix &CdMna, UInt stateOffset,
             UInt mnaVectorSize) const override {
    const Complex conductance = mComponent->getMNAConductance();
    const Complex prevCurrentFactor =
        calculateDPInductorPreviousCurrentFactor(conductance);

    const Matrix K = buildSinglePhaseComplexInterfaceVoltageMapping(
        *mComponent, mnaVectorSize);

    // Complex history state h:
    //   i[k+1] = Y_L vIntf[k+1] + h[k]
    //   h[k+1] = alpha h[k] + (1 + alpha) Y_L vIntf[k+1]
    // Therefore: AdLocal = alpha, BdMna = (1 + alpha) Y_L K,
    // CdMna = -K^T in real-imaginary augmented form.
    AdLocal.block(stateOffset, stateOffset, 2, 2) +=
        realAugment(prevCurrentFactor);

    BdMna.block(stateOffset, 0, 2, mnaVectorSize) +=
        realAugment((Complex(1.0, 0.0) + prevCurrentFactor) * conductance) * K;

    stampTwoTerminalCurrentInjectionMapping(K, CdMna, stateOffset,
                                            Matrix::Identity(2, 2));
  }

  void contributeMetadata(StateSpaceMetadata &metadata,
                          UInt stateOffset) const override {
    addSinglePhaseComplexStateMetadata(metadata, stateOffset,
                                       mComponent->name());
  }

private:
  std::shared_ptr<DP::Ph1::Inductor> mComponent;
};

class DPPh1CapacitorStateSpaceContributor final
    : public MNAStateSpaceContributor {
public:
  explicit DPPh1CapacitorStateSpaceContributor(
      std::shared_ptr<DP::Ph1::Capacitor> component)
      : mComponent(std::move(component)) {}

  UInt getStateCount() const override { return 2; }

  void stamp(Matrix &AdLocal, Matrix &BdMna, Matrix &CdMna, UInt stateOffset,
             UInt mnaVectorSize) const override {
    const Complex conductance = mComponent->getMNAConductance();

    const Matrix K = buildSinglePhaseComplexInterfaceVoltageMapping(
        *mComponent, mnaVectorSize);

    // Complex history state h:
    //   i[k+1] = Y_C vIntf[k+1] + h[k]
    //   h[k+1] = -h[k] - (Y_C + conj(Y_C)) vIntf[k+1]
    // Therefore: AdLocal = -1, BdMna = -2 Re(Y_C) K,
    // CdMna = -K^T in real-imaginary augmented form.
    AdLocal.block(stateOffset, stateOffset, 2, 2) -= Matrix::Identity(2, 2);

    BdMna.block(stateOffset, 0, 2, mnaVectorSize) -=
        2.0 * conductance.real() * K;

    stampTwoTerminalCurrentInjectionMapping(K, CdMna, stateOffset,
                                            Matrix::Identity(2, 2));
  }

  void contributeMetadata(StateSpaceMetadata &metadata,
                          UInt stateOffset) const override {
    addSinglePhaseComplexStateMetadata(metadata, stateOffset,
                                       mComponent->name());
  }

private:
  std::shared_ptr<DP::Ph1::Capacitor> mComponent;
};

class DPPh1TwoTerminalVTypeSSNStateSpaceContributor final
    : public MNAStateSpaceContributor {
public:
  explicit DPPh1TwoTerminalVTypeSSNStateSpaceContributor(
      std::shared_ptr<DP::VTypeSSNComp> component)
      : mComponent(std::move(component)) {}

  UInt getStateCount() const override {
    return 2 * mComponent->getStateCount();
  }

  void stamp(Matrix &AdLocal, Matrix &BdMna, Matrix &CdMna, UInt stateOffset,
             UInt mnaVectorSize) const override {
    const UInt complexStateCount = mComponent->getStateCount();
    const UInt realStateCount = getStateCount();

    const MatrixComp &discreteA = mComponent->getDiscreteA();
    const MatrixComp &discreteB = mComponent->getDiscreteB();
    const MatrixComp outputC = mComponent->getC().cast<Complex>();

    const Matrix K = buildSinglePhaseComplexInterfaceVoltageMapping(
        *mComponent, mnaVectorSize);

    // Complex history-coordinate state s = discreteA x + discreteB vIntf:
    //   s[k+1] = discreteA s[k] + (discreteA + I) discreteB vIntf[k+1]
    //   yHist[k] = C s[k]
    // Therefore: AdLocal = discreteA, BdMna = (discreteA + I) discreteB K,
    // CdMna = -K^T C in real-imaginary augmented form.
    AdLocal.block(stateOffset, stateOffset, realStateCount, realStateCount) +=
        realAugment(discreteA);

    const MatrixComp inputUpdate =
        (discreteA +
         MatrixComp::Identity(complexStateCount, complexStateCount)) *
        discreteB;

    BdMna.block(stateOffset, 0, realStateCount, mnaVectorSize) +=
        realAugment(inputUpdate) * K;

    stampTwoTerminalCurrentInjectionMapping(K, CdMna, stateOffset,
                                            realAugment(outputC));
  }

  void contributeMetadata(StateSpaceMetadata &metadata,
                          UInt stateOffset) const override {
    addComplexStateMetadata(metadata, stateOffset, mComponent->getStateCount(),
                            mComponent->name());
  }

private:
  std::shared_ptr<DP::VTypeSSNComp> mComponent;
};

} // namespace

MNAStateSpaceContributor::Ptr
MNAStateSpaceContributorFactory::create(const MNAInterface::Ptr &component) {
  if (!component)
    return nullptr;

  if (auto inductor =
          std::dynamic_pointer_cast<EMT::Ph3::Inductor>(component)) {
    return std::make_shared<EMTPh3InductorStateSpaceContributor>(inductor);
  }

  if (auto capacitor =
          std::dynamic_pointer_cast<EMT::Ph3::Capacitor>(component)) {
    return std::make_shared<EMTPh3CapacitorStateSpaceContributor>(capacitor);
  }

  if (auto variableSsn =
          std::dynamic_pointer_cast<EMT::Ph3::TwoTerminalVTypeVariableSSNComp>(
              component)) {
    return std::make_shared<EMTPh3TwoTerminalVTypeSSNStateSpaceContributor>(
        variableSsn, true);
  }

  if (auto ssn = std::dynamic_pointer_cast<EMT::Ph3::TwoTerminalVTypeSSNComp>(
          component)) {
    return std::make_shared<EMTPh3TwoTerminalVTypeSSNStateSpaceContributor>(
        ssn, false);
  }

  if (std::dynamic_pointer_cast<EMT::Ph3::Resistor>(component))
    return nullptr;

  if (std::dynamic_pointer_cast<EMT::Ph3::VoltageSource>(component))
    return nullptr;

  if (auto inductor = std::dynamic_pointer_cast<DP::Ph1::Inductor>(component)) {
    return std::make_shared<DPPh1InductorStateSpaceContributor>(inductor);
  }

  if (auto capacitor =
          std::dynamic_pointer_cast<DP::Ph1::Capacitor>(component)) {
    return std::make_shared<DPPh1CapacitorStateSpaceContributor>(capacitor);
  }

  if (auto ssn = std::dynamic_pointer_cast<DP::Ph1::TwoTerminalVTypeSSNComp>(
          component)) {
    return std::make_shared<DPPh1TwoTerminalVTypeSSNStateSpaceContributor>(ssn);
  }

  if (std::dynamic_pointer_cast<DP::Ph1::Resistor>(component))
    return nullptr;

  if (std::dynamic_pointer_cast<DP::Ph1::VoltageSource>(component))
    return nullptr;

  throw std::invalid_argument(
      "Unsupported component in MNA state-space extraction.");
}

MNAStateSpaceContributor::List MNAStateSpaceContributorFactory::createList(
    const MNAInterface::List &components) {
  MNAStateSpaceContributor::List contributors;

  for (const auto &component : components) {
    auto contributor = create(component);
    if (contributor)
      contributors.push_back(contributor);
  }

  return contributors;
}

} // namespace DPsim
