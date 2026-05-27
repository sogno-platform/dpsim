// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim/MNAStateSpaceContributor.h>

#include <dpsim-models/EMT/EMT_Ph3_Capacitor.h>
#include <dpsim-models/EMT/EMT_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeSSNComp.h>
#include <dpsim-models/EMT/EMT_Ph3_VoltageSource.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

#include <stdexcept>
#include <utility>

using namespace CPS;

namespace DPsim {
namespace {

using SimPowerCompReal = CPS::SimPowerComp<CPS::Real>;

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

Bool isVariableMNAComponent(const MNAInterface::Ptr &component) {
  return std::dynamic_pointer_cast<MNAVariableCompInterface>(component) !=
         nullptr;
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

private:
  std::shared_ptr<EMT::Ph3::Capacitor> mComponent;
};

class EMTPh3TwoTerminalVTypeSSNStateSpaceContributor final
    : public MNAStateSpaceContributor {
public:
  EMTPh3TwoTerminalVTypeSSNStateSpaceContributor(
      std::shared_ptr<EMT::Ph3::TwoTerminalVTypeSSNComp> component,
      Bool isVariable)
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

private:
  std::shared_ptr<EMT::Ph3::TwoTerminalVTypeSSNComp> mComponent;
  Bool mIsVariable = false;
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

  if (auto ssn = std::dynamic_pointer_cast<EMT::Ph3::TwoTerminalVTypeSSNComp>(
          component)) {
    return std::make_shared<EMTPh3TwoTerminalVTypeSSNStateSpaceContributor>(
        ssn, isVariableMNAComponent(component));
  }

  if (std::dynamic_pointer_cast<EMT::Ph3::Resistor>(component))
    return nullptr;

  if (std::dynamic_pointer_cast<EMT::Ph3::VoltageSource>(component))
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
