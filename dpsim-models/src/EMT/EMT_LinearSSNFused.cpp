// SPDX-License-Identifier: MPL-2.0
#include <dpsim-models/EMT/EMT_LinearSSNFused.h>

#include <dpsim-models/EMT/EMT_Ph3_SSN_ImpedanceLoad.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_PiLine.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_Transformer.h>
#include <dpsim-models/MNAStampUtils.h>

#include <stdexcept>

using namespace CPS;

namespace {
using PhaseVector = Eigen::Matrix<Real, 3, 1>;
}

EMT::LinearSSNFused::LinearSSNFused(String name, ComponentList components,
                                    SimNode::List nodes, Logger::Level level)
    : MNASimPowerComp<Real>(name, true, true, level),
      mComponents(std::move(components)) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(static_cast<UInt>(nodes.size()));
  connect(std::move(nodes));
  mFusedState = mAttributes->create<Matrix>("fused_state");
  mFusedState->get() = Matrix::Zero(1, 1);
}

void EMT::LinearSSNFused::initializeFromNodesAndTerminals(Real frequency) {
  for (const auto &component : mComponents)
    component->initializeFromNodesAndTerminals(frequency);
}

Real EMT::LinearSSNFused::vectorValue(const Matrix &vector, UInt index) {
  return Math::realFromVectorElement(vector, index);
}

void EMT::LinearSSNFused::addVectorValue(Matrix &vector, UInt index,
                                         Real value) {
  Math::addToVectorElement(vector, index, value);
}

void EMT::LinearSSNFused::setDeviceHistories(
    const std::vector<Matrix> &histories) {
  if (histories.size() != mHistories.size())
    throw std::invalid_argument(
        "LinearSSNFused device history count does not match its models.");
  mHistories = histories;
}

void EMT::LinearSSNFused::mnaCompInitialize(Real omega, Real timeStep,
                                            Attribute<Matrix>::Ptr leftVector) {
  mModels.clear();
  mHistories.clear();
  mModels.reserve(mComponents.size());
  mHistories.reserve(mComponents.size());

  for (const auto &component : mComponents) {
    // Let the original component perform one-time parameter/discretization
    // setup.  No child lifecycle method is used after this extraction point.
    component->mnaInitialize(omega, timeStep, leftVector);
    LinearSSNModel model;
    if (auto line = std::dynamic_pointer_cast<Ph3::SSN::PiLine>(component))
      line->exportFusedModel(model);
    else if (auto load =
                 std::dynamic_pointer_cast<Ph3::SSN::ImpedanceLoad>(component))
      load->exportFusedModel(model);
    else if (auto transformer =
                 std::dynamic_pointer_cast<Ph3::SSN::Transformer>(component))
      transformer->exportFusedModel(model);
    else
      throw std::invalid_argument(
          "LinearSSNFused received an unsupported component type.");
    mModels.push_back(std::move(model));
    mHistories.emplace_back(
        mModels.back().kind == LinearSSNModel::Kind::PiLine
            ? 9
            : (mModels.back().kind == LinearSSNModel::Kind::ImpedanceLoad ? 6
                                                                          : 15),
        1);
  }
  mModelsReady = true;
}

void EMT::LinearSSNFused::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (!mModelsReady)
    throw std::logic_error("LinearSSNFused model used before initialization.");
  for (const auto &model : mModels) {
    const UInt dimension = model.terminalCount * 3;
    for (UInt row = 0; row < dimension; ++row) {
      const UInt rowTerminal = row / 3;
      if (model.grounded[rowTerminal])
        continue;
      for (UInt col = 0; col < dimension; ++col) {
        const UInt colTerminal = col / 3;
        if (model.grounded[colTerminal])
          continue;
        Math::addToMatrixElement(systemMatrix, model.nodeIndices[row],
                                 model.nodeIndices[col],
                                 model.portAdmittance(row, col));
      }
    }
  }
}

void EMT::LinearSSNFused::updateHistories() {
  for (UInt modelIndex = 0; modelIndex < mModels.size(); ++modelIndex) {
    auto &model = mModels[modelIndex];
    auto &history = mHistories[modelIndex];
    switch (model.kind) {
    case LinearSSNModel::Kind::PiLine: {
      const auto current = model.state.block<3, 1>(0, 0);
      const auto inductorVoltage = model.state.block<3, 1>(3, 0);
      const auto capVoltage0 = model.state.block<3, 1>(6, 0);
      const auto capCurrent0 = model.state.block<3, 1>(9, 0);
      const auto capVoltage1 = model.state.block<3, 1>(12, 0);
      const auto capCurrent1 = model.state.block<3, 1>(15, 0);
      const Matrix inductorHistory =
          model.inductorAdmittance * inductorVoltage + current;
      history.block<3, 1>(0, 0).noalias() =
          model.seriesSystemInverse * inductorHistory;
      history.block<3, 1>(3, 0).noalias() =
          -capCurrent0 - model.capAdmittance * capVoltage0;
      history.block<3, 1>(6, 0).noalias() =
          -capCurrent1 - model.capAdmittance * capVoltage1;
      break;
    }
    case LinearSSNModel::Kind::ImpedanceLoad: {
      history.block<3, 1>(0, 0).setZero();
      history.block<3, 1>(3, 0).setZero();
      if (model.hasLoadInductance)
        for (UInt phase = 0; phase < 3; ++phase)
          history(phase) = model.inductorAdmittanceScalar * model.state(phase) +
                           model.state(3 + phase);
      if (model.hasLoadCapacitance)
        for (UInt phase = 0; phase < 3; ++phase)
          history(3 + phase) =
              -model.state(9 + phase) -
              model.capacitorAdmittanceScalar * model.state(6 + phase);
      break;
    }
    case LinearSSNModel::Kind::Transformer: {
      const Matrix h0 =
          model.seriesSystemInverse *
          (model.inductorAdmittance * model.state.block<3, 1>(0, 0) +
           model.state.block<3, 1>(3, 0));
      const Matrix h1 =
          model.seriesSystemInverse *
          (model.inductorAdmittance * model.state.block<3, 1>(6, 0) +
           model.state.block<3, 1>(9, 0));
      const Matrix hMag =
          model.magnetizingInductorAdmittance * model.state.block<3, 1>(12, 0) +
          model.state.block<3, 1>(15, 0);
      const Matrix midpointHistory = -h0 + h1 + hMag;
      const Matrix midpointSource = model.midpointInverse * midpointHistory;
      history.block<3, 1>(0, 0) = h0;
      history.block<3, 1>(3, 0) = h1;
      history.block<3, 1>(6, 0) = hMag;
      history.block<3, 1>(9, 0) = h0 + model.seriesAdmittance * midpointSource;
      history.block<3, 1>(12, 0) =
          model.ratioFromReference *
          (model.seriesAdmittance * midpointSource - h1);
      break;
    }
    }
  }
}

void EMT::LinearSSNFused::addHistoryStamp(Matrix &rightVector) const {
  for (UInt modelIndex = 0; modelIndex < mModels.size(); ++modelIndex) {
    const auto &model = mModels[modelIndex];
    const auto &history = mHistories[modelIndex];
    if (model.kind == LinearSSNModel::Kind::PiLine) {
      if (!model.grounded[0])
        for (UInt phase = 0; phase < 3; ++phase)
          addVectorValue(rightVector, model.nodeIndices[phase],
                         history(phase) - history(3 + phase));
      if (!model.grounded[1])
        for (UInt phase = 0; phase < 3; ++phase)
          addVectorValue(rightVector, model.nodeIndices[3 + phase],
                         -history(phase) - history(6 + phase));
    } else if (model.kind == LinearSSNModel::Kind::ImpedanceLoad) {
      if (!model.grounded[0])
        for (UInt phase = 0; phase < 3; ++phase)
          addVectorValue(rightVector, model.nodeIndices[phase],
                         -(history(phase) + history(3 + phase)));
    } else {
      const UInt ref = model.referenceTerminal;
      const UInt other = 1 - ref;
      if (!model.grounded[ref])
        for (UInt phase = 0; phase < 3; ++phase)
          addVectorValue(rightVector, model.nodeIndices[ref * 3 + phase],
                         history(9 + phase));
      if (!model.grounded[other])
        for (UInt phase = 0; phase < 3; ++phase)
          addVectorValue(rightVector, model.nodeIndices[other * 3 + phase],
                         history(12 + phase));
    }
  }
}

void EMT::LinearSSNFused::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  for (const auto &component : mComponents) {
    auto childRightVector = component->getRightVector();
    childRightVector->get().setZero();
    component->mnaApplyRightSideVectorStamp(childRightVector->get());
    rightVector += childRightVector->get();
  }
}

void EMT::LinearSSNFused::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  if (mDeviceRhsManaged)
    return;
  addHistoryStamp(rightVector);
}

void EMT::LinearSSNFused::mnaCompPreStep(Real, Int) {
  if (!mDeviceManaged)
    updateHistories();
}

void EMT::LinearSSNFused::updateStates(const Matrix &leftVector) {
  for (UInt modelIndex = 0; modelIndex < mModels.size(); ++modelIndex) {
    auto &model = mModels[modelIndex];
    if (model.kind == LinearSSNModel::Kind::PiLine) {
      PhaseVector v0 = PhaseVector::Zero(), v1 = PhaseVector::Zero();
      for (UInt phase = 0; phase < 3; ++phase) {
        if (!model.grounded[0])
          v0(phase) = vectorValue(leftVector, model.nodeIndices[phase]);
        if (!model.grounded[1])
          v1(phase) = vectorValue(leftVector, model.nodeIndices[3 + phase]);
      }
      const PhaseVector interfaceVoltage = v1 - v0;
      const PhaseVector current = model.seriesAdmittance * interfaceVoltage +
                                  mHistories[modelIndex].block(0, 0, 3, 1);
      const PhaseVector inductorVoltage =
          interfaceVoltage - model.seriesResistance * current;
      const PhaseVector capCurrent0 =
          model.capAdmittance * v0 + mHistories[modelIndex].block(3, 0, 3, 1);
      const PhaseVector capCurrent1 =
          model.capAdmittance * v1 + mHistories[modelIndex].block(6, 0, 3, 1);
      model.state.block<3, 1>(0, 0) = current;
      model.state.block<3, 1>(3, 0) = inductorVoltage;
      model.state.block<3, 1>(6, 0) = v0;
      model.state.block<3, 1>(9, 0) = capCurrent0;
      model.state.block<3, 1>(12, 0) = v1;
      model.state.block<3, 1>(15, 0) = capCurrent1;
    } else if (model.kind == LinearSSNModel::Kind::ImpedanceLoad) {
      PhaseVector voltage = PhaseVector::Zero();
      if (!model.grounded[0])
        for (UInt phase = 0; phase < 3; ++phase)
          voltage(phase) = vectorValue(leftVector, model.nodeIndices[phase]);
      const auto &history = mHistories[modelIndex];
      for (UInt phase = 0; phase < 3; ++phase) {
        if (model.hasLoadInductance) {
          model.state(phase) = voltage(phase);
          model.state(3 + phase) =
              model.inductorAdmittanceScalar * voltage(phase) + history(phase);
        }
        if (model.hasLoadCapacitance) {
          model.state(6 + phase) = voltage(phase);
          model.state(9 + phase) =
              model.capacitorAdmittanceScalar * voltage(phase) +
              history(3 + phase);
        }
      }
    } else {
      const auto &history = mHistories[modelIndex];
      PhaseVector v0 = PhaseVector::Zero(), v1 = PhaseVector::Zero();
      for (UInt phase = 0; phase < 3; ++phase) {
        if (!model.grounded[0])
          v0(phase) = vectorValue(leftVector, model.nodeIndices[phase]);
        if (!model.grounded[1])
          v1(phase) = vectorValue(leftVector, model.nodeIndices[3 + phase]);
      }
      const UInt referenceTerminal = model.referenceTerminal;
      const UInt otherTerminal = 1 - referenceTerminal;
      const PhaseVector referenceVoltage = referenceTerminal == 0 ? v0 : v1;
      const PhaseVector referredOtherVoltage =
          model.ratioFromReference * (otherTerminal == 0 ? v0 : v1);
      const PhaseVector midpointHistory = -history.block(0, 0, 3, 1) +
                                          history.block(3, 0, 3, 1) +
                                          history.block(6, 0, 3, 1);
      const PhaseVector midpointVoltage =
          model.midpointInverse *
          (model.seriesAdmittance * referenceVoltage +
           model.seriesAdmittance * referredOtherVoltage + midpointHistory);
      const PhaseVector totalVoltage0 = midpointVoltage - referenceVoltage;
      const PhaseVector current0 =
          model.seriesAdmittance * totalVoltage0 + history.block(0, 0, 3, 1);
      const PhaseVector voltageL0 =
          totalVoltage0 - model.seriesResistanceHalf * current0;
      const PhaseVector totalVoltage1 = referredOtherVoltage - midpointVoltage;
      const PhaseVector current1 =
          model.seriesAdmittance * totalVoltage1 + history.block(3, 0, 3, 1);
      const PhaseVector voltageL1 =
          totalVoltage1 - model.seriesResistanceHalf * current1;
      const PhaseVector voltageLMag = -midpointVoltage;
      const PhaseVector currentMagnetizing =
          model.magnetizingInductorAdmittance * voltageLMag +
          history.block(6, 0, 3, 1);
      model.state.block<3, 1>(0, 0) = voltageL0;
      model.state.block<3, 1>(3, 0) = current0;
      model.state.block<3, 1>(6, 0) = voltageL1;
      model.state.block<3, 1>(9, 0) = current1;
      model.state.block<3, 1>(12, 0) = voltageLMag;
      model.state.block<3, 1>(15, 0) = currentMagnetizing;
    }
  }
}

void EMT::LinearSSNFused::mnaCompPostStep(Real, Int timeStepCount,
                                          Attribute<Matrix>::Ptr &leftVector) {
  if (!mDeviceManaged)
    updateStates(**leftVector);
  if (!mDeviceManaged)
    mFusedState->get()(0, 0) = static_cast<Real>(timeStepCount);
}

void EMT::LinearSSNFused::mnaCompUpdateVoltage(const Matrix &) {}
void EMT::LinearSSNFused::mnaCompUpdateCurrent(const Matrix &) {}

void EMT::LinearSSNFused::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies, AttributeBase::List &,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mFusedState);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::LinearSSNFused::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  (void)prevStepDependencies;
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mFusedState);
}
