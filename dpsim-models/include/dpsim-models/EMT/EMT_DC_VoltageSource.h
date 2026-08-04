// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>

namespace CPS {
namespace EMT {
namespace DC {

/// Ideal scalar DC voltage source.
///
/// v = v_terminal1 - v_terminal0. The MNA branch-current unknown is positive
/// from terminal 1 to terminal 0, so p = v * i follows the passive sign
/// convention.
class VoltageSource final : public MNASimPowerComp<Real>,
                            public SharedFactory<VoltageSource> {
public:
  using SharedFactory<VoltageSource>::make;

  const Attribute<Real>::Ptr mVoltageRef;

  VoltageSource(String uid, String name,
                Logger::Level logLevel = Logger::Level::off);
  VoltageSource(String name, Logger::Level logLevel = Logger::Level::off)
      : VoltageSource(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;
  void setParameters(Real voltage);
  void initializeFromNodesAndTerminals(Real frequency) override;

  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void
  mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override final;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override final;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override final;
  void mnaCompPreStep(Real time, Int timeStepCount) override final;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override final;
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override final;
  void mnaCompAddPostStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes,
      Attribute<Matrix>::Ptr &leftVector) override final;

private:
  void validateDCTerminals() const;
};

} // namespace DC
} // namespace EMT
} // namespace CPS
