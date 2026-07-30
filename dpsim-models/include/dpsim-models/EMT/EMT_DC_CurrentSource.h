// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>

namespace CPS {
namespace EMT {
namespace DC {

/// Ideal scalar DC current source.
///
/// Positive current flows from terminal 1 to terminal 0. The interface voltage
/// is v = v_terminal1 - v_terminal0, so p = v * i follows the passive sign
/// convention.
class CurrentSource final : public MNASimPowerComp<Real>,
                            public SharedFactory<CurrentSource> {
public:
  using SharedFactory<CurrentSource>::make;

  const Attribute<Real>::Ptr mCurrentRef;

  CurrentSource(String uid, String name,
                Logger::Level logLevel = Logger::Level::off);
  CurrentSource(String name, Logger::Level logLevel = Logger::Level::off)
      : CurrentSource(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;
  void setParameters(Real current);
  void initializeFromNodesAndTerminals(Real frequency) override;

  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &) override final {}
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override final;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override final;
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
