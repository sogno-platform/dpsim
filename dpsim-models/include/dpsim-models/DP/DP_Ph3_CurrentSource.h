// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {
/// \brief Ideal current source model
///
/// This model uses modified nodal analysis to represent an ideal current source.
/// This involves the stamping of the current to the right side vector.
class CurrentSource : public MNASimPowerComp<Complex>,
                      public SharedFactory<CurrentSource> {
public:
  /// Reference current phasor, independent per phase (A, B, C)
  const Attribute<MatrixComp>::Ptr mCurrentRef;
  /// Defines UID, name and logging level
  CurrentSource(String uid, String name,
                Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  CurrentSource(String name, Logger::Level logLevel = Logger::Level::off)
      : CurrentSource(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;
  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
  /// Setter for reference current, one phasor per phase
  void setParameters(MatrixComp currentRef);

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Returns voltage through the component
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// MNA pre step operations
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph3
} // namespace DP
} // namespace CPS
