// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph3_VoltageSource.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {
/// \brief Network injection model
///
/// This model represents network injections by an ideal three-phase voltage
/// source. When no reference voltage is set, it is seeded from the power-flow
/// node voltage, so the component acts as a stiff slack.
class NetworkInjection : public CompositePowerComp<Complex>,
                         public SharedFactory<NetworkInjection> {
private:
  // ### Electrical Subcomponents ###
  /// Voltage source
  std::shared_ptr<DP::Ph3::VoltageSource> mSubVoltageSource;

  // #### solver ####
  /// Vector to collect subcomponent right vector stamps
  std::vector<const Matrix *> mRightVectorStamps;

public:
  const CPS::Attribute<MatrixComp>::Ptr mVoltageRef;
  const CPS::Attribute<Real>::Ptr mSrcFreq;

  /// Defines UID, name, component parameters and logging level
  NetworkInjection(String uid, String name,
                   Logger::Level loglevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  NetworkInjection(String name, Logger::Level logLevel = Logger::Level::off)
      : NetworkInjection(name, name, logLevel) {}
  ///
  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeParentFromNodesAndTerminals(Real frequency) override;
  /// Setter for reference voltage parameters
  void setParameters(MatrixComp voltageRef, Real srcFreq = 0.0);

  // #### MNA Section ####
  /// Stamps right side (source) vector
  void mnaParentApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Returns current through the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// Updates voltage across component
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// MNA pre step operations
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph3
} // namespace DP
} // namespace CPS
