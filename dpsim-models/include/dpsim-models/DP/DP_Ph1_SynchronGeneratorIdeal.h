/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph1_VoltageSource.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// Ideal voltage source representing a synchronous generator
class SynchronGeneratorIdeal : public CompositePowerComp<Complex>,
                               public SharedFactory<SynchronGeneratorIdeal> {
private:
  /// Inner voltage source that represents the generator
  std::shared_ptr<DP::Ph1::VoltageSource> mSubVoltageSource;

public:
  /// Voltage set point [V]
  /// CHECK: Is this attribute necessary? It is not used in the component itself, only initialized
  const Attribute<Complex>::Ptr mVoltageRef;
  /// Defines UID, name, component parameters and logging level
  SynchronGeneratorIdeal(String uid, String name,
                         Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  SynchronGeneratorIdeal(String name,
                         Logger::Level logLevel = Logger::Level::off);

  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
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
  /// Updates current through the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// Updates voltage across component
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
