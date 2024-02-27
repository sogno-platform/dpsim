/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/EMT/EMT_Ph3_CurrentSource.h>
#include <dpsim-models/EMT/EMT_Ph3_VoltageSource.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
/// Ideal voltage source representing a synchronous generator
class SynchronGeneratorIdeal : public CompositePowerComp<Real>,
                               public SharedFactory<SynchronGeneratorIdeal> {
private:
  /// Specifies type of ideal source
  CPS::GeneratorType mSourceType;
  /// Inner voltage source that represents the generator
  std::shared_ptr<EMT::Ph3::VoltageSource> mSubVoltageSource;
  /// Inner voltage source that represents the generator
  std::shared_ptr<EMT::Ph3::CurrentSource> mSubCurrentSource;

public:
  /// CHECK: Is this actually necessary? It is never read from within the component's code
  const Attribute<MatrixComp>::Ptr mRefVoltage;

  /// Defines UID, name, component parameters and logging level
  SynchronGeneratorIdeal(
      String uid, String name, Logger::Level logLevel = Logger::Level::off,
      CPS::GeneratorType sourceType = CPS::GeneratorType::IdealVoltageSource);
  /// Defines name, component parameters and logging level
  SynchronGeneratorIdeal(String name,
                         Logger::Level logLevel = Logger::Level::off);

  SimPowerComp<Real>::Ptr clone(String name);

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency);

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
} // namespace Ph3
} // namespace EMT
} // namespace CPS
