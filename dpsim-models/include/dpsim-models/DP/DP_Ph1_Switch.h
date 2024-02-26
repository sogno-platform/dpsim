/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Switch.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief Dynamic phasor switch
///
/// The switch can be opened and closed.
/// Each state has a specific resistance value.
class Switch : public MNASimPowerComp<Complex>,
               public Base::Ph1::Switch,
               public SharedFactory<Switch>,
               public MNASwitchInterface {
protected:
public:
  /// Defines UID, name, component parameters and logging level
  Switch(String uid, String name, Logger::Level loglevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  Switch(String name, Logger::Level logLevel = Logger::Level::off)
      : Switch(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name);

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency);

  // #### General MNA section ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector);
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix);
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector);
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector);
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector);
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector);
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector);

  // #### MNA section for switch ####
  /// Check if switch is closed
  Bool mnaIsClosed();
  /// Stamps system matrix considering the defined switch position
  void mnaCompApplySwitchSystemMatrixStamp(Bool closed,
                                           SparseMatrixRow &systemMatrix,
                                           Int freqIdx);
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
