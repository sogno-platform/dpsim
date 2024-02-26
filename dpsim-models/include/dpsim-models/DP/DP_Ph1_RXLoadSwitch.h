/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/DP/DP_Ph1_RXLoad.h>
#include <dpsim-models/DP/DP_Ph1_Switch.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// Constant impedance load model consisting of RLC elements
class RXLoadSwitch : public CompositePowerComp<Complex>,
                     public MNASwitchInterface,
                     public SharedFactory<RXLoadSwitch> {
protected:
  /// Internal RXLoad
  std::shared_ptr<DP::Ph1::RXLoad> mSubRXLoad;
  /// Internal protection switch
  std::shared_ptr<DP::Ph1::Switch> mSubSwitch;
  /// internal switch is only opened after this time offset
  Real mSwitchTimeOffset = 1.0;
  /// Right side vectors of subcomponents
  std::vector<const Matrix *> mRightVectorStamps;

public:
  /// Defines UID, name and logging level
  RXLoadSwitch(String uid, String name,
               Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  RXLoadSwitch(String name, Logger::Level logLevel = Logger::Level::off);

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency);
  /// Sets model specific parameters
  void setParameters(Real activePower, Real reactivePower, Real nomVolt,
                     Real openResistance, Real closedResistance,
                     Bool closed = false);
  /// Sets only switch parameters so that load parameters could be calculated from powerflow
  void setSwitchParameters(Real openResistance, Real closedResistance,
                           Bool closed = false);
  /// built-in logic for protection switch
  void updateSwitchState(Real time);

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

  // #### MNA section for switch ####
  /// Check if switch is closed
  Bool mnaIsClosed() override;
  /// Stamps system matrix considering the defined switch position
  void mnaCompApplySwitchSystemMatrixStamp(Bool closed,
                                           SparseMatrixRow &systemMatrix,
                                           Int freqIdx) override;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
