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
namespace Ph3 {
/// \brief Dynamic phasor three-phase switch
///
/// The switch can be opened and closed.
/// Each state has a specific resistance value.
/// For this model, the resistance model is the same for all phases and
/// only in series.
class SeriesSwitch : public MNASimPowerComp<Complex>,
                     public Base::Ph1::Switch,
                     public SharedFactory<SeriesSwitch>,
                     public MNASwitchInterface {

public:
  /// Defines UID, name and logging level
  SeriesSwitch(String uid, String name,
               Logger::Level loglevel = Logger::Level::off);
  /// Defines name and logging level
  SeriesSwitch(String name, Logger::Level logLevel = Logger::Level::off)
      : SeriesSwitch(name, name, logLevel) {}

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### General MNA section ####
  ///
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  // #### MNA section for switches ####
  /// Check if switch is closed
  Bool mnaIsClosed() override;
  /// Stamps system matrix considering the defined switch position
  void mnaCompApplySwitchSystemMatrixStamp(Bool closed,
                                           SparseMatrixRow &systemMatrix,
                                           Int freqIdx) override;

  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph3
} // namespace DP
} // namespace CPS
