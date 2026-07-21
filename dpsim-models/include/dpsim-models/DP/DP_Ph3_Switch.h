// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Ph3_Switch.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {
/// \brief Dynamic phasor three-phase switch with per-phase resistance.
///
/// The switch can be opened and closed; each state has its own 3x3 resistance
/// matrix. An asymmetric closed matrix (e.g. one low diagonal entry) models a
/// single-phase fault. DP analogue of EMT::Ph3::Switch, driven by SwitchEvent3Ph.
class Switch : public MNASimPowerComp<Complex>,
               public Base::Ph3::Switch,
               public SharedFactory<Switch>,
               public MNAVariableCompInterface,
               public MNASwitchInterface {
public:
  /// Defines UID, name, component parameters and logging level
  Switch(String uid, String name, Logger::Level loglevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  Switch(String name, Logger::Level logLevel = Logger::Level::off)
      : Switch(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### General MNA section ####
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

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  // #### MNA section for variable component ####
  Bool hasParameterChanged() override;
};
} // namespace Ph3
} // namespace DP
} // namespace CPS
