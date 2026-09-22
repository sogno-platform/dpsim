/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph3_Resistor.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// \brief Three-phase dynamic-phasor resistor.
///
/// Three-phase DP interface quantities use phase-peak complex envelopes.
/// Power-flow node voltages are converted from line-line RMS to phase peak
/// during initialization.
class Resistor : public MNASimPowerComp<Complex>,
                 public Base::Ph3::Resistor,
                 public MNATearInterface,
                 public SharedFactory<Resistor> {
public:
  Resistor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);

  Resistor(String name, Logger::Level logLevel = Logger::Level::off)
      : Resistor(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initialize DP phase-peak envelopes from line-line RMS PF node voltages.
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Nothing here depends on the timestep.
  bool mnaUpdateTimeStep(Real timeStep) override { return timeStep > 0; }
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;

  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;

  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

  // #### MNA Tear Section ####
  void mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) override;
};

} // namespace Ph3
} // namespace DP
} // namespace CPS
