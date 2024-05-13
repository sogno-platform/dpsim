/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Resistor.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
/// @brief EMT three-phase resistor
///
/// This resistor has resistance values different from zero
/// only on the main diagonal. These values are identical.
class SeriesResistor : public MNASimPowerComp<Real>,
                       public Base::Ph1::Resistor,
                       public SharedFactory<SeriesResistor> {

public:
  /// Defines UID, name and log level
  SeriesResistor(String uid, String name,
                 Logger::Level logLevel = Logger::Level::off);
  /// Defines name and log level
  SeriesResistor(String name, Logger::Level logLevel = Logger::Level::off)
      : SeriesResistor(name, name, logLevel) {}

  // #### General ####
  /// Return new instance with the same parameters
  SimPowerComp<Real>::Ptr clone(String name) override;
  /// Initializes states from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Initializes MNA specific variables
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Update interface voltage from MNA system results
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface voltage from MNA system results
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph3
} // namespace EMT
} // namespace CPS
