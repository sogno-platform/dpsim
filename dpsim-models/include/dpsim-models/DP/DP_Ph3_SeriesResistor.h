/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Resistor.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {
///
class SeriesResistor : public MNASimPowerComp<Complex>,
                       public Base::Ph1::Resistor,
                       public SharedFactory<SeriesResistor> {

public:
  /// Defines UID, name and logging level
  SeriesResistor(String uid, String name,
                 Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  SeriesResistor(String name, Logger::Level logLevel = Logger::Level::off)
      : SeriesResistor(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name);

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency);
  // #### MNA section ####
  ///
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector);
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix);
  ///
  void mnaCompUpdateVoltage(const Matrix &leftVector);
  ///
  void mnaCompUpdateCurrent(const Matrix &leftVector);

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
