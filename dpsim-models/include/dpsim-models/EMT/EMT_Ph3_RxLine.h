/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph3_PiLine.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/EMT/EMT_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

class RxLine : public CompositePowerComp<Real>,
               public Base::Ph3::PiLine,
               public SharedFactory<RxLine> {
protected:
  /// True after createSubComponents() runs; prevents double-construction.
  bool mSubCompCreated = false;
  /// Inductance submodel
  std::shared_ptr<Inductor> mSubInductor;
  /// Resistor submodel
  std::shared_ptr<Resistor> mSubResistor;
  /// Inductor end to ground resistor to facilitate initialization
  std::shared_ptr<Resistor> mInitialResistor;

public:
  /// Defines UID, name, logging level
  RxLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  RxLine(String name, Logger::Level logLevel = Logger::Level::off)
      : RxLine(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;

  // #### General ####
  /// Constructs and registers MNA subcomponents; idempotent.
  void createSubComponents() override;
  /// Derives values from power flow data and pushes them to subcomponents
  void initializeParentFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;

  /// MNA pre and post step operations
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph3
} // namespace EMT
} // namespace CPS
