/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph3_PiLine.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/SP/SP_Ph3_Inductor.h>
#include <dpsim-models/SP/SP_Ph3_Resistor.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace SP {
namespace Ph3 {

class RxLine : public MNASimPowerComp<Real>,
               public Base::Ph3::PiLine,
               public SharedFactory<RxLine> {
protected:
  /// Voltage across the component [V]
  Matrix mVoltage;
  /// Current through the component [A]
  Matrix mCurrent;
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

  SimPowerComp<Real>::Ptr clone(String name);

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency);

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector);
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix);
  /// Stamps system matrix
  void mnaApplyInitialSystemMatrixStamp(SparseMatrixRow &systemMatrix);
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector);
  void mnaCompUpdateVoltage(const Matrix &leftVector);
  void mnaCompUpdateCurrent(const Matrix &leftVector);

  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                     AttributeBase::List &attributeDependencies,
                                     AttributeBase::List &modifiedAttributes) {
    mAttributeDependencies.push_back(
        line.mSubResistor->attribute("right_vector"));
    mAttributeDependencies.push_back(
        line.mSubInductor->attribute("right_vector"));
    mModifiedAttributes.push_back(line.attribute("right_vector"));
  }

  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) {
    mAttributeDependencies.push_back(leftVector);
    mAttributeDependencies.push_back(line.mSubInductor->attribute("i_intf"));
    mModifiedAttributes.push_back(line.attribute("i_intf"));
    mModifiedAttributes.push_back(line.attribute("v_intf"));
  }
};
} // namespace Ph3
} // namespace SP
} // namespace CPS
