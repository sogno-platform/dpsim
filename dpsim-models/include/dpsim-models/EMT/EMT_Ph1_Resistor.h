/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>

#include <dpsim-models/Base/Base_Ph1_Resistor.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/EigenvalueCompInterface.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
/// EMT Resistor
class Resistor : public MNASimPowerComp<Real>,
                 public Base::Ph1::Resistor,
                 public SharedFactory<Resistor>,
                 public EigenvalueCompInterface {
protected:
public:
  /// Defines UID, name, component parameters and logging level
  Resistor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  Resistor(String name, Logger::Level logLevel = Logger::Level::off)
      : Resistor(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftSideVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override {}
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  // #### Implementation of eigenvalue component interface ####
  void stampBranchNodeIncidenceMatrix(UInt branchIdx,
                                      Matrix &branchNodeIncidenceMatrix) final;
};
} // namespace Ph1
} // namespace EMT
} // namespace CPS
