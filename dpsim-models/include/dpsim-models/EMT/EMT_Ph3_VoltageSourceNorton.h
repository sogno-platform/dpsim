/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <dpsim-models/Base/Base_Ph1_VoltageSource.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
/// \brief Voltage source with Norton equivalent model
class VoltageSourceNorton : public MNASimPowerComp<Real>,
                            public Base::Ph1::VoltageSource,
                            public SharedFactory<VoltageSourceNorton> {
protected:
  void updateState(Real time);

  /// Equivalent current source [A]
  Matrix mEquivCurrent = Matrix::Zero(3, 1);

  //  ### Real Voltage source parameters ###
  /// conductance [S]
  Real mConductance;

public:
  /// Resistance [ohm]
  const Attribute<Real>::Ptr mResistance;
  /// Defines UID, name and logging level
  VoltageSourceNorton(String uid, String name,
                      Logger::Level logLevel = Logger::Level::off);
  ///
  VoltageSourceNorton(String name, Logger::Level logLevel = Logger::Level::off)
      : VoltageSourceNorton(name, name, logLevel) {}

  // #### General ####
  void setParameters(Complex voltageRef, Real srcFreq, Real resistance);
  ///
  void setVoltageRef(Complex voltage) const;

  SimPowerComp<Real>::Ptr clone(String name) override;
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override {}

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Returns current through the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  void mnaCompPreStep(Real time, Int timeStepCount) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;

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
