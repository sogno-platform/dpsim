/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
/// EMT ResIndSeries
/// Assumption: non diagonal terms are equal to zero, Z00=Z11=Z22
class ResIndSeries : public MNASimPowerComp<Real>,
                     public SharedFactory<ResIndSeries> {
protected:
  /// DC equivalent current source [A]
  Matrix mEquivCurrent;
  /// Equivalent conductance [S]
  Real mEquivCond;
  /// Coefficient in front of previous current value
  Real mPrevCurrFac;

public:
  /// Inductance [H]
  const Attribute<Matrix>::Ptr mInductance;
  ///Resistance [ohm]
  const Attribute<Matrix>::Ptr mResistance;
  /// Defines UID, name, component parameters and logging level
  ResIndSeries(String uid, String name,
               Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  ResIndSeries(String name, Logger::Level logLevel = Logger::Level::off)
      : ResIndSeries(name, name, logLevel) {}
  ///
  SimPowerComp<Real>::Ptr clone(String name);

  // #### General ####
  /// Sets model specific parameters
  void setParameters(Matrix resistanceMatrix, Matrix inductanceMatrix);

  /// Initializes auxiliar variables
  void initVars(Real timeStep);
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftSideVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// MNA pre step operations
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  /// MNA pre and post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// add MNA pre and post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph3
} // namespace EMT
} // namespace CPS