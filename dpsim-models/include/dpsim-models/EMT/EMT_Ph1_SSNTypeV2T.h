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
namespace Ph1 {
/// \brief SSNTypeV2T
/// Model for a one phase, two terminal V-type SSN component which can be represented using
/// a state space equation system
/// x' = A * x + B * u
/// y = C * x + D * u
/// with x: state vector, y: output vector, u: input vector,
/// where u represents external voltage (mIntfVoltage),
/// y represents external current (mIntfCurrent),
/// x represents any component states.
class SSNTypeV2T : public MNASimPowerComp<Real>,
                   public SharedFactory<SSNTypeV2T> {
protected:
  Matrix mX;
  Matrix mU;
  Matrix mUOld;
  Matrix mW;
  Matrix mY_hist;

public:
  const CPS::Attribute<Matrix>::Ptr mA;
  const CPS::Attribute<Matrix>::Ptr mB;
  const CPS::Attribute<Matrix>::Ptr mC;
  const CPS::Attribute<Matrix>::Ptr mD;

  const CPS::Attribute<Matrix>::Ptr mdA;
  const CPS::Attribute<Matrix>::Ptr mdB;
  const CPS::Attribute<Matrix>::Ptr mdC;

  /// Defines UID, name, component parameters and logging level
  SSNTypeV2T(String uid, String name,
             Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  SSNTypeV2T(String name, Logger::Level logLevel = Logger::Level::off)
      : SSNTypeV2T(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;

  void manualInit(Matrix initialState, Matrix initialInput,
                  Matrix initialOldInput, Real initCurr, Real initVol);
  void ssnUpdateState();
  void setSSNMatricesToZero();
  // #### General ####
  void setParameters(const Matrix A, const Matrix B, const Matrix C,
                     const Matrix D);
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

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
  /// Update interface current from MNA system result
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
} // namespace Ph1
} // namespace EMT
} // namespace CPS
