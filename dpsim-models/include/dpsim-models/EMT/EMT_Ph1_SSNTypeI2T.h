// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
/// \brief SSNTypeI2T
/// Model for a one phase, two terminal I-type SSN component which can be represented using
/// a state space equation system
/// x' = A * x + B * u
/// y = C * x + D * u
/// with x: state vector, y: output vector, u: input vector,
/// where u represents external current (mIntfCurrent),
/// y represents external voltage (mIntfVoltage),
/// x represents any component states.
class SSNTypeI2T : public MNASimPowerComp<Real>,
                   public SharedFactory<SSNTypeI2T> {
private:
  void ssnUpdateState();
  void setSSNMatricesToZero();

protected:
  Matrix mX;
  Matrix mU;
  Matrix mUOld;
  Matrix mW;
  Matrix mYHist;

public:
  const CPS::Attribute<Matrix>::Ptr mA;
  const CPS::Attribute<Matrix>::Ptr mB;
  const CPS::Attribute<Matrix>::Ptr mC;
  const CPS::Attribute<Matrix>::Ptr mD;

  const CPS::Attribute<Matrix>::Ptr mdA;
  const CPS::Attribute<Matrix>::Ptr mdB;
  const CPS::Attribute<Matrix>::Ptr mdC;

  /// Defines UID, name, component parameters and logging level
  SSNTypeI2T(String uid, String name,
             Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  SSNTypeI2T(String name, Logger::Level logLevel = Logger::Level::off)
      : SSNTypeI2T(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;

  void manualInit(Matrix initialState, Matrix initialInput,
                  Matrix initialOldInput, Real initCurrent, Real initVoltage);
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
