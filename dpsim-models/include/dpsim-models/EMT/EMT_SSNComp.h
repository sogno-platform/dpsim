// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {

/// \brief Abstract base class for EMT state-space nodal (SSN) components.
///
/// This class implements the common linear SSN logic shared by derived
/// component types, including state-space parameter storage, trapezoidal
/// discretization, steady-state initialization helpers, history-vector
/// calculation, state update, and generic MNA scheduling.
///
/// The exact interpretation of input and output quantities is defined in
/// derived classes.
class SSNComp : public MNASimPowerComp<Real> {
private:
  const Int mInputSize;
  const Int mOutputSize;

protected:
  Real mTimeStep;

  Matrix mW;
  Matrix mYHist;

  Matrix mA;
  Matrix mB;
  Matrix mC;
  Matrix mD;

  Matrix mdA;
  Matrix mdB;

  const Attribute<Matrix>::Ptr mX;

  SSNComp(String uid, String name, Int inputSize, Int outputSize,
          Logger::Level logLevel = Logger::Level::off);

  virtual Matrix calculateHistoryVector() const;

  MatrixComp calculateSteadyStateStateFromInput(const MatrixComp &u,
                                                Real frequency) const;
  MatrixComp calculateSteadyStateOutputFromInput(const MatrixComp &x,
                                                 const MatrixComp &u) const;

  void updateState(const Matrix &uOld, const Matrix &uNew);

  void recomputeDiscreteModel();

  /// Hook for variable/time-varying SSN components.
  virtual void updateStateSpaceModel();

  virtual Attribute<Matrix>::Ptr inputAttribute() const = 0;
  virtual Attribute<Matrix>::Ptr outputAttribute() const = 0;

public:
  void setParameters(const Matrix &A, const Matrix &B, const Matrix &C,
                     const Matrix &D);

  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override final;

  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override final;

  void mnaCompPreStep(Real time, Int timeStepCount) override final;

  void mnaCompAddPostStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes,
      Attribute<Matrix>::Ptr &leftVector) override final;
};

} // namespace EMT
} // namespace CPS
