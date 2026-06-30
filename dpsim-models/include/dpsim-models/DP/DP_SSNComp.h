// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {

/// \brief Abstract base class for DP/SFA state-space nodal (SSN) components.
///
/// DP counterpart of EMT::SSNComp. The SFA carrier shift uses the real-augmented
/// form A_aug = [[A, w I], [-w I, A]]; see the .cpp for how it folds back to the
/// complex envelope model. Single carrier only. Input/output meaning is fixed by
/// derived V-type / I-type classes.
class SSNComp : public MNASimPowerComp<Complex> {
private:
  const Int mInputSize;
  const Int mOutputSize;

protected:
  Real mTimeStep;

  /// Continuous-time real physical model (user-provided).
  Matrix mA;
  Matrix mB;
  Matrix mC;
  Matrix mD;

  /// Complex envelope discrete operators (folded from the augmented model).
  MatrixComp mDiscreteA;
  MatrixComp mDiscreteB;

  /// Complex Norton matrix W = C dB + D.
  MatrixComp mW;
  /// Complex history term.
  MatrixComp mYHist;

  /// Complex envelope state X.
  const Attribute<MatrixComp>::Ptr mX;

  SSNComp(String uid, String name, Int inputSize, Int outputSize,
          Logger::Level logLevel = Logger::Level::off);

  /// Build A_aug = [[A, w I], [-w I, A]].
  Matrix buildAugmentedA(Real omega) const;
  /// Build B_aug = blkdiag(B, B).
  Matrix buildAugmentedB() const;

  /// Discretize the augmented model and fold it into complex dA, dB, W.
  virtual void recomputeDiscreteModel(Real omega);

  virtual MatrixComp calculateHistoryVector() const;

  /// Steady-state envelope state X_ss = (j w I - A)^-1 B u.
  virtual MatrixComp calculateSteadyStateStateFromInput(const MatrixComp &u,
                                                        Real omega) const;
  virtual MatrixComp
  calculateSteadyStateOutputFromInput(const MatrixComp &x,
                                      const MatrixComp &u) const;

  virtual void updateState(const MatrixComp &uOld, const MatrixComp &uNew);

  /// Hook for variable/time-varying SSN components.
  virtual void updateStateSpaceModel();

  virtual Attribute<MatrixComp>::Ptr inputAttribute() const = 0;
  virtual Attribute<MatrixComp>::Ptr outputAttribute() const = 0;

public:
  /// Get number of internal state variables of the SSN model.
  UInt getStateCount() const;

  /// Get the complex discrete state-transition operator of the envelope model.
  const MatrixComp &getDiscreteA() const;
  /// Get the complex discrete input operator of the envelope model.
  const MatrixComp &getDiscreteB() const;
  /// Get the continuous-time output matrix of the SSN model.
  const Matrix &getC() const;

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

} // namespace DP
} // namespace CPS
