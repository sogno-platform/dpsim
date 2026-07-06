// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {

/// Single-phase, two-terminal, variable V-type SSN base with a mixed
/// real + complex-envelope state, packed as one real vector.
class MixedVTypeVariableSSNComp : public MNASimPowerComp<Complex>,
                                  public MNAVariableCompInterface {
private:
  Bool mParameterChanged;

protected:
  static constexpr Int mInitializationMaxIterations = 10;
  static constexpr Real mInitializationTolerance = 1e-9;

  const Int mRealStateCount;
  const Int mComplexStateCount;

  Real mTimeStep;

  /// Continuous-time real model over the packed state and packed [Re,Im] input/output.
  Matrix mA, mB, mC, mD, mE, mF;
  /// Discretized (trapezoidal) real operators.
  Matrix mdA, mdB, mdE;

  /// Complex Norton admittance / history current stamped into the network.
  Complex mW;
  Complex mYHist;

  /// Packed real state: [realStates..., Re(cplxState0), Im(cplxState0), ...].
  const Attribute<Matrix>::Ptr mX;

  MixedVTypeVariableSSNComp(String uid, String name, Int realStateCount,
                            Int complexStateCount,
                            Logger::Level logLevel = Logger::Level::off);

  /// Total packed real state size: realStateCount + 2*complexStateCount.
  Int stateSize() const;

  static Matrix packComplex(const Complex &c);
  static Complex unpackComplex(const Matrix &v);

  Attribute<MatrixComp>::Ptr inputAttribute() const;
  Attribute<MatrixComp>::Ptr outputAttribute() const;

  /// Default: v = v_terminal1 - v_terminal0.
  virtual MatrixComp buildInitialInputFromNodes(Real frequency);

  void setParameters(const Matrix &A, const Matrix &B, const Matrix &C,
                     const Matrix &D);
  void setParameters(const Matrix &A, const Matrix &B, const Matrix &C,
                     const Matrix &D, const Matrix &E, const Matrix &F);

  const Matrix &stateOffset() const;
  const Matrix &outputOffset() const;
  void setStateOffset(const Matrix &E);
  void setOutputOffset(const Matrix &F);

  virtual Matrix calculateHistoryVectorReal() const;
  virtual void updateState(const Complex &uOld, const Complex &uNew);
  virtual void recomputeDiscreteModel();

  /// Rebuild A/B/C/D/E/F from the current state/input; returns true if the stamp changed.
  virtual Bool updateComponentParameters() = 0;
  void updateStateSpaceModel();

public:
  Bool hasParameterChanged() override final;

  /// Fixed-point steady-state init; valid only when mRealStateCount == 0, override otherwise.
  void initializeFromNodesAndTerminals(Real frequency) override;

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

  void
  mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override final;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override final;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override final;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override final;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override final;
};

} // namespace Ph1
} // namespace DP
} // namespace CPS
