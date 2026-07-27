// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// Three-phase, two-terminal, variable V-type SSN base with a mixed
/// real + per-phase complex-envelope state, packed as one real vector.
///
/// Ph3 analogue of DP::Ph1::MixedVTypeVariableSSNComp: the input/output are
/// three complex phase envelopes (not one), so mW is a 3x3 complex Norton
/// admittance and mYHist a 3x1 history current, stamped with
/// MNAStampUtils::stampAdmittanceMatrix exactly as the fixed-parameter
/// DP::Ph3::TwoTerminalVTypeSSNComp does. Each phase's complex self term is
/// shifted by the same single carrier (A - jw_N I per phase); the per-phase
/// coupling and any envelope shift are the derived component's own A/B, baked
/// in pre-shifted (do not double-shift, [[reference-mixedvtype-ssn-preshifted-a]]).
class MixedVTypeVariableSSNComp : public MNASimPowerComp<Complex>,
                                  public MNAVariableCompInterface {
private:
  Bool mParameterChanged;

protected:
  static constexpr Int mInitializationMaxIterations = 10;
  static constexpr Real mInitializationTolerance = 1e-9;

  /// Number of complex network channels (phases) this base folds to/from.
  static constexpr Int mPhaseCount = 3;

  const Int mRealStateCount;
  const Int mComplexStateCount;

  Real mTimeStep;

  /// Continuous-time real model over the packed state and packed [Re,Im] input/output.
  Matrix mA, mB, mC, mD, mE, mF;
  /// Discretized (trapezoidal) real operators.
  Matrix mdA, mdB, mdE;

  /// Complex Norton admittance / history current stamped into the network.
  MatrixComp mW;
  MatrixComp mYHist;

  /// Packed real state: [realStates..., Re(cplxState0), Im(cplxState0), ...].
  const Attribute<Matrix>::Ptr mX;

  MixedVTypeVariableSSNComp(String uid, String name, Int realStateCount,
                            Int complexStateCount,
                            Logger::Level logLevel = Logger::Level::off);

  /// Total packed real state size: realStateCount + 2*complexStateCount.
  Int stateSize() const;

  /// Pack an m-vector of complex into a 2m real vector [Re0,Im0,Re1,Im1,...].
  static Matrix packComplex(const MatrixComp &c);
  /// Inverse of packComplex.
  static MatrixComp unpackComplex(const Matrix &v);
  /// Fold a 2m x 2m real block-[[a,-b],[b,a]] matrix into an m x m complex map.
  static MatrixComp foldComplexMatrix(const Matrix &real);

  Attribute<MatrixComp>::Ptr inputAttribute() const;
  Attribute<MatrixComp>::Ptr outputAttribute() const;

  /// Default: balanced envelope from v_terminal1 - v_terminal0.
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
  virtual void updateState(const MatrixComp &uOld, const MatrixComp &uNew);
  virtual void recomputeDiscreteModel();

  /// Update derived attributes used for logging/inspection; called once per
  /// step after the state update. Empty by default, override in derived
  /// components (mirrors EMT::SSNComp::updateLogAttributes).
  virtual void updateLogAttributes(const Matrix &u) const;

  /// Rebuild A/B/C/D/E/F from the current state/input; returns true if the stamp changed.
  virtual Bool updateComponentParameters() = 0;
  void updateStateSpaceModel();

public:
  UInt getStateCount() const;
  const Matrix &getDiscreteA() const;
  const Matrix &getDiscreteB() const;
  const Matrix &getC() const;

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

} // namespace Ph3
} // namespace DP
} // namespace CPS
