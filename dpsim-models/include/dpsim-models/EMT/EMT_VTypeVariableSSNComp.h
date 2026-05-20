// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace EMT {

/// \brief Abstract base class for variable EMT V-type SSN components.
///
/// This class extends VTypeSSNComp with support for time-varying or
/// piecewise-linear state-space models whose MNA conductance stamp may change
/// during simulation. Affine state and output terms
///
///   x' = A x + B u + E
///   y  = C x + D u + F
///
/// are supported in addition to the linear state-space model.
class VTypeVariableSSNComp : public VTypeSSNComp,
                             public MNAVariableCompInterface {
private:
  Bool mParameterChanged;
  Matrix mE;
  Matrix mdE;
  Matrix mF;

protected:
  static constexpr Int mInitializationMaxIterations = 10;
  static constexpr Real mInitializationTolerance = 1e-9;

  VTypeVariableSSNComp(String uid, String name, Int inputSize, Int outputSize,
                       Logger::Level logLevel = Logger::Level::off);

  Matrix calculateHistoryVector() const override final;
  MatrixComp calculateSteadyStateStateFromInput(const MatrixComp &u,
                                                Real frequency) const override;
  MatrixComp
  calculateSteadyStateOutputFromInput(const MatrixComp &x,
                                      const MatrixComp &u) const override;
  void updateState(const Matrix &uOld, const Matrix &uNew) override;
  void recomputeDiscreteModel() override;
  void updateStateSpaceModel() override final;

  const Matrix &stateOffset() const;
  const Matrix &outputOffset() const;

  void setStateOffset(const Matrix &E);
  void setOutputOffset(const Matrix &F);

  void setParameters(const Matrix &A, const Matrix &B, const Matrix &C,
                     const Matrix &D);
  void setParameters(const Matrix &A, const Matrix &B, const Matrix &C,
                     const Matrix &D, const Matrix &E);
  void setParameters(const Matrix &A, const Matrix &B, const Matrix &C,
                     const Matrix &D, const Matrix &E, const Matrix &F);

  /// Update the current stepwise-linearized component model.
  /// Returns true if the MNA conductance stamp changed.
  virtual Bool updateComponentParameters() = 0;

public:
  Bool hasParameterChanged() override final;

  void initializeFromNodesAndTerminals(Real frequency) override;
};

} // namespace EMT
} // namespace CPS
