// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <array>
#include <vector>

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

  virtual MatrixComp calculateSteadyStateStateFromInput(const MatrixComp &u,
                                                        Real frequency) const;
  virtual MatrixComp
  calculateSteadyStateOutputFromInput(const MatrixComp &x,
                                      const MatrixComp &u) const;

  virtual void updateState(const Matrix &uOld, const Matrix &uNew);

  /// Update derived attributes used for logging/inspection.
  virtual void updateLogAttributes(const Matrix &u) const;

  virtual void recomputeDiscreteModel();

  /// Hook for variable/time-varying SSN components.
  virtual void updateStateSpaceModel();

  virtual Attribute<Matrix>::Ptr inputAttribute() const = 0;
  virtual Attribute<Matrix>::Ptr outputAttribute() const = 0;

public:
  struct LocalAbcStateBlock {
    /// Local SSN state indices representing one native abc-frame state block.
    std::array<UInt, 3> indices;

    /// Local base name of this abc-frame state block.
    ///
    /// For example, a block named "vc" is labelled as vc_a, vc_b, vc_c in
    /// the native frame and vc_d, vc_q, vc_0 in the GlobalDQ0 frame.
    String name;
  };

  /// Get number of internal state variables of the SSN model.
  UInt getStateCount() const;

  /// Get local SSN state names.
  ///
  /// Returned names are local to this SSN component. The MNA state-space
  /// contributor adds the component name to form global extracted-state names.
  ///
  /// The default implementation returns no names. Missing names are filled with
  /// generic fallback names by the extractor.
  virtual std::vector<String> getLocalStateNames() const;

  /// Get local abc-frame state blocks.
  ///
  /// Returned indices are local SSN state indices. The MNA state-space
  /// contributor adds the global extracted-state offset.
  virtual std::vector<LocalAbcStateBlock> getLocalAbcStateBlocks() const;

  /// Get discrete state transition matrix used by the trapezoidal SSN model.
  const Matrix &getDiscreteA() const;

  /// Get discrete input matrix used by the trapezoidal SSN model.
  const Matrix &getDiscreteB() const;

  /// Get continuous-time output matrix of the SSN model.
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

} // namespace EMT
} // namespace CPS
