/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/Base/Base_Ph3_PiLine.h>
#include <dpsim-models/MNASimPowerComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {
namespace SSN {

/// Three-phase Pi line stamped directly as a multiport state-space Norton model.
///
/// The series R-L branch and both shunt G-C branches are represented by one
/// component, avoiding per-element tasks and composite right-vector reduction.
class PiLine final : public MNASimPowerComp<Complex>,
                     public Base::Ph3::PiLine,
                     public SharedFactory<PiLine> {
public:
  using SharedFactory<PiLine>::make;

  PiLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  PiLine(String name, Logger::Level logLevel = Logger::Level::off)
      : PiLine(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String copySuffix) override;

  void setParameters(const Matrix &seriesResistance,
                     const Matrix &seriesInductance,
                     const Matrix &parallelCapacitance = Matrix::Zero(3, 3),
                     const Matrix &parallelConductance = Matrix::Zero(3, 3));

  void initializeFromNodesAndTerminals(Real frequency) override;
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  Bool mnaHasCompactRightSideVectorStamp() const override { return true; }
  void mnaAddCompactRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

private:
  using TerminalVoltage = Eigen::Matrix<Complex, 3, 1>;

  Attribute<MatrixComp>::Ptr mState;
  MatrixComp mInductorAdmittance;
  MatrixComp mSeriesAdmittance;
  MatrixComp mSeriesSystemInverse;
  MatrixComp mCapAdmittance;
  MatrixComp mCapPreviousVoltage;
  MatrixComp mSeriesHistory;
  MatrixComp mInductorHistory;
  MatrixComp mCapHistory0;
  MatrixComp mCapHistory1;
  Complex mInductorPreviousCurrent = Complex(1., 0.);

  TerminalVoltage terminalVoltage(const Matrix &leftVector, UInt terminal);
};

} // namespace SSN
} // namespace Ph3
} // namespace DP
} // namespace CPS
