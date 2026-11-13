/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/Base/Base_Ph3_Transformer.h>
#include <dpsim-models/EMT/EMT_LinearSSNModel.h>
#include <dpsim-models/MNASimPowerComp.h>

namespace CPS::EMT::Ph3::SSN {

/// Three-phase transformer represented by a single Norton multiport stamp.
/// The equivalent preserves the conventional transformer's T network and
/// magnetizing branch while eliminating its internal virtual nodes.
class Transformer final : public MNASimPowerComp<Real>,
                          public Base::Ph3::Transformer,
                          public SharedFactory<Transformer> {
public:
  using SharedFactory<Transformer>::make;

  Transformer(String uid, String name,
              Logger::Level logLevel = Logger::Level::off,
              Bool withResistiveLosses = false);
  Transformer(String name, Logger::Level logLevel = Logger::Level::off)
      : Transformer(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String copySuffix) override;

  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratedPower, Real ratioAbs, Real ratioPhase,
                     Matrix resistance, Matrix inductance);
  void setMagnetizingBranch(Real noLoadCurrent, Real noLoadLoss);
  void setReferenceWinding(WindingReference referenceWinding);

  void initializeFromNodesAndTerminals(Real frequency) override;
  void exportFusedModel(LinearSSNModel &model);
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  Bool mnaHasCompactRightSideVectorStamp() const override { return true; }
  void mnaAddCompactRightSideVectorStamp(Matrix &rightVector) override;
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
  using PhaseVector = Eigen::Matrix<Real, 3, 1>;

  Attribute<Matrix>::Ptr mState;
  Matrix *mStateValue;
  Matrix mSeriesResistanceHalf;
  Matrix mSeriesInductanceHalf;
  Matrix mInductorAdmittance;
  Matrix mSeriesSystemInverse;
  Matrix mSeriesAdmittance;
  Matrix mMagnetizingInductorAdmittance;
  Matrix mMagnetizingConductance;
  Matrix mMidpointInverse;
  Matrix mPortAdmittance00;
  Matrix mPortAdmittance01;
  Matrix mPortAdmittance10;
  Matrix mPortAdmittance11;
  Matrix mHistory0;
  Matrix mHistory1;
  Matrix mHistoryMagnetizing;
  Matrix mMidpointHistory;
  Matrix mMidpointSource;
  Matrix mSource0;
  Matrix mSource1;
  Matrix mMagnetizingResistance;
  Matrix mMagnetizingInductance;
  Real mNoLoadCurrent = 0.01;
  Real mNoLoadLoss = 1e-3;
  Bool mWithResistiveLosses = false;
  UInt mReferenceTerminal = 0;
  Real mOrientationSign = 1.;
  Complex mRatioFromReference = Complex(1., 0.);
  WindingReference mReferenceWinding = WindingReference::Auto;

  UInt nonReferenceTerminal() const { return 1 - mReferenceTerminal; }
  void resolveWindingRoles();
  PhaseVector terminalVoltage(const Matrix &leftVector, UInt terminal);
  void updateHistory();
};

} // namespace CPS::EMT::Ph3::SSN
