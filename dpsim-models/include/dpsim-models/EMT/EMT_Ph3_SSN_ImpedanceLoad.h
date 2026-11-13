/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/EMT/EMT_LinearSSNModel.h>
#include <dpsim-models/MNASimPowerComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
namespace SSN {

/// Balanced fixed-impedance load and bus shunt as one SSN MNA component.
class ImpedanceLoad final : public MNASimPowerComp<Real>,
                            public SharedFactory<ImpedanceLoad> {
public:
  using SharedFactory<ImpedanceLoad>::make;

  ImpedanceLoad(String uid, String name,
                Logger::Level logLevel = Logger::Level::off);
  ImpedanceLoad(String name, Logger::Level logLevel = Logger::Level::off)
      : ImpedanceLoad(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String copySuffix) override;

  /// Powers are total three-phase values; voltage is line-to-line RMS.
  /// Shunt conductance and susceptance are per-phase Siemens.
  void setParameters(Real activePower, Real reactivePower, Real nominalVoltage,
                     Real shuntConductance = 0., Real shuntSusceptance = 0.);

  void initializeFromNodesAndTerminals(Real frequency) override;
  void exportFusedModel(LinearSSNModel &model);
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
  using PhaseVector = Eigen::Matrix<Real, 3, 1>;

  Attribute<Matrix>::Ptr mState;
  Matrix *mStateValue;
  Real mActivePower = 0.;
  Real mReactivePower = 0.;
  Real mNominalVoltage = 0.;
  Real mShuntConductance = 0.;
  Real mShuntSusceptance = 0.;
  Real mLoadInductance = 0.;
  Real mLoadCapacitance = 0.;
  Real mLoadConductance = 0.;
  Real mTotalConductance = 0.;
  Real mInductorAdmittanceScalar = 0.;
  Real mCapacitorAdmittanceScalar = 0.;
  Matrix mNodalAdmittance;
  Matrix mInductorAdmittance;
  Matrix mCapacitorAdmittance;
  Matrix mHistoryLoad;
  Matrix mHistoryShunt;

  void updateHistory();
  PhaseVector nodeVoltage(const Matrix &leftVector);
  void updateCurrentFromVoltage(const PhaseVector &voltage);
};

} // namespace SSN
} // namespace Ph3
} // namespace EMT
} // namespace CPS
