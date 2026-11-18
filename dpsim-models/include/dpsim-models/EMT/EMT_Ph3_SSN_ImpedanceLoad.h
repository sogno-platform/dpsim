/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
namespace SSN {

/// Balanced fixed-impedance load and bus shunt as one SSN MNA component.
class ImpedanceLoad final : public VTypeSSNComp,
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
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;

private:
  Real mActivePower = 0.;
  Real mReactivePower = 0.;
  Real mNominalVoltage = 0.;
  Real mShuntConductance = 0.;
  Real mShuntSusceptance = 0.;
  Real mLoadInductance = 0.;
  Real mLoadCapacitance = 0.;
  Real mLoadConductance = 0.;
  Real mTotalConductance = 0.;

protected:
  MatrixComp buildInitialInputFromNodes(Real frequency) override;
};

} // namespace SSN
} // namespace Ph3
} // namespace EMT
} // namespace CPS
