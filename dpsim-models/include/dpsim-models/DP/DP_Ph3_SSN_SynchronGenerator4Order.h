// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <dpsim-models/DP/DP_Ph3_MixedVTypeVariableSSNComp.h>

namespace CPS::DP::Ph3::SSN {

/// Fourth-order dq synchronous generator represented as a nonlinear SSN port.
class SynchronGenerator4Order : public MixedVTypeVariableSSNComp {
public:
  using Ptr = std::shared_ptr<SynchronGenerator4Order>;
  static Ptr make(String uid, String name,
                  Logger::Level level = Logger::Level::off) {
    return std::make_shared<SynchronGenerator4Order>(uid, name, level);
  }
  static Ptr make(String name, Logger::Level level = Logger::Level::off) {
    return make(name, name, level);
  }

  SynchronGenerator4Order(String uid, String name,
                          Logger::Level level = Logger::Level::off);
  SynchronGenerator4Order(String name, Logger::Level level = Logger::Level::off)
      : SynchronGenerator4Order(name, name, level) {}

  void setParametersOperationalPerUnit(Real nominalPower, Real nominalVoltage,
                                       Real nominalFrequency, Real inertia,
                                       Real resistance, Real xdTransient,
                                       Real xqTransient, Real xd, Real xq,
                                       Real td0Transient, Real tq0Transient,
                                       Real mechanicalDamping = 1.0);
  /// Initial terminal P/Q and voltage magnitude are per unit; angle is radians.
  void setPowerflowInitialValues(Real activePower, Real reactivePower,
                                 Real voltageMagnitudePu, Real voltageAngleRad);
  void initializeFromNodesAndTerminals(Real frequency) override;
  Matrix getState() const;

protected:
  Bool updateComponentParameters() override;
  void updateLogAttributes(const Matrix &u) const override;

private:
  friend class SynchronGenerator4OrderJacobianCheck;
  void evaluate(const Matrix &x, const Matrix &u, Matrix &f, Matrix &y) const;
  void linearize(const Matrix &x, const Matrix &u, Matrix &A, Matrix &B,
                 Matrix &C, Matrix &D) const;
  void initializeState(const MatrixComp &phaseVoltage);

  Real mNominalPower = 0., mNominalVoltage = 0., mFrequency = 0.;
  Real mBaseV = 0., mBaseI = 0., mBaseZ = 0., mOmega = 0.;
  Real mH = 0., mRa = 0., mXd = 0., mXq = 0., mXdp = 0., mXqp = 0.;
  Real mTd0p = 0., mTq0p = 0., mDamping = 0.;
  Real mPm = 0., mQm = 0., mVpu = 1., mVangle = 0.;
  Real mEd0 = 0., mEq0 = 1., mId0 = 0., mIq0 = 0.;
  bool mConfigured = false, mInitialValuesConfigured = false;
  Matrix mPackedVoltage = Matrix::Zero(6, 1);
  Matrix mEvaluationDerivative = Matrix::Zero(4, 1);
  Matrix mEvaluationOutput = Matrix::Zero(6, 1);
};

} // namespace CPS::DP::Ph3::SSN
