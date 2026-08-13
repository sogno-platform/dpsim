// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <memory>

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeVariableSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Induction machine model formulated as a variable V-type SSN component.
///
/// Electrical model from Dufour:
///
///   psi_dot = (-R L^-1 + Omega) psi + v_dq
///   i_dq    = (L^-1) psi
///
/// State vector:
///   psi = [psi_sd, psi_sq, psi_rd', psi_rq', mechanical_speed, electrical_angle]^t
///
/// Primed/dashed quantities are reffered to stator.
///
class SSN_InductionMotor : public TwoTerminalVTypeVariableSSNComp {
public:
  using Ptr = std::shared_ptr<SSN_InductionMotor>;

  static Ptr make(String uid, String name,
                  Logger::Level logLevel = Logger::Level::off) {
    return std::make_shared<SSN_InductionMotor>(uid, name, logLevel);
  }

  static Ptr make(String name, Logger::Level logLevel = Logger::Level::off) {
    return std::make_shared<SSN_InductionMotor>(name, name, logLevel);
  }

  enum StateIndex {
    PsiSd = 0,
    PsiSq,
    PsiRotorD,
    PsiRotorQ,
    MechanicalSpeed,
    ElectricalAngle,
    StateCount
  };

protected:
  static constexpr Int mElectricalStateSize = 4;
  static constexpr Int mStateSize = StateCount;
  static constexpr Int mInputSize = 3;
  static constexpr Int mOutputSize = 3;

  Int mPolePairs;

  Real mNominalFrequency;
  Real mNominalMechanicalSpeed;

  Real mStatorResistance;
  Real mRotorResistance;

  //rotor inductance
  Real mLr_dash;
  //mutual inductance
  Real mLm;
  //stator inductance
  Real mLs;

  Real mRotorInertia;
  Real mMechanicalDamping;

  Real mMechanicalTorque;
  Real mInitialElectricalAngle;
  Bool mAutoInitializeMechanicalTorque;

  Matrix mInductanceMatrix;
  Matrix mInverseInductanceMatrix;
  Matrix mResistanceMatrix;

  Real mJacobianRelativeStep;
  Real mJacobianAbsoluteStep;

  Attribute<Real>::Ptr mElectricalPower;
  Attribute<Real>::Ptr mElectricalTorque;
  Attribute<Real>::Ptr mMechanicalSpeedLog;
  Attribute<Real>::Ptr mElectricalAngleLog;
  Attribute<Real>::Ptr mStatorCurrentD;
  Attribute<Real>::Ptr mStatorCurrentQ;
  Attribute<Real>::Ptr mStatorVoltageD;
  Attribute<Real>::Ptr mStatorVoltageQ;

  Matrix getParkTransformMatrix(Real electricalAngle) const;
  Matrix getInverseParkTransformMatrix(Real electricalAngle) const;
  Matrix buildSpeedMatrix(Real electricalSpeed) const;

  void rebuildMachineMatrices();

  void evaluateStateDerivative(const Matrix &x, const Matrix &u,
                               Matrix &stateDerivative) const;
  void evaluateOutput(const Matrix &x, const Matrix &u, Matrix &output) const;

  void calculateNumericalJacobians(const Matrix &x, const Matrix &u, Matrix &A,
                                   Matrix &B, Matrix &C, Matrix &D) const;

  void buildStateSpaceModel(const Matrix &x, const Matrix &u, Matrix &A,
                            Matrix &B, Matrix &C, Matrix &D, Matrix &E,
                            Matrix &F) const;

  Bool updateComponentParameters() override;

  std::vector<String> getLocalStateNames() const override;
  void updateLogAttributes(const Matrix &u) const override;

public:
  SSN_InductionMotor(String uid, String name,
                     Logger::Level logLevel = Logger::Level::off);

  explicit SSN_InductionMotor(String name,
                              Logger::Level logLevel = Logger::Level::off)
      : SSN_InductionMotor(name, name, logLevel) {}

  /// \brief Configure the sixth-order machine.
  ///
  /// Units:
  /// - frequency: Hz
  /// - resistances: ohm
  /// - inductances: H
  /// - inertia: kg m^2
  /// - damping: N m s/rad
  /// - field voltage: V, referred to stator
  /// - mechanical torque: N m
  void setParameters(Real nominalFrequency, Int polePairs,
                     Real statorResistance, Real rotorResistance,
                     Real statorInductance, Real rotorInductance,
                     Real mutualInductance, Real rotorInertia,
                     Real mechanicalDamping, Real mechanicalTorque,
                     Real initialElectricalAngle = 0.0,
                     Bool autoInitializeMechanicalTorque = true);

  void setMechanicalTorque(Real mechanicalTorque);

  void setNumericalLinearizationParameters(Real relativeStep,
                                           Real absoluteStep);

  void initializeFromNodesAndTerminals(Real frequency) override;

  Matrix getState() const;
  Matrix getStateDerivative() const;
  Matrix getInterfaceVoltage() const;
  Matrix getInterfaceCurrent() const;
  std::vector<String> getStateNames() const { return getLocalStateNames(); }
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
