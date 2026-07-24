// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <memory>

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeVariableSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Sixth-order synchronous-machine electrical model with mechanical
/// rotor dynamics, formulated as a variable V-type SSN component.
///
/// Electrical model from Dufour:
///
///   psi_dot = (-R L^-1 + Omega) psi + [v_sd, v_sq, v_f, 0, 0, 0]^T
///   i_dqf   = firstThreeRows(L^-1) psi
///
/// State vector:
///   [psi_sd, psi_sq, psi_f', psi_kd', psi_kq1', psi_kq2', omega_m, theta_e]
///
/// Assumptions of this first implementation:
/// - balanced three-wire stator operation; zero sequence is omitted;
/// - constant externally supplied field voltage;
/// - externally applied shaft torque;
/// - no magnetic saturation;
/// - rotor electrical quantities are referred to the stator;
/// - all parameters use SI units.
class SSN_SynchronousGenerator : public TwoTerminalVTypeVariableSSNComp {
public:
  using Ptr = std::shared_ptr<SSN_SynchronousGenerator>;

  static Ptr make(String uid, String name,
                  Logger::Level logLevel = Logger::Level::off) {
    return std::make_shared<SSN_SynchronousGenerator>(uid, name, logLevel);
  }

  static Ptr make(String name, Logger::Level logLevel = Logger::Level::off) {
    return std::make_shared<SSN_SynchronousGenerator>(name, name, logLevel);
  }

  enum StateIndex {
    PsiSd = 0,
    PsiSq,
    PsiField,
    PsiDamperD,
    PsiDamperQ1,
    PsiDamperQ2,
    MechanicalSpeed,
    ElectricalAngle,
    StateCount
  };

protected:
  static constexpr Int mElectricalStateSize = 6;
  static constexpr Int mStateSize = StateCount;
  static constexpr Int mInputSize = 3;
  static constexpr Int mOutputSize = 3;

  Int mPolePairs;

  Real mNominalFrequency;
  Real mNominalMechanicalSpeed;

  Real mStatorResistance;
  Real mFieldResistance;
  Real mDamperResistanceD;
  Real mDamperResistanceQ1;
  Real mDamperResistanceQ2;

  Real mLd;
  Real mLq;
  Real mLmd;
  Real mLmq;
  Real mLField;
  Real mLDamperD;
  Real mLDamperQ1;
  Real mLDamperQ2;

  Real mRotorInertia;
  Real mMechanicalDamping;

  Real mFieldVoltage;
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
  Attribute<Real>::Ptr mFieldCurrent;

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
  SSN_SynchronousGenerator(String uid, String name,
                           Logger::Level logLevel = Logger::Level::off);

  explicit SSN_SynchronousGenerator(String name,
                                    Logger::Level logLevel = Logger::Level::off)
      : SSN_SynchronousGenerator(name, name, logLevel) {}

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
                     Real statorResistance, Real fieldResistance,
                     Real damperResistanceD, Real damperResistanceQ1,
                     Real damperResistanceQ2, Real ld, Real lq, Real lmd,
                     Real lmq, Real lField, Real lDamperD, Real lDamperQ1,
                     Real lDamperQ2, Real rotorInertia, Real mechanicalDamping,
                     Real fieldVoltage, Real mechanicalTorque,
                     Real initialElectricalAngle = 0.0,
                     Bool autoInitializeMechanicalTorque = true);

  void setFieldVoltage(Real fieldVoltage);
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
