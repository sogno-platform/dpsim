/* Copyright 2017-2026 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNASyncGenInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Three-phase constant active and reactive power load.
///
/// The load is represented by a voltage-dependent current sink. Above the
/// configured minimum voltage, its instantaneous three-phase active and
/// reactive powers equal the configured set points. Below that voltage, the
/// denominator of the current law is limited so the current remains finite.
/// For unbalanced or distorted voltages, this is an instantaneous-p/q model:
/// the current may become unbalanced or distorted to maintain total p and q.
/// Same-time-step correction currently requires the standard direct-MNA solve
/// path; DPSim's system-matrix-recomputation path does not run correctors.
class PQLoad : public MNASimPowerComp<Real>,
               public MNASyncGenInterface,
               public SharedFactory<PQLoad> {
protected:
  /// Calculates the voltage vector shifted by 90 degrees for positive sequence.
  static Matrix quadratureVoltage(const Matrix &voltage);
  /// Calculates the load current for a given instantaneous abc voltage.
  Matrix calculateCurrent(const Matrix &voltage) const;
  /// Updates the instantaneous measured powers.
  void updateMeasuredPower();

public:
  /// Three-phase total active-power set point [W], positive for consumption.
  const Attribute<Real>::Ptr mActivePower;
  /// Three-phase total reactive-power set point [var], positive for inductive
  /// consumption.
  const Attribute<Real>::Ptr mReactivePower;
  /// Nominal line-to-line RMS voltage [V].
  const Attribute<Real>::Ptr mNomVoltage;
  /// Minimum line-to-line voltage in per unit of mNomVoltage.
  const Attribute<Real>::Ptr mMinimumVoltagePerUnit;
  /// Instantaneous measured three-phase active power [W].
  const Attribute<Real>::Ptr mMeasuredActivePower;
  /// Instantaneous measured three-phase reactive power [var].
  const Attribute<Real>::Ptr mMeasuredReactivePower;

  /// Defines UID, name and logging level.
  PQLoad(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level.
  PQLoad(String name, Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level.
  PQLoad(String name, Real activePower, Real reactivePower, Real nominalVoltage,
         Logger::Level logLevel = Logger::Level::off);
  /// Defines UID, name, component parameters and logging level.
  PQLoad(String uid, String name, Real activePower, Real reactivePower,
         Real nominalVoltage, Logger::Level logLevel = Logger::Level::off);

  /// Sets total three-phase P/Q, nominal line-to-line voltage and voltage floor.
  void setParameters(Real activePower, Real reactivePower, Real nominalVoltage,
                     Real minimumVoltagePerUnit = 0.1);
  SimPowerComp<Real>::Ptr clone(String name) override;

  String description() override {
    return fmt::format("Active: {}MW, Reactive: {}MVAr, Voltage: {}kV",
                       **mActivePower / 1e6, **mReactivePower / 1e6,
                       **mNomVoltage / 1e3);
  }

  /// Initializes set points and instantaneous values from terminal data.
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
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

  // #### Same-time-step fixed-point correction ####
  void correctorStep() override;
  void updateVoltage(const Matrix &leftVector) override;
  bool requiresIteration() override;
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
