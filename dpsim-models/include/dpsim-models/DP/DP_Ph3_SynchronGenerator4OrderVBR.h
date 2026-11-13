/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/DP/DP_Ph3_SynchronGeneratorDQ.h>
#include <dpsim-models/MNAStampUtils.h>

namespace CPS {
namespace DP {
namespace Ph3 {
/// Three-phase positive-sequence VBR synchronous generator with transient
/// d- and q-axis states and the rotor speed/angle states (fourth order).
class SynchronGenerator4OrderVBR
    : public SynchronGeneratorDQ,
      public SharedFactory<SynchronGenerator4OrderVBR> {
public:
  SynchronGenerator4OrderVBR(String uid, String name,
                             Logger::Level logLevel = Logger::Level::off);
  SynchronGenerator4OrderVBR(String name,
                             Logger::Level logLevel = Logger::Level::off);

  void setParametersOperationalPerUnit(Real nomPower, Real nomVolt,
                                       Real nomFreq, Int poleNumber,
                                       Real nomFieldCur, Real Rs, Real Ld,
                                       Real Lq, Real Ld_t, Real Lq_t, Real Ll,
                                       Real Td0_t, Real Tq0_t, Real inertia);

  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  Bool mnaHasCompactRightSideVectorStamp() const override { return true; }
  void mnaAddCompactRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

private:
  Real mTimeStep = 0;
  Real mEdTransient = 0;
  Real mEqTransient = 0;
  Real mInitialEdTransient = 0;
  Real mInitialEqTransient = 0;
  Real mInitialCurrentD = 0;
  Real mInitialCurrentQ = 0;
  Real mNortonEd = 0;
  Real mNortonEq = 0;
  Real mMechanicalPower = 0;
  Bool mInitialMechanicalPowerSet = false;
  // The EMT SG4 VBR model has no shaft damping term. Keep the same swing
  // equation in DP so that damping does not hide initialization/model errors.
  Real mDamping = 1.0;
  Real mCurrentD = 0;
  Real mCurrentQ = 0;
  MatrixComp mAdmittance;
  MatrixComp mInternalVoltage;
  MatrixComp mNortonCurrent;
  MatrixComp mVoltageDifference;

  void updateCurrentAndStates(Real time);
};
} // namespace Ph3
} // namespace DP
} // namespace CPS
