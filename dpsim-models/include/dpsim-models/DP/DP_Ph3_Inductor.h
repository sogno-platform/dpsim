/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph3_Inductor.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// \brief Three-phase dynamic-phasor inductor.
///
/// The inductor is represented by the trapezoidal-integration companion model.
/// Three-phase DP interface quantities use phase-peak complex envelopes.
/// Power-flow node voltages are line-line RMS phasors and are converted to the
/// DP phase-peak convention during initialization.
class Inductor : public MNASimPowerComp<Complex>,
                 public Base::Ph3::Inductor,
                 public MNATearInterface,
                 public SharedFactory<Inductor> {
protected:
  /// Equivalent current source [A]
  MatrixComp mEquivCurrent;
  /// Equivalent conductance [S]
  MatrixComp mEquivCond;
  /// Coefficient multiplying the previous current value
  Complex mPrevCurrFac;

  void initVars(Real omega, Real timeStep);

public:
  Inductor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);

  Inductor(String name, Logger::Level logLevel = Logger::Level::off)
      : Inductor(name, name, logLevel) {}

  Inductor(String name, Real inductance,
           Logger::Level logLevel = Logger::Level::off);

  SimPowerComp<Complex>::Ptr clone(String name) override;

  const MatrixComp &getMNAConductance() const { return mEquivCond; }
  Complex getPreviousCurrentFactor() const { return mPrevCurrFac; }

  // #### General ####
  /// Initialize DP phase-peak envelopes from line-line RMS PF node voltages.
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;

  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;

  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;

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

  // #### MNA Tear Section ####
  void mnaTearInitialize(Real omega, Real timestep) override;
  void mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) override;
  void mnaTearApplyVoltageStamp(Matrix &voltageVector) override;
  void mnaTearPostStep(MatrixComp voltage, MatrixComp current) override;
};

} // namespace Ph3
} // namespace DP
} // namespace CPS
