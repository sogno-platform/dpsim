/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/EMT/EMT_Ph1_VoltageSource.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
class VoltageSourceRamp : public CompositePowerComp<Real>,
                          public SharedFactory<VoltageSourceRamp> {
protected:
  ///
  Complex mAddVoltage;
  ///
  Real mSwitchTime;
  ///
  Real mAddSrcFreq;
  ///
  Real mRampTime;
  ///
  std::shared_ptr<VoltageSource> mSubVoltageSource;

public:
  ///
  const Attribute<Complex>::Ptr mVoltageRef;
  ///
  const Attribute<Real>::Ptr mSrcFreq;
  /// Defines UID, name and logging level
  VoltageSourceRamp(String uid, String name,
                    Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  VoltageSourceRamp(String name, Logger::Level logLevel = Logger::Level::off)
      : VoltageSourceRamp(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override {}
  ///
  void setParameters(Complex voltage, Complex addVoltage, Real srcFreq,
                     Real addSrcFreq, Real switchTime, Real rampTime);
  ///
  void initialize(Matrix frequencies) override;

  // #### MNA section ####
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  void updateState(Real time);
};
} // namespace Ph1
} // namespace EMT
} // namespace CPS
