/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_SVC.h>
#include <dpsim-models/DP/DP_Ph1_Capacitor.h>
#include <dpsim-models/DP/DP_Ph1_Inductor.h>
#include <dpsim-models/DP/DP_Ph1_Switch.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief SVC
///
/// SVC represented by an Inductor
/// The inductor is represented by a DC equivalent circuit which corresponds to
/// one iteration of the trapezoidal integration method.
/// The equivalent DC circuit is a resistance in parallel with a current source.
/// The resistance is constant for a defined time step and system
/// frequency and the current source changes for each iteration.
class SVC : public MNASimPowerComp<Complex>,
            public Base::Ph1::SVC,
            public MNAVariableCompInterface,
            public SharedFactory<SVC> {
protected:
  /// ### internal components
  /// Internal inductor
  std::shared_ptr<DP::Ph1::Inductor> mSubInductor;
  std::shared_ptr<DP::Ph1::Switch> mSubInductorProtectionSwitch;
  /// Internal capacitor
  std::shared_ptr<DP::Ph1::Capacitor> mSubCapacitor;
  std::shared_ptr<DP::Ph1::Switch> mSubCapacitorProtectionSwitch;

  Bool mValueChange = false;
  Bool mInductiveMode = false;

  // has state changed? (allow this only once)
  // could be redundant with internal status of switch element (mnaIsClosed)
  Real mSwitchStateChange = false;
  Real mSwitchROpen = 1e9;
  Real mSwitchRClosed = 1e-9;
  Real mCounter = 0;

  Real mBSetCounter = 0;

  // values of PT1 for measurement of voltage at PCC
  Real mTm = 0.01;
  Real mKm = 1;
  Real mLPrev;
  Real mCPrev;

  // variables for protection
  Real mProtCount1 = 0;
  Real mProtCount2 = 0;
  Real mProtCount3 = 0;
  Bool mDisconnect = false;

public:
  const Attribute<Real>::Ptr mVpcc;
  const Attribute<Real>::Ptr mVmeasPrev;

  /// Defines UID, name and log level
  SVC(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  /// Defines name and log level
  SVC(String name, Logger::Level logLevel = Logger::Level::off)
      : SVC(name, name, logLevel) {}

  // #### General ####
  /// Initializes states from power flow data
  void initializeFromNodesAndTerminals(Real frequency);

  // #### MNA section ####
  /// Initializes MNA specific variables
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector);
  //void mnaCompInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors);
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix);
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector);
  /// Update interface voltage from MNA system results
  void mnaCompUpdateVoltage(const Matrix &leftVector);
  /// Update interface current from MNA system results
  void mnaCompUpdateCurrent(const Matrix &leftVector);
  /// MNA pre step operations
  void mnaCompPreStep(Real time, Int timeStepCount);
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector);
  /// add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                     AttributeBase::List &attributeDependencies,
                                     AttributeBase::List &modifiedAttributes);
  /// add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector);

  // #### Tearing methods ####
  Bool ValueChanged();
  void setSwitchState();

  void updateSusceptance();
  void mechanicalModelUpdateSusceptance(Real time);
  // check protection function
  void checkProtection(Real time);
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
