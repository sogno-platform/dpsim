/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Switch.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace SP {
namespace Ph1 {
/// \brief
///
/// Switch with variable resistance to avoid numerical oscillations, when an inductive current is suddenly interrupted.
/// It is useful as a fault switch especially on a faulted generator or transformer winding.

/// The switch resistance changes at a defined fixed rate by multiplying previous resistance value with a factor for the rate of change
/// The MNA variable component interface is used to recompute the system Matrix after each change.
class varResSwitch : public MNASimPowerComp<Complex>,
                     public Base::Ph1::Switch,
                     public MNAVariableCompInterface,
                     public MNASwitchInterface,
                     public SharedFactory<varResSwitch> {

protected:
  Bool mPrevState = false;
  Real mDeltaResClosed = 0;
  Real mDeltaResOpen = 1.5;
  Real mPrevRes; // previous resistance value to multiply with rate of change
  // because we change the base value mClosedResistance & mOpenResistance to recompute the system Matrix
  // we need to save the initialisation values to use them as target values in the transition
  Real mInitClosedRes;
  Real mInitOpenRes;

public:
  void setInitParameters(Real timestep);

  /// Defines UID, name and log level
  varResSwitch(String uid, String name,
               Logger::Level logLevel = Logger::Level::off);
  /// Defines name and log level
  varResSwitch(String name, Logger::Level logLevel = Logger::Level::off)
      : varResSwitch(name, name, logLevel) {}

  // varResSwitch(String name, Real openRes, Real closedRes ,
  // 		Logger::Level logLevel = Logger::Level::off);

  SimPowerComp<Complex>::Ptr clone(String name);

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
  // void mnaCompPreStep(Real time, Int timeStepCount);
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector);
  /// add MNA pre step dependencies
  /// void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
  /// add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector);

  // #### MNA section for switch ####
  /// Check if switch is closed
  Bool mnaIsClosed();
  /// Stamps system matrix considering the defined switch position
  void mnaCompApplySwitchSystemMatrixStamp(Bool closed,
                                           SparseMatrixRow &systemMatrix,
                                           Int freqIdx);

  Bool hasParameterChanged();
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
