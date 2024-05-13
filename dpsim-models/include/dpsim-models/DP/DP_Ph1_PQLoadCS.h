/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph1_CurrentSource.h>
#include <dpsim-models/PowerProfile.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// TODO: read from CSV files
/// \brief PQ-load represented by a current source
class PQLoadCS : public CompositePowerComp<Complex>,
                 public SharedFactory<PQLoadCS> {
protected:
  /// Internal current source
  std::shared_ptr<DP::Ph1::CurrentSource> mSubCurrentSource;
  void updateSetPoint();
  void updateIntfValues();

public:
  const Attribute<Real>::Ptr mActivePower;
  const Attribute<Real>::Ptr mReactivePower;
  const Attribute<Real>::Ptr mNomVoltage;

  /// Defines UID, name and logging level
  PQLoadCS(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);
  /// Defines UID, name and logging level
  PQLoadCS(String name, Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  PQLoadCS(String name, Real activePower, Real reactivePower, Real volt,
           Logger::Level logLevel = Logger::Level::off);
  /// Defines UID, name and logging level
  PQLoadCS(String uid, String name, Real activePower, Real reactivePower,
           Real nomVolt, Logger::Level logLevel = Logger::Level::off);

  void setParameters(Real activePower, Real reactivePower, Real nomVolt);
  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// MNA pre and post step operations
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;

  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
