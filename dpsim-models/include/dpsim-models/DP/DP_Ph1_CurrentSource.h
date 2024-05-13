/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_CurrentSource.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief Dynamic phasor ideal current source
///
/// A positive current is flowing out of node1 and into node2.
/// In case of a dynamic phasor simulation, a frequency different
/// from zero is added on top of the system frequency.
class CurrentSource : public MNASimPowerComp<Complex>,
                      public SharedFactory<CurrentSource> {
public:
  const Attribute<Complex>::Ptr mCurrentRef;
  /// Defines UID, name and logging level
  CurrentSource(String uid, String name,
                Logger::Level loglevel = Logger::Level::off);
  /// Defines name and logging level
  CurrentSource(String name, Logger::Level logLevel = Logger::Level::off)
      : CurrentSource(name, name, logLevel) {}
  /// Defines name, component parameters and logging level
  CurrentSource(String name, Complex current,
                Logger::Level logLevel = Logger::Level::off);

  void setParameters(Complex current);

  SimPowerComp<Complex>::Ptr clone(String copySuffix) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  ///
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override {}
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  ///
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;

  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
