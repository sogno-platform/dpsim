/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Task.h>

namespace CPS {
/// Interface to be implemented by all models used by the MNA solver.
class MNAInterface {
public:
  typedef std::shared_ptr<MNAInterface> Ptr;
  typedef std::vector<Ptr> List;

  // #### MNA Base Functions ####
  /// Initializes variables of components
  virtual void mnaInitialize(Real omega, Real timeStep) = 0;
  virtual void mnaInitialize(Real omega, Real timeStep,
                             Attribute<Matrix>::Ptr leftVector) = 0;
  /// Stamps system matrix
  virtual void mnaApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) = 0;
  /// Stamps right side (source) vector
  virtual void mnaApplyRightSideVectorStamp(Matrix &rightVector) = 0;
  /// Update interface voltage from MNA system result
  virtual void mnaUpdateVoltage(const Matrix &leftVector) = 0;
  /// Update interface current from MNA system result
  virtual void mnaUpdateCurrent(const Matrix &leftVector) = 0;
  /// MNA pre step operations
  virtual void mnaPreStep(Real time, Int timeStepCount) = 0;
  /// MNA post step operations
  virtual void mnaPostStep(Real time, Int timeStepCount,
                           Attribute<Matrix>::Ptr &leftVector) = 0;
  /// Add MNA pre step dependencies
  virtual void
  mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                            AttributeBase::List &attributeDependencies,
                            AttributeBase::List &modifiedAttributes) = 0;
  /// Add MNA post step dependencies
  virtual void
  mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                             AttributeBase::List &attributeDependencies,
                             AttributeBase::List &modifiedAttributes,
                             Attribute<Matrix>::Ptr &leftVector) = 0;

  // #### MNA Harmonic Base Functions ####
  /// Initializes variables of components
  virtual void
  mnaInitializeHarm(Real omega, Real timeStep,
                    std::vector<Attribute<Matrix>::Ptr> leftVector) = 0;
  /// Stamps system matrix considering the frequency index
  virtual void mnaApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix,
                                             Int freqIdx) = 0;
  /// Stamps right side (source) vector considering the frequency index
  virtual void mnaApplyRightSideVectorStampHarm(Matrix &sourceVector) = 0;
  virtual void mnaApplyRightSideVectorStampHarm(Matrix &sourceVector,
                                                Int freqIdx) = 0;
  /// Return list of MNA tasks
  virtual const Task::List &mnaTasks() const = 0;
  // Return right vector attribute
  virtual Attribute<Matrix>::Ptr getRightVector() const = 0;
};
} // namespace CPS
