/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
class MNATearInterface {
public:
  // Returns a list of additional components connected to ground that
  // need to be considered for the original systems.
  virtual MNAInterface::List mnaTearGroundComponents() {
    return MNAInterface::List();
  }
  // Initialize the internal state of the component
  virtual void mnaTearInitialize(Real omega, Real timeStep) {}
  // Apply the stamp to the impedance matrix of the removed network
  virtual void mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) = 0;
  // TODO: if we're consequent, these should be implemented as tasks
  // Apply the stamp to the vector of additional voltages in the removed network
  virtual void mnaTearApplyVoltageStamp(Matrix &currentVector) {}
  // Update the internal state based on the solution of the complete system
  virtual void mnaTearPostStep(Complex voltage, Complex current) {}
  // Update the internal state based on the solution of the complete system
  virtual void mnaTearPostStep(MatrixComp voltage, MatrixComp current) {}

  void mnaTearSetIdx(UInt compIdx) { mTearIdx = compIdx; }

protected:
  UInt mTearIdx;
};
} // namespace CPS
