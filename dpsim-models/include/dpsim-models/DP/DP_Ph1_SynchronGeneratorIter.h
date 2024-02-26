/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_ReducedOrderSynchronGenerator.h>
#include <dpsim-models/Solver/MNASyncGenInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// @brief Base class for DP VBR synchronous generator model single phase
class SynchronGeneratorIter
    : public Base::ReducedOrderSynchronGenerator<Complex>,
      public MNASyncGenInterface {
public:
private:
protected:
  /// Constructor
  SynchronGeneratorIter(const String &uid, const String &name,
                        Logger::Level logLevel);
  SynchronGeneratorIter(const String &name, Logger::Level logLevel);

  // #### General Functions ####
  ///
  virtual void specificInitialization() = 0;
  ///
  virtual void stepInPerUnit() = 0;
  //
  virtual void correctorStep() = 0;
  ///
  void updateVoltage(const Matrix &leftVector);
  ///
  bool requiresIteration();
  ///
  Matrix parkTransform(Real theta, const Matrix &abcVector);

  // ### MNA Section ###
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix);
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector);
  void mnaCompPostStep(const Matrix &leftVector);
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector);

public:
  virtual ~SynchronGeneratorIter();

  /// Mark that parameter changes so that system matrix is updated
  Bool hasParameterChanged() override { return 1; };
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
