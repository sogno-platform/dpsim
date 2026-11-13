/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_ReducedOrderSynchronGenerator.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
/// @brief Base class for EMT VBR simplefied synchronous generator models
class ReducedOrderSynchronGeneratorVBR
    : public Base::ReducedOrderSynchronGenerator<Real>,
      public MNAVariableCompInterface {

public:
  // Common elements of all VBR models
  /// voltage behind reactance
  Matrix mEvbr;
  /// norton equivalent current of mEvbr
  Matrix mIvbr;

protected:
  Matrix *mVdq0Value = nullptr;
  Matrix *mIdq0Value = nullptr;
  Real *mEfValue = nullptr;

  /// Resistance matrix in dq0 reference frame
  MatrixFixedSize<3, 3> mResistanceMatrixDq0;

  /// Conductance matrix
  MatrixFixedSize<3, 3> mConductanceMatrix;
  /// Fixed dq0 conductance; the abc conductance is obtained by rotation.
  MatrixFixedSize<3, 3> mConductanceMatrixDq0;

  ///
  MatrixFixedSize<3, 3> mAbcToDq0;
  MatrixFixedSize<3, 3> mDq0ToAbc;

  /// Constructor
  ReducedOrderSynchronGeneratorVBR(const String &uid, const String &name,
                                   Logger::Level logLevel);
  ReducedOrderSynchronGeneratorVBR(const String &name, Logger::Level logLevel);

  // #### General Functions ####
  /// Specific component initialization
  virtual void specificInitialization() override = 0;
  ///
  void initializeResistanceMatrix() override;
  ///
  virtual void stepInPerUnit() override = 0;
  ///
  void calculateResistanceMatrix();
  /// Park Transformation according to Kundur
  MatrixFixedSize<3, 3> get_parkTransformMatrix() const;
  /// Inverse Park Transformation according to Kundur
  MatrixFixedSize<3, 3> get_inverseParkTransformMatrix() const;
  /// Update both transforms while calculating each phase angle once.
  void updateParkTransformMatrices();

  // ### MNA Section ###
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Accumulate only the three source entries instead of its full-sized stamp.
  Bool mnaHasCompactRightSideVectorStamp() const override { return true; }
  void mnaAddCompactRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompPostStep(const Matrix &leftVector) override;
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;

public:
  /// Mark that parameter changes so that system matrix is updated
  Bool hasParameterChanged() override { return true; };
  Bool hasContinuouslyChangingSystemMatrix() const override { return true; };
};
} // namespace Ph3
} // namespace EMT
} // namespace CPS
