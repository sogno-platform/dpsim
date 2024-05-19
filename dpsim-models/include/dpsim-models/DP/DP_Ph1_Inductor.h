/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Inductor.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/EigenvalueDynamicCompInterface.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief Inductor
///
/// The inductor is represented by a DC equivalent circuit which corresponds to
/// one iteration of the trapezoidal integration method.
/// The equivalent DC circuit is a resistance in parallel with a current source.
/// The resistance is constant for a defined time step and system
/// frequency and the current source changes for each iteration.
class Inductor : public MNASimPowerComp<Complex>,
                 public Base::Ph1::Inductor,
                 public MNATearInterface,
                 public SharedFactory<Inductor>,
                 public EigenvalueDynamicCompInterface<Complex> {
protected:
  /// DC equivalent current source for harmonics [A]
  MatrixComp mEquivCurrent;
  /// Equivalent conductance for harmonics [S]
  MatrixComp mEquivCond;
  /// Coefficient in front of previous current value for harmonics
  MatrixComp mPrevCurrFac;
  ///
  void initVars(Real timeStep);

public:
  /// Defines UID, name and log level
  Inductor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);
  /// Defines name and log level
  Inductor(String name, Logger::Level logLevel = Logger::Level::off)
      : Inductor(name, name, logLevel) {}

  // #### General ####
  /// Return new instance with the same parameters
  SimPowerComp<Complex>::Ptr clone(String name) override;
  /// Initializes state variables considering the number of frequencies
  void initialize(Matrix frequencies) override;
  /// Initializes states from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Initializes MNA specific variables
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompInitializeHarm(
      Real omega, Real timeStep,
      std::vector<Attribute<Matrix>::Ptr> leftVectors) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix,
                                         Int freqIdx) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompApplyRightSideVectorStampHarm(Matrix &rightVector) override;
  /// Update interface voltage from MNA system results
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateVoltageHarm(const Matrix &leftVector, Int freqIdx);
  /// Update interface current from MNA system results
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  void mnaCompUpdateCurrentHarm();
  /// MNA pre step operations
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
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

  // #### Tearing methods ####
  void mnaTearInitialize(Real omega, Real timestep) override;
  void mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) override;
  void mnaTearApplyVoltageStamp(Matrix &voltageVector) override;
  void mnaTearPostStep(Complex voltage, Complex current) override;

  class MnaPreStepHarm : public Task {
  public:
    MnaPreStepHarm(Inductor &inductor)
        : Task(**inductor.mName + ".MnaPreStepHarm"), mInductor(inductor) {
      // actually depends on L, but then we'd have to modify the system matrix anyway
      mModifiedAttributes.push_back(inductor.attribute("right_vector"));
      mPrevStepDependencies.push_back(inductor.attribute("v_intf"));
      mPrevStepDependencies.push_back(inductor.attribute("i_intf"));
    }
    void execute(Real time, Int timeStepCount);

  private:
    Inductor &mInductor;
  };

  class MnaPostStepHarm : public Task {
  public:
    MnaPostStepHarm(Inductor &inductor,
                    const std::vector<Attribute<Matrix>::Ptr> &leftVectors)
        : Task(**inductor.mName + ".MnaPostStepHarm"), mInductor(inductor),
          mLeftVectors(leftVectors) {
      for (UInt i = 0; i < mLeftVectors.size(); i++)
        mAttributeDependencies.push_back(mLeftVectors[i]);
      mModifiedAttributes.push_back(mInductor.attribute("v_intf"));
      mModifiedAttributes.push_back(mInductor.attribute("i_intf"));
    }
    void execute(Real time, Int timeStepCount);

  private:
    Inductor &mInductor;
    std::vector<Attribute<Matrix>::Ptr> mLeftVectors;
  };

  // #### Implementation of eigenvalue dynamic component interface ####
  void stampSignMatrix(MatrixVar<Complex> &signMatrix, Complex coeffDP) final;
  void stampDiscretizationMatrix(MatrixVar<Complex> &discretizationMatrix,
                                 Complex coeffDP) final;
  void stampBranchNodeIncidenceMatrix(Matrix &branchNodeIncidenceMatrix) final;
  void setBranchIdx(UInt i) final;

private:
  /// Branch index
  UInt mBranchIdx;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
