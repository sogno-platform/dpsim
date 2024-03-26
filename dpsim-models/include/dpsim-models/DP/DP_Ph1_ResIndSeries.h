/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief resistor inductor series element
class ResIndSeries : public MNASimPowerComp<Complex>,
                     public MNATearInterface,
                     public SharedFactory<ResIndSeries> {
private:
  /// Impedance
  Complex mImpedance;
  /// DC equivalent current source for harmonics [A]
  MatrixComp mEquivCurrent;
  /// Equivalent conductance for harmonics [S]
  MatrixComp mEquivCond;
  /// Coefficient in front of previous current value for harmonics
  MatrixComp mPrevCurrFac;

public:
  /// Inductance [H]
  const Attribute<Real>::Ptr mInductance;
  ///Resistance [ohm]
  const Attribute<Real>::Ptr mResistance;
  /// Defines UID, name and log level
  ResIndSeries(String uid, String name,
               Logger::Level logLevel = Logger::Level::off);
  /// Defines name and log level
  ResIndSeries(String name, Logger::Level logLevel = Logger::Level::off)
      : ResIndSeries(name, name, logLevel) {}

  // #### General ####
  /// Sets model specific parameters
  void setParameters(Real resistance, Real inductance);
  /// Return new instance with the same parameters
  SimPowerComp<Complex>::Ptr clone(String name);
  /// Initializes state variables considering the number of frequencies
  void initialize(Matrix frequencies);
  /// Initializes states from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
  /// Initializes auxiliar variables
  void initVars(Real timeStep);

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
  void mnaCompApplyRightSideVectorStampHarm(Matrix &sourceVector,
                                            Int freqIdx) override;
  /// Update interface current from MNA system results
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  void mnaCompUpdateCurrentHarm();
  /// Add MNA pre step dependencies
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  /// Add MNA post step dependencies
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

  class MnaPreStepHarm : public CPS::Task {
  public:
    MnaPreStepHarm(ResIndSeries &ResIndSeries)
        : Task(**ResIndSeries.mName + ".MnaPreStepHarm"),
          mResIndSeries(ResIndSeries) {
      // actually depends on C, but then we'd have to modify the system matrix anyway
      mModifiedAttributes.push_back(mResIndSeries.attribute("right_vector"));
      mPrevStepDependencies.push_back(mResIndSeries.attribute("i_intf"));
      mPrevStepDependencies.push_back(mResIndSeries.attribute("v_intf"));
    }
    void execute(Real time, Int timeStepCount);

  private:
    ResIndSeries &mResIndSeries;
  };

  class MnaPostStepHarm : public CPS::Task {
  public:
    MnaPostStepHarm(ResIndSeries &ResIndSeries,
                    const std::vector<Attribute<Matrix>::Ptr> &leftVectors)
        : Task(**ResIndSeries.mName + ".MnaPostStepHarm"),
          mResIndSeries(ResIndSeries), mLeftVectors(leftVectors) {
      for (UInt i = 0; i < mLeftVectors.size(); i++)
        mAttributeDependencies.push_back(mLeftVectors[i]);
      mModifiedAttributes.push_back(mResIndSeries.attribute("v_intf"));
      mModifiedAttributes.push_back(mResIndSeries.attribute("i_intf"));
    }
    void execute(Real time, Int timeStepCount);

  private:
    ResIndSeries &mResIndSeries;
    std::vector<Attribute<Matrix>::Ptr> mLeftVectors;
  };

  // #### Tearing methods ####
  void mnaTearInitialize(Real omega, Real timestep) override;
  void mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) override;
  void mnaTearApplyVoltageStamp(Matrix &voltageVector) override;
  void mnaTearPostStep(Complex voltage, Complex current) override;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS