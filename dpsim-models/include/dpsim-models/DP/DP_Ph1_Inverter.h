/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <map>

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// @brief Single phase inverter model
///
/// add more explanation here regarding bessel function model
class Inverter : public MNASimPowerComp<Complex>,
                 public SharedFactory<Inverter> {
protected:
  // #### Model specific variables ####
  /// DC bus voltage
  Real mVin = 360;
  /// system frequency (should be updated every step)
  Real mFreqMod = 50;
  /// system angular frequency
  Real mOmMod = 2. * PI * mFreqMod;
  /// switching frequency (constant)
  Real mFreqCar = 10e3;
  /// switching angular frequency
  Real mOmCar = 2. * PI * mFreqCar;
  /// Modulation Index
  Real mModIdx = 0.87;
  //mMr = sqrt(2)*mV_grid/mV_in;
  /// Carrier phase
  Real mPhaseCar = 0;
  /// Modulation phase
  Real mPhaseMod = 0;

  /// Number of harmonics
  UInt mHarNum;
  /// Maximum number of carrier signal harmonics
  Int mMaxCarrierHarm = 2;
  /// Maximum number of modulation signal harmonics
  Int mMaxModulHarm = 3;
  ///
  UInt mCarHarNum;
  ///
  UInt mModHarNum;
  ///
  MatrixInt mHarmMap;
  /// Maximum upper limit for Bessel function 1st kind summation
  Int mMaxBesselSumIdx = 20;
  /// Vector of carrier signal harmonics
  std::vector<Int> mCarHarms;
  /// Vector of modulation signal harmonics
  std::vector<Int> mModHarms;

  /// voltage part of system fundamental
  Real mVfund = 0;
  /// Vector of phasor frequencies
  Matrix mPhasorFreqs;
  /// Vector of phasor magnitudes
  Matrix mPhasorMags;
  /// Vector of phasor phases
  Matrix mPhasorPhases;
  ///
  std::vector<Real> mFactorials;
  ///
  std::map<Int, Real> mMultInvFactorials;

  void generateFrequencies();

  // #### Math functions ####

  /// Bessel function
  Real besselFirstKind_n(Int n, Int k_max, Real x) const;
  /// Bessel function using look up tables for factorials
  Real besselFirstKind_n_opt(Int n, Int k_max, Real x);
  long long factorial(Int n) const;
  Real multInvFactorial(Int n) const;
  Real multInvIntGamma(Real n) const;

public:
  /// Defines UID, name and logging level
  Inverter(String name, String uid,
           Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  Inverter(String name, Logger::Level logLevel = Logger::Level::off)
      : Inverter(name, name, logLevel) {}

  // #### General ####
  ///
  void initializeFromNodesAndTerminals(Real frequency) override;
  ///
  void initialize(Matrix frequencies) override;
  ///
  void setParameters(const std::vector<Int> &carrierHarms,
                     const std::vector<Int> &modulHarms, Real inputVoltage,
                     Real ratio, Real phase);
  ///
  void calculatePhasors();

  // #### MNA Functions ####
  /// Initializes internal variables of the component
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
  void mnaCompApplyRightSideVectorStampHarm(Matrix &sourceVector,
                                            Int freqIdx) override;
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

  class MnaPreStepHarm : public CPS::Task {
  public:
    MnaPreStepHarm(Inverter &inverter)
        : Task(**inverter.mName + ".MnaPreStepHarm"), mInverter(inverter) {
      mModifiedAttributes.push_back(mInverter.attribute("right_vector"));
      mModifiedAttributes.push_back(mInverter.attribute("v_intf"));
    }
    void execute(Real time, Int timeStepCount);

  private:
    Inverter &mInverter;
  };

  class MnaPostStepHarm : public CPS::Task {
  public:
    MnaPostStepHarm(Inverter &inverter,
                    const std::vector<Attribute<Matrix>::Ptr> &leftVectors)
        : Task(**inverter.mName + ".MnaPostStepHarm"), mInverter(inverter),
          mLeftVectors(leftVectors) {
      for (UInt i = 0; i < mLeftVectors.size(); i++)
        mAttributeDependencies.push_back(mLeftVectors[i]);
      mModifiedAttributes.push_back(mInverter.attribute("i_intf"));
    }
    void execute(Real time, Int timeStepCount);

  private:
    Inverter &mInverter;
    std::vector<Attribute<Matrix>::Ptr> mLeftVectors;
  };
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
