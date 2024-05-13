/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_SynchronGenerator.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph1_Inductor.h>
#include <dpsim-models/DP/DP_Ph1_VoltageSource.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// @brief Synchronous generator model for transient stability analysis
///
/// This model is based on Eremia section 2.1.6.
class SynchronGeneratorTrStab : public CompositePowerComp<Complex>,
                                public Base::SynchronGenerator,
                                public SharedFactory<SynchronGeneratorTrStab> {
protected:
  // #### Model specific variables ####
  /// Absolute d-axis transient reactance X'd
  Real mXpd;
  /// Absolute d-axis transient inductance
  Real mLpd;
  /// Absolute damping coefficient
  Real mKd;
  /// Equivalent impedance for loadflow calculation
  Complex mImpedance;
  /// Inner voltage source that represents the generator
  std::shared_ptr<VoltageSource> mSubVoltageSource;
  /// Inner inductor that represents the generator impedance
  std::shared_ptr<Inductor> mSubInductor;
  /// Logging
  Matrix mStates;
  /// Flag for usage of attribute of w_ref (otherwise mNomOmega is used)
  Bool mUseOmegaRef = false;
  /// Flag for usage of actual mechanical speed for torque conversion (otherwise mNomOmega is used)
  Bool mConvertWithOmegaMech = true;

public:
  // #### Model specific variables ####
  /// emf behind transient reactance
  const Attribute<Complex>::Ptr mEp;
  /// fixed absolute value of emf behind transient reactance
  /// CHECK: Is this necessary / could this be derived from mEp?
  const Attribute<Real>::Ptr mEp_abs;
  /// Angle by which the emf Ep is leading the system reference frame
  /// CHECK: Is this necessary / could this be derived from mEp?
  const Attribute<Real>::Ptr mEp_phase;
  /// Angle by which the emf Ep is leading the terminal voltage
  const Attribute<Real>::Ptr mDelta_p;
  ///
  const Attribute<Real>::Ptr mRefOmega;
  ///
  const Attribute<Real>::Ptr mRefDelta;
  ///
  SynchronGeneratorTrStab(String uid, String name,
                          Logger::Level logLevel = Logger::Level::off);
  ///
  SynchronGeneratorTrStab(String name,
                          Logger::Level logLevel = Logger::Level::off)
      : SynchronGeneratorTrStab(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General Functions ####
  /// Flags to modify model behavior
  void setModelFlags(Bool convertWithOmegaMech);
  ///
  void setInitialValues(Complex elecPower, Real mechPower);
  /// \brief Initializes the machine parameters
  void setFundamentalParametersPU(Real nomPower, Real nomVolt, Real nomFreq,
                                  Real Ll, Real Lmd, Real Llfd, Real inertia,
                                  Real D = 0);
  /// \brief Initializes the machine parameters
  void setStandardParametersSI(Real nomPower, Real nomVolt, Real nomFreq,
                               Int polePairNumber, Real Rs, Real Lpd,
                               Real inertiaJ, Real Kd = 0);
  /// \brief Initializes the machine parameters
  void setStandardParametersPU(Real nomPower, Real nomVolt, Real nomFreq,
                               Real Xpd, Real inertia, Real Rs = 0, Real D = 0);
  ///
  void step(Real time);
  ///
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA Functions ####
  /// Initializes variables of component
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override;
  /// Performs with the model of a synchronous generator
  /// to calculate the flux and current from the voltage vector.
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  /// Retrieves calculated voltage from simulation for next step
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  ///
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  ///
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;

  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;

  void setReferenceOmega(Attribute<Real>::Ptr refOmegaPtr,
                         Attribute<Real>::Ptr refDeltaPtr);

  class AddBStep : public Task {
  public:
    AddBStep(SynchronGeneratorTrStab &generator)
        : Task(**generator.mName + ".AddBStep"), mGenerator(generator) {
      mAttributeDependencies.push_back(
          generator.mSubVoltageSource->mRightVector);
      mAttributeDependencies.push_back(generator.mSubInductor->mRightVector);
      mModifiedAttributes.push_back(generator.mRightVector);
    }

    void execute(Real time, Int timeStepCount);

  private:
    SynchronGeneratorTrStab &mGenerator;
  };
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
