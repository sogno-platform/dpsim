/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_VoltageSource.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// average inverter model with LC filter
class AvVoltSourceInverterStateSpace
    : public MNASimPowerComp<Real>,
      public Base::Ph1::VoltageSource,
      public SharedFactory<AvVoltSourceInverterStateSpace> {
protected:
  Real mTimeStep;

  /// filter paramter
  Real mLf;
  Real mCf;
  Real mRf;

  /// PLL
  Real mOmegaN;
  Real mKiPLL;
  Real mKpPLL;

  /// Power controller
  Real mOmegaCutoff;
  Real mKiPowerCtrld;
  Real mKiPowerCtrlq;
  Real mKpPowerCtrld;
  Real mKpPowerCtrlq;

  /// Current controller
  Real mKiCurrCtrld;
  Real mKiCurrCtrlq;
  Real mKpCurrCtrld;
  Real mKpCurrCtrlq;

  /// connection to grid
  Real mRc;

public:
  // ### parameters ###
  const Attribute<Real>::Ptr mPref;
  const Attribute<Real>::Ptr mQref;

  // states
  const Attribute<Real>::Ptr mThetaPLL;
  const Attribute<Real>::Ptr mPhiPLL;

  const Attribute<Real>::Ptr mP;
  const Attribute<Real>::Ptr mQ;

  const Attribute<Real>::Ptr mPhi_d;
  const Attribute<Real>::Ptr mPhi_q;

  const Attribute<Real>::Ptr mGamma_d;
  const Attribute<Real>::Ptr mGamma_q;

  const Attribute<Matrix>::Ptr mVcabc;

protected:
  Matrix mIfabc = Matrix::Zero(3, 1);
  /*Real mIfa;
		Real mIfb;
		Real mIfc;*/

  /*Real mVca;
		Real mVcb;
		Real mVcc;*/

  // Norton equivalant voltage source
  Matrix mEquivCurrent = Matrix::Zero(3, 1);
  //  ### Real Voltage source parameters ###
  /// conductance of mRc[S]
  Real mYc;
  // #### Matrices ####
  Matrix mStates;
  // u_old
  Matrix mU;
  /// output
  Matrix mIg_abc = Matrix::Zero(3, 1);
  Matrix mA;
  Matrix mB;
  Matrix mC;
  Matrix mD;
  // park transform matrix
  Matrix mParkTransform;

public:
  AvVoltSourceInverterStateSpace(String uid, String name,
                                 Logger::Level logLevel = Logger::Level::off);
  AvVoltSourceInverterStateSpace(String name,
                                 Logger::Level logLevel = Logger::Level::off)
      : AvVoltSourceInverterStateSpace(name, name, logLevel) {}

  // initialize with parameters already set.
  // sysVoltNom: phase voltage

  void initializeStates(Real omega, Real timeStep,
                        Attribute<Matrix>::Ptr leftVector);

  // initialize with parameters.
  //void initialize(Real theta, Real phi_pll, Real p, Real q, Real phi_d, Real phi_q,
  //	Real gamma_d, Real gamma_q, Real i_fd, Real i_fq, Real v_cd, Real v_cq);

  //void initStates(Real initOmegaPLL, Real initPhiPLL, Real initP, Real initQ,
  //	Real initPhid, Real initPhiQ, Real initGamma_d, Real initGamma_q, Real initVcabc, Real initIfabc);

  void updateStates();

  void setParameters(Real sysOmega, Complex sysVoltNom, Real Pref, Real Qref,
                     Real Lf, Real Cf, Real Rf, Real Rc, Real Kp_pll,
                     Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl,
                     Real Kp_currCtrl, Real Ki_currCtrl);

  void setFilterParameters(Real Lf, Real Cf, Real Rf);

  void setControllerParameters(Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl,
                               Real Ki_powerCtrl, Real Kp_currCtrl,
                               Real Ki_currCtrl);

  //update park transform coefficients inside A, B matrices according to the new states (thea_pll)
  //update Ig_abc in matrix B
  void updateLinearizedCoeffs();

  Matrix getParkTransformMatrix(Real theta);
  Matrix getInverseParkTransformMatrix(Real theta);
  Matrix parkTransform(Real theta, Real fa, Real fb, Real fc);
  Matrix inverseParkTransform(Real theta, Real fd, Real fq, Real zero = 0.);

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Returns current through the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// update equivalent current of the equivalent source
  void updateEquivCurrent(Real time);

  void mnaCompPreStep(Real time, Int timeStepCount) override;
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
};
} // namespace Ph3
} // namespace EMT
} // namespace CPS
