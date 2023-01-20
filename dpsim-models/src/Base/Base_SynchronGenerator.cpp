/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Base/Base_SynchronGenerator.h>

using namespace CPS;

void Base::SynchronGenerator::setBaseParameters(Real nomPower, Real nomVolt,
                                                Real nomFreq) {
  mNomPower = nomPower;
  mNomVolt = nomVolt;
  mNomFreq = nomFreq;
  mNomOmega = nomFreq * 2 * PI;

  // Set base stator values
  mBase_V_RMS = mNomVolt / sqrt(3);
  mBase_V = mBase_V_RMS * sqrt(2);
  mBase_I_RMS = mNomPower / (3 * mBase_V_RMS);
  mBase_I = mBase_I_RMS * sqrt(2);
  mBase_Z = mBase_V / mBase_I;

  mBase_OmElec = mNomOmega;
  mBase_L = mBase_Z / mBase_OmElec;
  mBase_Psi = mBase_L * mBase_I;

  mBase_OmMech = mBase_OmElec;
  mBase_T = mNomPower / mBase_OmMech;
}

void Base::SynchronGenerator::setBaseParameters(Real nomPower, Real nomVolt,
                                                Real nomFreq,
                                                Real nomFieldCur) {
  mNomFieldCur = nomFieldCur;
  Base::SynchronGenerator::setBaseParameters(nomPower, nomVolt, nomFreq);
}

void Base::SynchronGenerator::setBaseAndFundamentalPerUnitParameters(
    Real nomPower, Real nomVolt, Real nomFreq, Real nomFieldCur, Int poleNumber,
    Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd, Real Rkd,
    Real Llkd, Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2, Real inertia) {
  mParameterType = ParameterType::perUnit;
  mNumericalMethod = NumericalMethod::Trapezoidal;

  setBaseParameters(nomPower, nomVolt, nomFreq, nomFieldCur);
  setAndApplyFundamentalPerUnitParameters(poleNumber, Rs, Ll, Lmd, Lmq, Rfd,
                                          Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2,
                                          Llkq2, inertia);
}

void Base::SynchronGenerator::setBaseAndOperationalPerUnitParameters(
    Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
    Real Rs, Real Ld, Real Lq, Real Ld_t, Real Lq_t, Real Ld_s, Real Lq_s,
    Real Ll, Real Td0_t, Real Tq0_t, Real Td0_s, Real Tq0_s, Real inertia) {
  mParameterType = ParameterType::perUnit;
  mNumericalMethod = NumericalMethod::Trapezoidal;

  setBaseParameters(nomPower, nomVolt, nomFreq, nomFieldCur);
  setOperationalPerUnitParameters(poleNumber, inertia, Rs, Ld, Lq, Ll, Ld_t,
                                  Lq_t, Ld_s, Lq_s, Td0_t, Tq0_t, Td0_s, Tq0_s);
  calculateFundamentalFromOperationalParameters();
}

void Base::SynchronGenerator::setOperationalPerUnitParameters(
    Int poleNumber, Real inertia, Real Rs, Real Ld, Real Lq, Real Ll, Real Ld_t,
    Real Lq_t, Real Ld_s, Real Lq_s, Real Td0_t, Real Tq0_t, Real Td0_s,
    Real Tq0_s) {
  mPoleNumber = poleNumber;
  **mInertia = inertia;

  **mRs = Rs;
  **mLl = Ll;
  **mLd = Ld;
  **mLq = Lq;

  **mLd_t = Ld_t;
  **mLq_t = Lq_t;
  **mLd_s = Ld_s;
  **mLq_s = Lq_s;
  **mTd0_t = Td0_t;
  **mTq0_t = Tq0_t;
  **mTd0_s = Td0_s;
  **mTq0_s = Tq0_s;
}

void Base::SynchronGenerator::setAndApplyFundamentalPerUnitParameters(
    Int poleNumber, Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd,
    Real Rkd, Real Llkd, Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
    Real inertia) {
  Base::SynchronGenerator::setFundamentalPerUnitParameters(
      poleNumber, Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2,
      Llkq2, inertia);

  Base::SynchronGenerator::applyFundamentalPerUnitParameters();
}

void Base::SynchronGenerator::setFundamentalPerUnitParameters(
    Int poleNumber, Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd,
    Real Rkd, Real Llkd, Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
    Real inertia) {
  // PoleNumber otherwise not set but currently not used in SynchronGeneratorDQ
  mPoleNumber = poleNumber;
  **mInertia = inertia;

  **mRs = Rs;
  **mLl = Ll;
  mLmd = Lmd;
  mLmq = Lmq;

  mRfd = Rfd;
  mLlfd = Llfd;
  mRkd = Rkd;
  mLlkd = Llkd;
  mRkq1 = Rkq1;
  mLlkq1 = Llkq1;
  mRkq2 = Rkq2;
  mLlkq2 = Llkq2;
}

void Base::SynchronGenerator::applyFundamentalPerUnitParameters() {
  // derive further parameters
  **mLd = **mLl + mLmd;
  **mLq = **mLl + mLmq;
  mLfd = mLlfd + mLmd;
  mLkd = mLlkd + mLmd;
  mLkq1 = mLlkq1 + mLmq;
  mLkq2 = mLlkq2 + mLmq;

  // base rotor values
  mBase_ifd = mLmd * mNomFieldCur;
  mBase_vfd = mNomPower / mBase_ifd;
  mBase_Zfd = mBase_vfd / mBase_ifd;
  mBase_Lfd = mBase_Zfd / mBase_OmElec;

  // derive number of damping windings
  if (mRkq2 == 0 && mLlkq2 == 0)
    mNumDampingWindings = 1;
  else
    mNumDampingWindings = 2;

  if (mNumDampingWindings == 1) {
    mVsr = Matrix::Zero(6, 1);
    mIsr = Matrix::Zero(6, 1);
    mPsisr = Matrix::Zero(6, 1);
    mInductanceMat = Matrix::Zero(6, 6);
    mResistanceMat = Matrix::Zero(6, 6);

    // Determinant of Lq(inductance matrix of q axis)
    // Real detLq = -(mLl + mLmq)*(mLlkq1 + mLmq) + mLmq*mLmq;
    // Determinant of Ld (inductance matrix of d axis)
    // Real detLd = (mLmd + mLl)*(-mLlfd*mLlkd - mLlfd*mLmd - mLmd*mLlkd) +
    // mLmd*mLmd*(mLlfd + mLlkd);

    mInductanceMat << **mLd, mLmd, mLmd, 0, 0, 0, mLmd, mLfd, mLmd, 0, 0, 0,
        mLmd, mLmd, mLkd, 0, 0, 0, 0, 0, 0, **mLq, mLmq, 0, 0, 0, 0, mLmq, mLkq1,
        0, 0, 0, 0, 0, 0, **mLl;

    mResistanceMat << **mRs, 0, 0, 0, 0, 0, 0, mRfd, 0, 0, 0, 0, 0, 0, mRkd, 0,
        0, 0, 0, 0, 0, **mRs, 0, 0, 0, 0, 0, 0, mRkq1, 0, 0, 0, 0, 0, 0, **mRs;

    // Compute inverse Inductance Matrix:
    mInvInductanceMat = mInductanceMat.inverse();
  } else {
    mVsr = Matrix::Zero(7, 1);
    mIsr = Matrix::Zero(7, 1);
    mPsisr = Matrix::Zero(7, 1);
    mInductanceMat = Matrix::Zero(7, 7);
    mResistanceMat = Matrix::Zero(7, 7);

    // Determinant of Lq(inductance matrix of q axis)
    // Real detLq = -mLmq*mLlkq2*(mLlkq1 + mLl) - mLl*mLlkq1*(mLlkq2 + mLmq);
    // Determinant of Ld (inductance matrix of d axis)
    // Real detLd = (mLmd + mLl)*(-mLlfd*mLlkd - mLlfd*mLmd - mLmd*mLlkd) +
    // mLmd*mLmd*(mLlfd + mLlkd);

    mInductanceMat << **mLd, mLmd, mLmd, 0, 0, 0, 0, mLmd, mLfd, mLmd, 0, 0, 0,
        0, mLmd, mLmd, mLkd, 0, 0, 0, 0, 0, 0, 0, **mLq, mLmq, mLmq, 0, 0, 0, 0,
        mLmq, mLkq1, mLmq, 0, 0, 0, 0, mLmq, mLmq, mLkq2, 0, 0, 0, 0, 0, 0, 0,
        **mLl;

    mResistanceMat << **mRs, 0, 0, 0, 0, 0, 0, 0, mRfd, 0, 0, 0, 0, 0, 0, 0,
        mRkd, 0, 0, 0, 0, 0, 0, 0, **mRs, 0, 0, 0, 0, 0, 0, 0, mRkq1, 0, 0, 0,
        0, 0, 0, 0, mRkq2, 0, 0, 0, 0, 0, 0, 0, **mRs;

    // Compute inverse Inductance Matrix:
    mInvInductanceMat = mInductanceMat.inverse();
  }
}

void Base::SynchronGenerator::calculateFundamentalFromOperationalParameters() {
  mLmd = **mLd - **mLl;
  mLmq = **mLq - **mLl;

  mLlfd = mLmd * (**mLd_t - **mLl) / (mLmd - **mLd_t + **mLl);
  mLlkq1 = mLmq * (**mLq_t - **mLl) / (mLmq - **mLq_t + **mLl);

  mLlkd = mLmd * mLlfd * (**mLd_s - **mLl) /
          (mLlfd * mLmd - (mLmd + mLlfd) * (**mLd_s - **mLl));
  mLlkq2 = mLmq * mLlkq1 * (**mLq_s - **mLl) /
           (mLlkq1 * mLmq - (mLmq + mLlkq1) * (**mLq_s - **mLl));

  mRfd = (mLmd + mLlfd) / (**mTd0_t * mNomOmega);
  mRkd = (1 / (**mTd0_s * mNomOmega)) * (mLlkd + mLmd * mLlfd / (mLmd + mLlfd));
  mRkq1 = (mLmq + mLlkq1) / (**mTq0_t * mNomOmega);
  mRkq2 =
      (1 / (**mTq0_s * mNomOmega)) * (mLlkq2 + mLmq * mLlkq1 / (mLmq + mLlkq1));

  Base::SynchronGenerator::applyFundamentalPerUnitParameters();
}

void Base::SynchronGenerator::setInitialValues(Real initActivePower,
                                               Real initReactivePower,
                                               Real initTerminalVolt,
                                               Real initVoltAngle,
                                               Real initMechPower) {
  mInitElecPower = Complex(initActivePower, initReactivePower);
  mInitTerminalVoltage = initTerminalVolt;
  mInitVoltAngle = initVoltAngle;
  mInitMechPower = initMechPower;
  mInitialValuesSet = true;
}

void Base::SynchronGenerator::initPerUnitStates() {
  // Power in per unit
  Real init_P = mInitElecPower.real() / mNomPower;
  Real init_Q = mInitElecPower.imag() / mNomPower;
  Real init_S_abs = sqrt(pow(init_P, 2.) + pow(init_Q, 2.));
  // Complex init_S = mInitElecPower;
  // Terminal voltage in pu
  Real init_vt_abs = mInitTerminalVoltage / mBase_V;
  // Complex init_vt = Complex(mInitTerminalVoltage*cos(mInitVoltAngle),
  // mInitTerminalVoltage*sin(mInitVoltAngle));
  Real init_it_abs = init_S_abs / init_vt_abs;
  // Complex init_it = std::conj( init_S / init_vt );
  // Power factor
  Real init_pf = acos(init_P / init_S_abs);
  // Load angle
  Real init_delta = atan(((mLmq + **mLl) * init_it_abs * cos(init_pf) -
                          **mRs * init_it_abs * sin(init_pf)) /
                         (init_vt_abs + **mRs * init_it_abs * cos(init_pf) +
                          (mLmq + **mLl) * init_it_abs * sin(init_pf)));
  // Real init_delta_deg = init_delta / PI * 180;

  // Electrical torque
  // Real init_Te = init_P + **mRs * pow(init_it, 2.);

  // dq stator voltages and currents
  Real init_vd = init_vt_abs * sin(init_delta);
  Real init_vq = init_vt_abs * cos(init_delta);
  Real init_id = init_it_abs * sin(init_delta + init_pf);
  Real init_iq = init_it_abs * cos(init_delta + init_pf);

  // Rotor voltage and current
  Real init_ifd = (init_vq + **mRs * init_iq + (mLmd + **mLl) * init_id) / mLmd;
  Real init_vfd = mRfd * init_ifd;

  // Flux linkages
  Real init_psid = init_vq + **mRs * init_iq;
  Real init_psiq = -init_vd - **mRs * init_id;
  Real init_psifd = mLfd * init_ifd - mLmd * init_id;
  Real init_psikd = mLmd * (init_ifd - init_id);
  Real init_psiq1 = -mLmq * init_iq;
  Real init_psiq2 = -mLmq * init_iq;

  // Initialize mechanical variables
  **mOmMech = 1;
  **mMechPower = mInitMechPower / mNomPower;
  **mMechTorque = **mMechPower / 1;
  mThetaMech = mInitVoltAngle + init_delta - PI / 2.;
  **mDelta = init_delta;

  if (mNumDampingWindings == 2) {
    mVsr << init_vd, init_vfd, 0, init_vq, 0, 0, 0;
    mIsr << -init_id, init_ifd, 0, -init_iq, 0, 0, 0;
    mPsisr << init_psid, init_psifd, init_psikd, init_psiq, init_psiq1,
        init_psiq2, 0;
  } else {
    mVsr << init_vd, init_vfd, 0, init_vq, 0, 0;
    mIsr << -init_id, init_ifd, 0, -init_iq, 0, 0;
    mPsisr << init_psid, init_psifd, init_psikd, init_psiq, init_psiq1, 0;
  }

	if (mNumDampingWindings == 2) {
		mVsr << init_vd, init_vfd, 0, init_vq, 0, 0, 0;
		mIsr << -init_id, init_ifd, 0, -init_iq, 0, 0, 0;
		mPsisr << init_psid, init_psifd, init_psikd, init_psiq, init_psiq1, init_psiq2, 0;
	}
	else {
		mVsr << init_vd, init_vfd, 0, init_vq, 0, 0;
		mIsr << -init_id, init_ifd, 0, -init_iq, 0, 0;
		mPsisr << init_psid, init_psifd, init_psikd, init_psiq, init_psiq1, 0;
	}

	**mElecTorque = (mPsisr(3,0)*mIsr(0,0) - mPsisr(0,0)*mIsr(3,0));
	**mVfd = (mLmd / mRfd) * init_vfd;

	// Initialize controllers
	if (mHasExciter){
		// Note: field voltage scaled by Lmd/Rfd to transform from synchronous generator pu system
		// to the exciter pu system
		mExciter->initialize(init_vt_abs, (mLmd / mRfd)*init_vfd);
	}
}

void Base::SynchronGenerator::calcStateSpaceMatrixDQ() {
  if (mNumDampingWindings == 2) {
    mLad = 1. / (1. / mLmd + 1. / **mLl + 1. / mLlfd + 1. / mLlkd);
    mLaq = 1. / (1. / mLmq + 1. / **mLl + 1. / mLlkq1 + 1. / mLlkq2);

    mPsisr = Matrix::Zero(7, 1);

    mOmegaFluxMat = Matrix::Zero(7, 7);
    mOmegaFluxMat << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0;

    mFluxStateSpaceMat =
        Matrix::Zero(7, 7);  // order of lambdas: ds; fd; kd; qs; kq1; kq2; 0s
    mFluxStateSpaceMat << **mRs / **mLl * mLad / **mLl - **mRs / **mLl,
        **mRs / **mLl * mLad / mLlfd, **mRs / **mLl * mLad / mLlkd, 0, 0, 0, 0,
        mRfd / mLlfd * mLad / **mLl, mRfd / mLlfd * mLad / mLlfd - mRfd / mLlfd,
        mRfd / mLlfd * mLad / mLlkd, 0, 0, 0, 0, mRkd / mLlkd * mLad / **mLl,
        mRkd / mLlkd * mLad / mLlfd, mRkd / mLlkd * mLad / mLlkd - mRkd / mLlkd,
        0, 0, 0, 0, 0, 0, 0, **mRs / **mLl * mLaq / **mLl - **mRs / **mLl,
        **mRs / **mLl * mLaq / mLlkq1, **mRs / **mLl * mLaq / mLlkq2, 0, 0, 0,
        0, mRkq1 / mLlkq1 * mLaq / **mLl,
        mRkq1 / mLlkq1 * mLaq / mLlkq1 - mRkq1 / mLlkq1,
        mRkq1 / mLlkq1 * mLaq / mLlkq2, 0, 0, 0, 0,
        mRkq2 / mLlkq2 * mLaq / **mLl, mRkq2 / mLlkq2 * mLaq / mLlkq1,
        mRkq2 / mLlkq2 * mLaq / mLlkq2 - mRkq2 / mLlkq2, 0, 0, 0, 0, 0, 0, 0,
        -**mRs / **mLl;

    mFluxToCurrentMat = Matrix::Zero(
        7, 7);  // need for electric torque id, iq ->1st and 4th row
    mFluxToCurrentMat << 1. / **mLl - mLad / **mLl / **mLl,
        -mLad / mLlfd / **mLl, -mLad / mLlkd / **mLl, 0, 0, 0, 0,
        -mLad / **mLl / mLlfd, 1. / mLlfd - mLad / mLlfd / mLlfd,
        -mLad / mLlkd / mLlfd, 0, 0, 0, 0, -mLad / **mLl / mLlkd,
        -mLad / mLlfd / mLlkd, 1. / mLlkd - mLad / mLlkd / mLlkd, 0, 0, 0, 0, 0,
        0, 0, 1. / **mLl - mLaq / **mLl / **mLl, -mLaq / mLlkq1 / **mLl,
        -mLaq / mLlkq2 / **mLl, 0, 0, 0, 0, -mLaq / **mLl / mLlkq1,
        1. / mLlkq1 - mLaq / mLlkq1 / mLlkq1, -mLaq / mLlkq2 / mLlkq1, 0, 0, 0,
        0, -mLaq / **mLl / mLlkq2, -mLaq / mLlkq1 / mLlkq2,
        1. / mLlkq2 - mLaq / mLlkq2 / mLlkq2, 0, 0, 0, 0, 0, 0, 0, 1. / **mLl;
  } else {
    mLad = 1. / (1. / mLmd + 1. / **mLl + 1. / mLlfd + 1. / mLlkd);
    mLaq = 1. / (1. / mLmq + 1. / **mLl + 1. / mLlkq1);

    mPsisr = Matrix::Zero(6, 1);

    mOmegaFluxMat = Matrix::Zero(6, 6);
    mOmegaFluxMat << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    mFluxStateSpaceMat = Matrix::Zero(6, 6);
    mFluxStateSpaceMat <<  // same order as above; only without kq2
        **mRs / **mLl * mLad / **mLl - **mRs / **mLl,
        **mRs / **mLl * mLad / mLlfd, **mRs / **mLl * mLad / mLlkd, 0, 0, 0,
        mRfd / mLlfd * mLad / **mLl, mRfd / mLlfd * mLad / mLlfd - mRfd / mLlfd,
        mRfd / mLlfd * mLad / mLlkd, 0, 0, 0, mRkd / mLlkd * mLad / **mLl,
        mRkd / mLlkd * mLad / mLlfd, mRkd / mLlkd * mLad / mLlkd - mRkd / mLlkd,
        0, 0, 0, 0, 0, 0, **mRs / **mLl * mLaq / **mLl - **mRs / **mLl,
        **mRs / **mLl * mLaq / mLlkq1, 0, 0, 0, 0,
        mRkq1 / mLlkq1 * mLaq / **mLl,
        mRkq1 / mLlkq1 * mLaq / mLlkq1 - mRkq1 / mLlkq1, 0, 0, 0, 0, 0, 0,
        -**mRs / **mLl;

    mFluxToCurrentMat = Matrix::Zero(6, 6);
    mFluxToCurrentMat << 1. / **mLl - mLad / **mLl / **mLl,
        -mLad / mLlfd / **mLl, -mLad / mLlkd / **mLl, 0, 0, 0,
        -mLad / **mLl / mLlfd, 1. / mLlfd - mLad / mLlfd / mLlfd,
        -mLad / mLlkd / mLlfd, 0, 0, 0, -mLad / **mLl / mLlkd,
        -mLad / mLlfd / mLlkd, 1. / mLlkd - mLad / mLlkd / mLlkd, 0, 0, 0, 0, 0,
        0, 1. / **mLl - mLaq / **mLl / **mLl, -mLaq / mLlkq1 / **mLl, 0, 0, 0,
        0, -mLaq / **mLl / mLlkq1, 1. / mLlkq1 - mLaq / mLlkq1 / mLlkq1, 0, 0,
        0, 0, 0, 0, 1. / **mLl;
  }
}

Real Base::SynchronGenerator::calcHfromJ(Real J, Real omegaNominal,
                                         Int polePairNumber) {
  return J * 0.5 * omegaNominal * omegaNominal / polePairNumber;
}

void Base::SynchronGenerator::addExciter(std::shared_ptr<Base::Exciter> exciter) {
	mExciter = exciter;
	mHasExciter = true;
}

void Base::SynchronGenerator::addGovernor(Real Ta, Real Tb, Real Tc, Real Fa,
	Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef) {
	mTurbineGovernor = Signal::TurbineGovernor::make("TurbineGovernor", CPS::Logger::Level::info);
  mTurbineGovernor->setParameters(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm);
	mTurbineGovernor->initialize(PmRef, Tm_init);
	mHasTurbineGovernor = true;
}
