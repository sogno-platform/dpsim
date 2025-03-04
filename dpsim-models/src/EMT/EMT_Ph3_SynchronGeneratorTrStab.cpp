/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGeneratorTrStab.h>
using namespace CPS;

Matrix EMT::Ph3::SynchronGeneratorTrStab::parkTransformPowerInvariant(
    Real theta, const Matrix &fabc) {
  // Calculates fdq = Tdq * fabc
  // Assumes that d-axis starts aligned with phase a
  Matrix Tdq = getParkTransformMatrixPowerInvariant(theta);
  Matrix dqvector = Tdq * fabc;
  return dqvector;
}

Matrix EMT::Ph3::SynchronGeneratorTrStab::getParkTransformMatrixPowerInvariant(
    Real theta) {
  // Return park matrix for theta
  // Assumes that d-axis starts aligned with phase a
  Matrix Tdq = Matrix::Zero(2, 3);
  Real k = sqrt(2. / 3.);
  Tdq << k * cos(theta), k * cos(theta - 2. * M_PI / 3.),
      k * cos(theta + 2. * M_PI / 3.), -k * sin(theta),
      -k * sin(theta - 2. * M_PI / 3.), -k * sin(theta + 2. * M_PI / 3.);
  return Tdq;
}

EMT::Ph3::SynchronGeneratorTrStab::SynchronGeneratorTrStab(
    String uid, String name, Logger::Level logLevel)
    : Base::SynchronGenerator(mAttributes), CompositePowerComp<Real>(uid, name,
                                                                     true, true,
                                                                     logLevel),
      mEp(mAttributes->create<Complex>("Ep")),
      mEp_abs(mAttributes->create<Real>("Ep_mag")),
      mEp_phase(mAttributes->create<Real>("Ep_phase")),
      mDelta_p(mAttributes->create<Real>("delta_r")),
      mRefOmega(mAttributes->createDynamic<Real>("w_ref")),
      mRefDelta(mAttributes->createDynamic<Real>("delta_ref")) {
  setVirtualNodeNumber(2);
  setTerminalNumber(1);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);

  mStates = Matrix::Zero(10, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph3::SynchronGeneratorTrStab::clone(String name) {
  auto copy = SynchronGeneratorTrStab::make(name, mLogLevel);
  copy->setStandardParametersPU(mNomPower, mNomVolt, mNomFreq, mXpd / mBase_Z,
                                **mInertia, **mRs, mKd);
  return copy;
}

void EMT::Ph3::SynchronGeneratorTrStab::setFundamentalParametersPU(
    Real nomPower, Real nomVolt, Real nomFreq, Real Ll, Real Lmd, Real Llfd,
    Real inertia, Real D) {
  setBaseParameters(nomPower, nomVolt, nomFreq);

  // Input is in per unit but all values are converted to absolute values.
  mParameterType = ParameterType::statorReferred;
  mStateType = StateType::statorReferred;

  **mLl = Ll;
  mLmd = Lmd;
  **mLd = **mLl + mLmd;
  mLlfd = Llfd;
  mLfd = mLlfd + mLmd;
  // M = 2*H where H = inertia
  **mInertia = inertia;
  // X'd in absolute values
  mXpd = mNomOmega * (**mLd - mLmd * mLmd / mLfd) * mBase_L;
  mLpd = (**mLd - mLmd * mLmd / mLfd) * mBase_L;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Parameters ---"
                     "\nimpedance: {:f}"
                     "\ninductance: {:f}",
                     mXpd, mLpd);
}

void EMT::Ph3::SynchronGeneratorTrStab::setStandardParametersSI(
    Real nomPower, Real nomVolt, Real nomFreq, Int polePairNumber, Real Rs,
    Real Lpd, Real inertiaJ, Real Kd) {
  setBaseParameters(nomPower, nomVolt, nomFreq);

  mParameterType = ParameterType::statorReferred;
  mStateType = StateType::statorReferred;

  // M = 2*H where H = inertia
  // H = J * 0.5 * omegaNom^2 / polePairNumber
  **mInertia = calcHfromJ(inertiaJ, 2 * PI * nomFreq, polePairNumber);
  // X'd in absolute values
  mXpd = mNomOmega * Lpd;
  mLpd = Lpd;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Parameters ---"
                     "\nimpedance: {:f}"
                     "\ninductance: {:f}",
                     mXpd, mLpd);
}

void EMT::Ph3::SynchronGeneratorTrStab::setStandardParametersPU(
    Real nomPower, Real nomVolt, Real nomFreq, Real Xpd, Real inertia, Real Rs,
    Real D) {
  setBaseParameters(nomPower, nomVolt, nomFreq);

  // Input is in per unit but all values are converted to absolute values.
  mParameterType = ParameterType::statorReferred;
  mStateType = StateType::statorReferred;

  // M = 2*H where H = inertia
  **mInertia = inertia;
  // X'd in absolute values
  mXpd = Xpd * mBase_Z;
  mLpd = Xpd * mBase_L;

  **mRs = Rs;
  //The units of D are per unit power divided by per unit speed deviation.
  // D is transformed to an absolute value to obtain Kd, which will be used in the swing equation
  mKd = D * mNomPower / mNomOmega;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Parameters ---"
                     "\nimpedance: {:f}"
                     "\ninductance: {:f}",
                     mXpd, mLpd);
}

void EMT::Ph3::SynchronGeneratorTrStab::setInitialValues(Complex elecPower,
                                                         Real mechPower) {
  mInitElecPower = elecPower;
  mInitMechPower = mechPower;
}

void EMT::Ph3::SynchronGeneratorTrStab::initializeFromNodesAndTerminals(
    Real frequency) {

  // Initialize omega mech with nominal system frequency
  **mOmMech = mNomOmega;

  mInitElecPower = (mInitElecPower == Complex(0, 0))
                       ? -terminal(0)->singlePower()
                       : mInitElecPower;
  mInitMechPower = (mInitElecPower == Complex(0, 0)) ? mInitElecPower.real()
                                                     : mInitMechPower;

  // use complex interface quantities for initialization calculations
  MatrixComp intfVoltageComplex = MatrixComp::Zero(3, 1);
  MatrixComp intfCurrentComplex = MatrixComp::Zero(3, 1);
  // // derive complex threephase initialization from single phase initial values (only valid for balanced systems)
  // intfVoltageComplex(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  intfVoltageComplex(0, 0) = initialSingleVoltage(0);
  intfVoltageComplex(1, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_B;
  intfVoltageComplex(2, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_C;
  intfCurrentComplex(0, 0) =
      std::conj(-2. / 3. * mInitElecPower / intfVoltageComplex(0, 0));
  intfCurrentComplex(1, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_B;
  intfCurrentComplex(2, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_C;

  //save real interface quantities calculated from complex ones
  **mIntfVoltage = intfVoltageComplex.real();
  **mIntfCurrent = intfCurrentComplex.real();

  mImpedance = Complex(**mRs, mXpd);

  // Calculate initial emf behind reactance from power flow results
  **mEp = intfVoltageComplex(0, 0) - mImpedance * intfCurrentComplex(0, 0);

  // The absolute value of Ep is constant, only delta_p changes every step
  **mEp_abs = Math::abs(**mEp);
  // Delta_p is the angular position of mEp with respect to the synchronously rotating reference
  **mDelta_p = Math::phase(**mEp);

  // // Update active electrical power that is compared with the mechanical power
  **mElecActivePower = (3. / 2. * intfVoltageComplex(0, 0) *
                        std::conj(-intfCurrentComplex(0, 0)))
                           .real();
  // mElecActivePower = ( (mEp - (**mIntfVoltage)(0,0)) / mImpedance *  (**mIntfVoltage)(0,0) ).real();
  // For infinite power bus
  // mElecActivePower = (Math::abs(mEp) * Math::abs((**mIntfVoltage)(0,0)) / mXpd) * sin(mDelta_p);

  // Start in steady state so that electrical and mech. power are the same
  // because of the initial condition mOmMech = mNomOmega the damping factor is not considered at the initialisation
  **mMechPower = **mElecActivePower - mKd * (**mOmMech - mNomOmega);

  // Initialize node between X'd and Ep
  mVirtualNodes[0]->setInitialVoltage(PEAK1PH_TO_RMS3PH * **mEp);

  MatrixComp vref = MatrixComp::Zero(3, 1);
  vref = CPS::Math::singlePhaseVariableToThreePhase(**mEp);

  // Create sub voltage source for emf
  mSubVoltageSource =
      EMT::Ph3::VoltageSource::make(**mName + "_src", mLogLevel);
  mSubVoltageSource->setParameters(vref, frequency);
  mSubVoltageSource->connect({SimNode::GND, mVirtualNodes[0]});
  mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[1], 0);
  mSubVoltageSource->initialize(mFrequencies);
  mSubVoltageSource->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubVoltageSource,
                     MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // Create sub inductor as Xpd
  mSubInductor = EMT::Ph3::Inductor::make(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mLpd));
  mSubInductor->connect({mVirtualNodes[0], terminal(0)->node()});
  mSubInductor->initialize(mFrequencies);
  mSubInductor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialize according to powerflow ---"
                     "\nTerminal 0 voltage: {:e}<{:e}"
                     "\nVoltage behind reactance: {:e}<{:e}"
                     "\ninitial electrical power: {:e}+j{:e}"
                     "\nactive electrical power: {:e}"
                     "\nmechanical power: {:e}"
                     "\n--- End of powerflow initialization ---",
                     Math::abs((**mIntfVoltage)(0, 0)),
                     Math::phaseDeg((**mIntfVoltage)(0, 0)), Math::abs(**mEp),
                     Math::phaseDeg(**mEp), mInitElecPower.real(),
                     mInitElecPower.imag(), **mElecActivePower, **mMechPower);
}

void EMT::Ph3::SynchronGeneratorTrStab::step(Real time) {
  // #### Calculations on input of time step k ####
  // Transform interface quantities to synchronously rotating DQ reference frame
  Matrix intfVoltageDQ = parkTransformPowerInvariant(mThetaN, **mIntfVoltage);
  Matrix intfCurrentDQ = parkTransformPowerInvariant(mThetaN, **mIntfCurrent);
  // Update electrical power (minus sign to calculate generated power from consumed current)
  **mElecActivePower = -1. * (intfVoltageDQ(0, 0) * intfCurrentDQ(0, 0) +
                              intfVoltageDQ(1, 0) * intfCurrentDQ(1, 0));

  // The damping factor Kd is adjusted to obtain a damping ratio of 0.3
  // Real MaxElecActivePower= Math::abs(mEp) * Math::abs((**mIntfVoltage)(0,0)) / mXpd;
  // mKd=4*0.3*sqrt(mNomOmega*mInertia*MaxElecActivePower*0.5);
  mKd = 1 * mNomPower;

  // #### Calculate state for time step k+1 ####
  // semi-implicit Euler or symplectic Euler method for mechanical equations
  Real dOmMech =
      mNomOmega / (2. * **mInertia * mNomPower) *
      (**mMechPower - **mElecActivePower - mKd * (**mOmMech - mNomOmega));
  if (mBehaviour == Behaviour::MNASimulation)
    **mOmMech = **mOmMech + mTimeStep * dOmMech;
  Real dDelta_p = **mOmMech - mNomOmega;
  if (mBehaviour == Behaviour::MNASimulation)
    **mDelta_p = **mDelta_p + mTimeStep * dDelta_p;
  // Update emf - only phase changes
  if (mBehaviour == Behaviour::MNASimulation)
    **mEp = Complex(**mEp_abs * cos(**mDelta_p), **mEp_abs * sin(**mDelta_p));

  // Update nominal system angle
  mThetaN = mThetaN + mTimeStep * mNomOmega;

  // mStates << Math::abs(mEp), Math::phaseDeg(mEp), mElecActivePower, mMechPower,
  // 	mDelta_p, mOmMech, dOmMech, dDelta_p, (**mIntfVoltage)(0,0).real(), (**mIntfVoltage)(0,0).imag();
  // SPDLOG_LOGGER_DEBUG(mSLog, "\nStates, time {:f}: \n{:s}", time, Logger::matrixToString(mStates));
}

void EMT::Ph3::SynchronGeneratorTrStab::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  mTimeStep = timeStep;
  mMnaTasks.push_back(std::make_shared<AddBStep>(*this));
}

void EMT::Ph3::SynchronGeneratorTrStab::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfVoltage);
};

void EMT::Ph3::SynchronGeneratorTrStab::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
};

void EMT::Ph3::SynchronGeneratorTrStab::mnaParentPreStep(Real time,
                                                         Int timeStepCount) {
  step(time);
  //change magnitude of subvoltage source
  MatrixComp vref = MatrixComp::Zero(3, 1);
  vref = CPS::Math::singlePhaseVariableToThreePhase(PEAK1PH_TO_RMS3PH * **mEp);
  mSubVoltageSource->mVoltageRef->set(vref);
}

void EMT::Ph3::SynchronGeneratorTrStab::AddBStep::execute(Real time,
                                                          Int timeStepCount) {
  **mGenerator.mRightVector = **mGenerator.mSubInductor->mRightVector +
                              **mGenerator.mSubVoltageSource->mRightVector;
}

void EMT::Ph3::SynchronGeneratorTrStab::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::SynchronGeneratorTrStab::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  SPDLOG_LOGGER_DEBUG(mSLog, "Read voltage from {:d}", matrixNodeIndex(0));
  (**mIntfVoltage)(0, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  (**mIntfVoltage)(1, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
  (**mIntfVoltage)(2, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}

void EMT::Ph3::SynchronGeneratorTrStab::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  SPDLOG_LOGGER_DEBUG(mSLog, "Read current from {:d}", matrixNodeIndex(0));

  **mIntfCurrent = **mSubInductor->mIntfCurrent;
}
