/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_SynchronGenerator4OrderIter.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::SynchronGenerator4OrderIter::SynchronGenerator4OrderIter
    (String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {

	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);
	mSimTime = 0.0;
	mStepNumber = 0;

	// model flags
	mVoltageForm = false;
	
	// model variables
	mIntfVoltage = Matrix::Zero(3,1);
	mIntfCurrent = Matrix::Zero(3,1);
	mVdq0 = Matrix::Zero(3,1);
	mIdq0 = Matrix::Zero(3,1);
	mEdq0_t = Matrix::Zero(3,1);
	mEdq0_t_prev = Matrix::Zero(3,1);
	mdEdq0_t = Matrix::Zero(3,1);
	mEdq0_t_prev = Matrix::Zero(3,1);

     // Register attributes
	addAttribute<Int>("NIterations", &mNumIterations2 , Flags::read);
	addAttribute<Real>("Etorque", &mElecTorque, Flags::read);
	addAttribute<Real>("delta", &mDelta_prev, Flags::read);
	addAttribute<Real>("Theta", &mThetaMech, Flags::read);
    addAttribute<Real>("w_r", &mOmMech, Flags::read);
	addAttribute<Matrix>("Edq0_t", &mEdq0_t, Flags::read);
	addAttribute<Matrix>("Vdq0", &mVdq0, Flags::read);
	addAttribute<Matrix>("Idq0", &mIdq0, Flags::read);

}

EMT::Ph3::SynchronGenerator4OrderIter::SynchronGenerator4OrderIter
	(String name, Logger::Level logLevel)
	: SynchronGenerator4OrderIter(name, name, logLevel) {
}

SimPowerComp<Real>::Ptr EMT::Ph3::SynchronGenerator4OrderIter::clone(String name) {
	auto copy = SynchronGenerator4OrderIter::make(name, mLogLevel);
	
	return copy;
}

void EMT::Ph3::SynchronGenerator4OrderIter::initializeFromNodesAndTerminals(Real frequency) {

	updateMatrixNodeIndices();

	// Initialize mechanical torque
	mMechTorque = mInitMechPower / mNomPower;
		
	// calculate steady state machine emf (i.e. voltage behind synchronous reactance)
	Complex Eq0 = mInitVoltage + Complex(0, mLq) * mInitCurrent;
	
	// Load angle
	mDelta = Math::phase(Eq0);
	
	// convert currrents to dq reference frame
	mIdq0(0,0) = Math::abs(mInitCurrent) * sin(mDelta - mInitCurrentAngle);
	mIdq0(1,0) = Math::abs(mInitCurrent) * cos(mDelta - mInitCurrentAngle);

	// convert voltages to dq reference frame
	mVdq0(0,0) = Math::abs(mInitVoltage) * sin(mDelta  - mInitVoltageAngle);
	mVdq0(1,0) = Math::abs(mInitVoltage) * cos(mDelta  - mInitVoltageAngle);

	// calculate Ef
	mEf = Math::abs(Eq0) + (mLd - mLq) * mIdq0(0,0);

	// initial electrical torque
	mElecTorque = mVdq0(0,0) * mIdq0(0,0) + mVdq0(1,0) * mIdq0(1,0);
	
	// Initialize omega mech with nominal system frequency
	mOmMech = mNomOmega / mBase_OmMech;
	
	// initialize theta and calculate transform matrix
	mThetaMech = mDelta - PI / 2.;
    
	mSLog->info(
		"\n--- Initialization  ---"
		"\nInitial Vd (per unit): {:f}"
		"\nInitial Vq (per unit): {:f}"
		"\nInitial Id (per unit): {:f}"
		"\nInitial Iq (per unit): {:f}"
		"\nInitial Ef (per unit): {:f}"
		"\nInitial mechanical torque (per unit): {:f}"
		"\nInitial electrical torque (per unit): {:f}"
		"\nInitial initial mechanical theta (per unit): {:f}"
        "\nInitial delta (per unit): {:f}"
		"\n--- Initialization finished ---",

		mVdq0(0,0),
		mVdq0(1,0),
		mIdq0(0,0),
		mIdq0(1,0),
		mEf,
		mMechTorque,
		mElecTorque,
		mThetaMech,
        mDelta
	);
	mSLog->flush();
}

void EMT::Ph3::SynchronGenerator4OrderIter::initialize() {
	mDelta = mDelta_prev;
	mdDelta = 0;
	mdDelta0 = 0;

	mOmMech_prev = mOmMech;
	mdOmMech = 0;
	mdOmMech0 = mdOmMech;

	mThetaMech_pred = mThetaMech;

	// initial voltage behind the transient reactance in the dq0 reference frame
	mEdq0_t(0,0) = mVdq0(0,0) - mIdq0(1,0) * mLq_t;
	mEdq0_t(1,0) = mVdq0(1,0) + mIdq0(0,0) * mLd_t;
	mEdq0_t(2,0) = 0.0;
	mEdq0_t_prev = mEdq0_t;

	// Initialize matrix of state representation
	mA = Matrix::Zero(3,3);
	mB = Matrix::Zero(3,3);
	mC = Matrix::Zero(3,1);
	calculateStateMatrix();

	// Check Results:
	mIntfCurrent = inverseParkTransform(mThetaMech, mIdq0);
	mIntfCurrent = mIntfCurrent*mBase_I;

	mIntfVoltage(0,0) =  mVdq0(0,0);
	mIntfVoltage(1,0) =  mVdq0(1,0);
	mIntfVoltage(2,0) =  0.0;
	mIntfVoltage = inverseParkTransform(mThetaMech, mIntfVoltage);
	mIntfVoltage = mIntfVoltage * mBase_V;

	//check initial derivatives (check initialization)
	if (mVoltageForm)
		mdEdq0_t = mA * mEdq0_t + mB * mVdq0 + mC;
	else
		mdEdq0_t = mA * mEdq0_t + mB * mIdq0 + mC;
	mdEdq0_t_prev = mdEdq0_t;

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nVerificacion of initial values:"
		"\nmIntfCurrent(0,0): {:f}"
		"\nmIntfCurrent(1,0): {:f}"
		"\nmIntfCurrent(2,0): {:f}"
		"\nmIntfVoltage(0,0): {:f}"
		"\nmIntfVoltage(1,0): {:f}"
		"\nmIntfVoltage(2,0): {:f}"
		"\ndEd_t(0): {:f}"
		"\ndEq_t(0): {:f}"
		"\nMax number of iterations: {:d}"
		"\nTolerance: {:f}"
		"\n--- Initialization finished ---",

		mEdq0_t(0,0),
		mEdq0_t(1,0),
		mIntfCurrent(0,0),
		mIntfCurrent(1,0),
		mIntfCurrent(2,0),
		mIntfVoltage(0,0),
		mIntfVoltage(1,0),
		mIntfVoltage(2,0),
		mdEdq0_t(0,0),
		mdEdq0_t(1,0),
		mMaxIterations,
		mMaxError
	);
	mSLog->flush();
}

void EMT::Ph3::SynchronGenerator4OrderIter::calculateStateMatrix() {
	if (mVoltageForm) {
		Real Td_t =  mTd0_t * (mLd_t / mLd);
		Real Tq_t =  mTq0_t * (mLq_t / mLq);
		mA << -1. / Tq_t   ,          0.0,		0.0,
	               0.0     ,      -1 / Td_t,	0.0,
				   0.0     ,          0.0,		0.0;
		mB << (1. / Tq_t) * (mLq-mLq_t) / mLq,   				0.0, 					0.0,
		  					0.0, 					(1. / Td_t) * (mLd-mLd_t) / mLd,	0.0,
							0.0, 								0.0,					0.0;
		mC <<               0.0,
		   		(1. / Td_t) * mEf * (mLd_t / mLd),
				   			0.0;
	}
	else {	// Currents form
		mA << -1./mTq0_t	,    	0.0,		0.0,		
		     	   0.0   	,    -1/mTd0_t,		0.0,
				   0.0		,		0.0,		0.0;
		mB <<             0.0                  , (1. / mTq0_t) * (mLq-mLq_t),		0.0,
		  		(-1. / mTd0_t) * (mLd-mLd_t)   ,            0.0             ,		0.0,
				  		  0.0				   ,			0.0				,		0.0;	
		mC <<          0.0,
		 		 (1./mTd0_t) * mEf,
				  	   0.0;
	}
}

void EMT::Ph3::SynchronGenerator4OrderIter::mnaInitialize(Real omega, 
		Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	// It is necessary?
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mTimeStep = timeStep;
	initialize();
	
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph3::SynchronGenerator4OrderIter::MnaPreStep::execute(Real time, Int timeStepCount) {
	mSynGen.stepInPerUnit(); //former system solve (trapezoidal)
	mSynGen.mnaApplyRightSideVectorStamp(mSynGen.mRightVector);
}

void EMT::Ph3::SynchronGenerator4OrderIter::stepInPerUnit() {
	mNumIterations = 0;
	mStepNumber = 0;
	std::cout << "Test" << std::endl;
	return;
}

void EMT::Ph3::SynchronGenerator4OrderIter::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(0,0), mIntfCurrent(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0,1), mIntfCurrent(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0,2), mIntfCurrent(2, 0));
	}
}

bool EMT::Ph3::SynchronGenerator4OrderIter::step() {
	/*
	if (Math::abs(mSimTime - mFaultTime) < 0.1 * mTimeStep)  {
		// stamp currents
		mnaApplyRightSideVectorStamp(mRightVector);

		return true;
	}
	if	(Math::abs(mSimTime - mFaultClearingTime) < 0.1 * mTimeStep) {
		
		//mIdq0 = mIdq_preFault;
		//std::cout << "mIdq0(0,0) = " << mIdq0(0,0) << std::endl;
		//std::cout << "mIdq0(1,0) = " << mIdq0(1,0) << std::endl;

		//mIntfCurrent = inverseParkTransform(mThetaMech, mIdq0);
		//mIntfCurrent = mIntfCurrent * mBase_I;

		// stamp currents
		mnaApplyRightSideVectorStamp(mRightVector);

		return true;
	} 
	*/
	std::cout << "Test1" << std::endl;
	mEdq0_t = mEdq0_t_prev + mdEdq0_t_prev;
	if (mStepNumber==0)
		// Predictor step (euler)
		mEdq0_t = mEdq0_t_prev + mdEdq0_t_prev;
	else if (mStepNumber==1)
		// Correction step (trapezoidal rule)
		mEdq0_t = mEdq0_t_prev + 0.5 * (mdEdq0_t_prev + mdEdq0_t);

	// armature currents for at t=k+1
	mIdq0(0,0) = (mEdq0_t(1,0) - mVdq0(1,0) ) / mLd_t;
	mIdq0(1,0) = (mVdq0(0,0) - mEdq0_t(0,0) ) / mLq_t;

	// calculate mechanical omega and at time k+1 (using trapezoidal rule)
	mElecTorque = mVdq0(0,0) * mIdq0(0,0) + mVdq0(1,0) * mIdq0(1,0);
	mdOmMech = mTimeStep / (2.* mH) * (mMechTorque - mElecTorque);
	mOmMech = mOmMech_prev + 0.5 * (mdOmMech0 + mdOmMech);

	// convert currents into the abc domain
	mThetaMech_pred = mThetaMech + mOmMech * mTimeStep * mBase_OmMech;
	mIntfCurrent = inverseParkTransform(mThetaMech_pred, mIdq0);
	mIntfCurrent = mIntfCurrent * mBase_I;

	// stamp currents
	mnaApplyRightSideVectorStamp(mRightVector);

	if (mStepNumber==1) {
		return false;
	} else {
		mStepNumber = 1;
		return true;
	}
}

void EMT::Ph3::SynchronGenerator4OrderIter::updateVoltage(const Matrix& leftVector) {
	mVdq0_prev = mVdq0;

	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
		mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
		mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	}

	// convert Vdq into abc domain
	mVdq0 = parkTransform(mThetaMech_pred, mIntfVoltage);
	mVdq0 = mVdq0 / mBase_V;

	/*
	if (Math::abs(mSimTime - mFaultTime) < 0.1 * mTimeStep) {
		mFaultTime = 10000;
		return;
	}
	
	if (Math::abs(mSimTime - mFaultClearingTime) < 0.1 * mTimeStep) {
		// CHECK FOR CORRECTNESS
		// re initialization
		mEdq_t(0,0) = mVdq0(0,0) - mXq_t * mIdq0(1,0);
		mEdq_t(1,0) = mVdq0(1,0) + mXd_t * mIdq0(0,0);
		mEdq_t(2,0) = 0.0;
		mEdq_t_prev = mEdq_t;
		if (mVoltageForm)
			mdEdq_t = mA * mEdq_t + mB * mVdq0 + mC;
		else
			mdEdq_t = mA * mEdq_t + mB * mIdq0 + mC;
		mdEdq_t = mTimeStep * mdEdq_t;
		mdEdq_t0 = mdEdq_t;
		mFaultClearingTime = 10000;
		return;
	} 
	*/

	// update torque 
	mElecTorque = mVdq0(0,0) * mIdq0(0,0) + mVdq0(1,0) * mIdq0(1,0);
	mdOmMech = mTimeStep / (2.* mH) * (mMechTorque - mElecTorque);
	mOmMech = mOmMech_prev + 0.5 * (mdOmMech0 + mdOmMech);
	mThetaMech_pred = mThetaMech + mOmMech * mTimeStep * mBase_OmMech;

	// calculate derivatives of voltage behind the transient reactance
	if (mVoltageForm)
		mdEdq0_t = mA * mEdq0_t + mB * mVdq0 + mC;
	else
		mdEdq0_t = mA * mEdq0_t + mB * mIdq0 + mC;
	mdEdq0_t = mTimeStep * mdEdq0_t;
}

bool EMT::Ph3::SynchronGenerator4OrderIter::checkVoltageDifference() {
	Matrix voltageDifference = mVdq0 - mVdq0_prev;

	if (Math::abs(voltageDifference(0,0)) > mMaxError || Math::abs(voltageDifference(1,0)) > mMaxError) {
		if (mNumIterations == mMaxIterations) {
			return false;
		} else {
			mNumIterations = mNumIterations + 1;
			return true;
		}
	} else {
		return false;
	}
}

void EMT::Ph3::SynchronGenerator4OrderIter::mnaPostStep(const Matrix& leftVector) {
	// correct voltage behind the reactance
	mEdq0_t(0,0) = mVdq0(0,0) - mLq_t * mIdq0(1,0);
	mEdq0_t(1,0) = mVdq0(1,0) + mLd_t * mIdq0(0,0);
	mEdq0_t(2,0) = 0.0;

	// calculate derivatives of voltage behind the transient reactance
	if (mVoltageForm)
		mdEdq0_t = mA * mEdq0_t + mB * mVdq0 + mC;
	else
		mdEdq0_t = mA * mEdq0_t + mB * mIdq0 + mC;
	mdEdq0_t = mTimeStep * mdEdq0_t;
	
	// update delta
	mdDelta = mTimeStep * (mOmMech - 1.) * mBase_OmMech;
	mDelta = mDelta_prev + 0.5 * (mdDelta0 + mdDelta);

	//
	mEdq0_t_prev = mEdq0_t;
	mOmMech_prev = mOmMech;
	mDelta_prev = mDelta;
	
	mdEdq0_t_prev = mdEdq0_t;
	mdOmMech0 = mdOmMech;
	mdDelta0 = mdDelta;
	mNumIterations2 = mNumIterations;

	//
	mSimTime = mSimTime + mTimeStep;
	mThetaMech = mThetaMech + mTimeStep * mOmMech * mBase_OmMech;

	if (Math::abs(mSimTime - mFaultTime) < 0.1 * mTimeStep) {
		mIdq0_preFault = mIdq0;
	}

}

void EMT::Ph3::SynchronGenerator4OrderIter::MnaPostStep::execute(Real time, Int timeStepCount) {
	mSynGen.mnaPostStep(*mLeftVector);
}

Matrix EMT::Ph3::SynchronGenerator4OrderIter::parkTransform(Real theta, const Matrix& abcVector) {
	Matrix dq0Vector(3, 1);
	Matrix abcToDq0(3, 3);

	// Park transform according to Kundur
	abcToDq0 <<
		 2./3.*cos(theta),	2./3.*cos(theta - 2.*PI/3.),  2./3.*cos(theta + 2.*PI/3.),
		-2./3.*sin(theta), -2./3.*sin(theta - 2.*PI/3.), -2./3.*sin(theta + 2.*PI/3.),
		 1./3., 			1./3., 						  1./3.;

	dq0Vector = abcToDq0 * abcVector;

	return dq0Vector;
}

Matrix EMT::Ph3::SynchronGenerator4OrderIter::inverseParkTransform(Real theta, const Matrix& dq0Vector) {
	Matrix abcVector(3, 1);
	Matrix dq0ToAbc(3, 3);

	// Park transform according to Kundur
	dq0ToAbc <<
		cos(theta), 		   -sin(theta), 		   1.,
		cos(theta - 2.*PI/3.), -sin(theta - 2.*PI/3.), 1.,
		cos(theta + 2.*PI/3.), -sin(theta + 2.*PI/3.), 1.;

	abcVector = dq0ToAbc * dq0Vector;

	return abcVector;
}
