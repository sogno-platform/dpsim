/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SVC.h>

using namespace CPS;

DP::Ph1::SVC::SVC(String uid, String name, Logger::Level logLevel)
	: MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
	mVpcc(mAttributes->create<Real>("Vpcc", 0)),
	mVmeasPrev(mAttributes->create<Real>("Vmeas", 0)) {
	setTerminalNumber(1);
	setVirtualNodeNumber(2);
    **mIntfVoltage = MatrixComp::Zero(1,1);
	**mIntfCurrent = MatrixComp::Zero(1,1);

	mDeltaV = mAttributes->create<Real>("DeltaV", 0);
	mBPrev = mAttributes->create<Real>("B");
	mViolationCounter = mAttributes->create<Real>("ViolationCounter", 0);
}

Bool DP::Ph1::SVC::ValueChanged() {
	return mValueChange;
}

void DP::Ph1::SVC::initializeFromNodesAndTerminals(Real frequency) {

	// initial state is both switches are open
	Real omega = 2. * PI * frequency;
	// init L and C with small/high values (both have high impedance)
	Real LInit = 1e6 / omega;
	Real CInit = 1e-6 / omega;
	mLPrev = LInit;
	mCPrev = CInit;

	// impedances of both branches
	Complex LImpedance = { mSwitchROpen, omega * LInit };
	Complex CImpedance = { mSwitchROpen, -1/(omega * CInit) };
	Complex impedance = LImpedance * CImpedance / (LImpedance + CImpedance);

	(**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
	(**mIntfCurrent)(0,0)  = (**mIntfVoltage)(0,0) / impedance;

	**mBPrev = 0;
	mPrevVoltage = (**mIntfVoltage)(0, 0).real();
	**mVmeasPrev = mPrevVoltage;

	if (mMechMode) {
		SPDLOG_LOGGER_DEBUG(mSLog, "Using Mechanical Model");
	}

	SPDLOG_LOGGER_INFO(mSLog,
		"\n --- Parameters ---"
		"\n Controller: T = {} K = {}"
		"\n Reference Voltage  {} [kV]"
		"\n Qmax = {} [var] -> BN = {} [S]"
		"\n Bmax = {} Bmin = {} [p.u.]"
		"\n Initial B: {}",
		mTr, mKr, mRefVolt, mQN, mBN, mBMax, mBMin, **mBPrev);

	// set voltages at virtual nodes
	Complex VLSwitch = (**mIntfVoltage)(0, 0) - LImpedance * (**mIntfCurrent)(0, 0);
	mVirtualNodes[0]->setInitialVoltage(VLSwitch);
	Complex VCSwitch = (**mIntfVoltage)(0, 0) - CImpedance * (**mIntfCurrent)(0, 0);
	mVirtualNodes[1]->setInitialVoltage(VCSwitch);

	// create elements
	// Inductor with Switch
	mSubInductor = std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
	mSubInductor->setParameters(LInit);
	mSubInductor->connect({ SimNode::GND, mVirtualNodes[0] });
	mSubInductor->initialize(mFrequencies);
	mSubInductor->initializeFromNodesAndTerminals(frequency);

	mSubInductorSwitch = std::make_shared<DP::Ph1::Switch>(**mName + "_Lswitch", mLogLevel);
	mSubInductorSwitch->setParameters(mSwitchROpen, mSwitchRClosed, false);
	mSubInductorSwitch->connect({ mVirtualNodes[0], mTerminals[0]->node() });
	mSubInductorSwitch->initialize(mFrequencies);
	mSubInductorSwitch->initializeFromNodesAndTerminals(frequency);

	// Capacitor with Switch
	mSubCapacitor = std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap", mLogLevel);
	mSubCapacitor->setParameters(CInit);
	mSubCapacitor->connect({ SimNode::GND, mVirtualNodes[1] });
	mSubCapacitor->initialize(mFrequencies);
	mSubCapacitor->initializeFromNodesAndTerminals(frequency);

	mSubCapacitorSwitch = std::make_shared<DP::Ph1::Switch>(**mName + "_Cswitch", mLogLevel);
	mSubCapacitorSwitch->setParameters(mSwitchROpen, mSwitchRClosed, false);
	mSubCapacitorSwitch->connect({ mVirtualNodes[1], mTerminals[0]->node() });
	mSubCapacitorSwitch->initialize(mFrequencies);
	mSubCapacitorSwitch->initializeFromNodesAndTerminals(frequency);

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nImpedance: {}"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		impedance,
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)));
}

// #### MNA functions ####

void DP::Ph1::SVC::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
		updateMatrixNodeIndices();

	SPDLOG_LOGGER_INFO(mSLog,
		"\nTerminal 0 connected to {:s} = sim node {:d}",
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex());

    mSubInductor->mnaInitialize(omega, timeStep, leftVector);
    mRightVectorStamps.push_back(&mSubInductor->attributeTyped<Matrix>("right_vector")->get());

    mSubInductorSwitch->mnaInitialize(omega, timeStep, leftVector);
    mRighteVctorStamps.push_back(&mSubInductorSwitch->attributeTyped<Matrix>("right_vector")->get());

    mSubCapacitor->mnaInitialize(omega, timeStep, leftVector);
    mRightVectorStamps.push_back(&mSubCapacitor->attributeTyped<Matrix>("right_vector")->get());

    mSubCapacitorSwitch->mnaInitialize(omega, timeStep, leftVector);
    mRightVectorStamps.push_back(&mSubCapacitorSwitch->attributeTyped<Matrix>("right_vector")->get());
}

void DP::Ph1::SVC::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubCapacitor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubCapacitorSwitch->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductorSwitch->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::SVC::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
	mSubCapacitor->mnaApplyRightSideVectorStamp(rightVector);
	mSubCapacitorSwitch->mnaApplyRightSideVectorStamp(rightVector);
	mSubInductorSwitch->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::SVC::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
    mSubInductor->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
    mSubInductorSwitch->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
    mSubCapacitor->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
    mSubCapacitorSwitch->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);

	// add pre-step dependencies of component itself
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::SVC::mnaCompPreStep(Real time, Int timeStepCount) {
    mSubInductor->mnaPreStep(time, timeStepCount);
    mSubInductorSwitch->mnaPreStep(time, timeStepCount);
    mSubCapacitor->mnaPreStep(time, timeStepCount);
    mSubCapacitorSwitch->mnaPreStep(time, timeStepCount);

    mnaCompApplyRightSideVectorStamp(**mRightVector);

	if (time > 0.1 && !mDisconnect) {
		if (mMechMode) {
			mechanicalModelUpdateSusceptance(time);
		}
		else {
		    updateSusceptance();
		}
		checkProtection(time);
	}
}

void DP::Ph1::SVC::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
    mSubInductor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
    mSubInductorSwitch->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
    mSubCapacitor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
    mSubCapacitorSwitch->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);

	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::SVC::mnaCompPostStep(Real time, Int timeStepCount) {
	mSubInductor->mnaPostStep(time, timeStepCount, leftVector);
    mSubInductorSwitch->mnaPostStep(time, timeStepCount, leftVector);
    mSubCapacitor->mnaPostStep(time, timeStepCount, leftVector);
    mSubCapacitorSwitch->mnaPostStep(time, timeStepCount, leftVector);

	mnaCompUpdateVoltage(**mLeftVector);
	mnaCompUpdateCurrent(**mLeftVector);

	mDeltaT = time - mPrevTimeStep;
	mPrevTimeStep = time;
	mValueChange = false;
}

void DP::Ph1::SVC::mnaCompUpdateVoltage(const Matrix& leftVector) {
	**mVpcc = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, 0).real();
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::SVC::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = 0;
	(**mIntfCurrent)(0, 0) += mSubInductor->intfCurrent()(0, 0);
	(**mIntfCurrent)(0, 0) += mSubCapacitor->intfCurrent()(0, 0);
}

void DP::Ph1::SVC::checkProtection(Real time) {
	// check states for violation of protection values
	// get inverse protection curve value (time delay value)

	Real Vpu = **mVmeasPrev / mNomVolt;
	if (Vpu > 1.4) {
		mProtCount1 = mProtCount1 + mDeltaT;
		if (mProtCount1 > 0.1) {
			mDisconnect = true;
		}
	}
	else {
		mProtCount1 = 0;
	}
	if (Vpu > 1.25)	{
		mProtCount2 = mProtCount2 + mDeltaT;
		if (mProtCount2 > 1) {
			mDisconnect = true;
		}
	}
	else {
		mProtCount2 = 0;
	}
	if (Vpu > 1.15)	{
		mProtCount3 = mProtCount3 + mDeltaT;
		if (mProtCount3 > 5) {
			mDisconnect = true;
		}
	}
	else {
		mProtCount3 = 0;
	}

	if (mDisconnect) {
		SPDLOG_LOGGER_DEBUG(mSLog, "Disconnect SVC because of overvoltage at {}", time);
		mSubCapacitorSwitch->open();
		mSubInductorSwitch->open();
		mValueChange = true;
	}
}

void DP::Ph1::SVC::updateSusceptance() {
	// calculate new B value
	// summarize some constants
	Real Fac1 = mDeltaT / (2 * mTr);
	Real Fac2 = mDeltaT * mKr / (2 * mTr);

	Complex vintf = (**mIntfVoltage)(0, 0);
	Real V = Math::abs((**mIntfVoltage)(0, 0).real());

	// Pt1 with trapez rule for voltage measurement
	Real Fac3 = mDeltaT / (2 * mTm);
	Real Vmeas = (1 / (1 + Fac3)) * (V + mPrevVoltage - **mVmeasPrev);

	**mDeltaV = (Vmeas - mRefVolt) / mNomVolt;
	Real deltaVPrev = (**mVmeasPrev - mRefVolt) / mNomVolt;

	// calc new B with trapezoidal rule
	//Real B = (1/(1+Fac1)) * (Fac2 * (mDeltaV + deltaVPrev) + (1-Fac1) * mBPrev);
	Real B = (1 / (1 + Fac1)) * (Fac2 * (**mDeltaV + deltaVPrev) + (1 - Fac1) * **mBPrev);
	//SPDLOG_LOGGER_INFO(mSLog, "New B value: percent={}, absolute={}", 100 * B, B * mBN);

	// check bounds
	if (B > mBMax) {
		B =  mBMax;
		//SPDLOG_LOGGER_DEBUG(mSLog, "New B value exceeds Bmax");
	}
	else if(B < mBMin) {
		B = mBMin;
		//SPDLOG_LOGGER_DEBUG(mSLog, "New B value exceeds Bmin");
	}

	// set new B if it has a new value and difference is big enough
	if (B != **mBPrev) {
	//if (B != mBPrev && mBSetCounter > 0.001){
		//mValueChange = true;
		//mBSetCounter = 0;
		Real omega = 2 * M_PI*mFrequencies(0, 0);

		if (B > 0) {
			// model inductive behaviour (decrease voltage)
			Real inductance = 1 / (omega * B * mBN);
			//check if change in reactance is sufficient to trigger a change
			if (Math::abs(1 - inductance / mLPrev) > 0.01) {
				mInductiveMode = true;
				mSubInductor->updateInductance(inductance, mDeltaT);
				//SPDLOG_LOGGER_DEBUG(mSLog, "Inductive Mode: New Inductance: L = {} [H]", inductance);
				mLPrev = inductance;

				mValueChange = true;
				mBSetCounter = 0;
			}
		}
		else {
			// model capacitive behaviour (increase voltage)
			Real capacitance = B * mBN / (-omega);
			//check if change in reactance is sufficient to trigger a change
			if (Math::abs(1 - capacitance / mCPrev) > 0.01) {
				mInductiveMode = false;
				mSubCapacitor->updateCapacitance(capacitance, mDeltaT);
				//SPDLOG_LOGGER_DEBUG(mSLog, "Capacitive Mode: New Capacitance: C = {} [F]", capacitance);
				mCPrev = capacitance;

				mValueChange = true;
				mBSetCounter = 0;
			}
		}

		// update inductance model
		setSwitchState();
	}
	else {
		mBSetCounter = mBSetCounter + mDeltaT;
	}

	// save values
	**mBPrev = B;
	mPrevVoltage = V;
	**mVmeasPrev = Vmeas;
}

// model SVC with a mechanical component and discrete
void DP::Ph1::SVC::mechanicalModelUpdateSusceptance(Real time) {
	// current voltage
	Real V = Math::abs((**mIntfVoltage)(0, 0).real());
	Real omega = 2 * M_PI*mFrequencies(0, 0);

	// Pt1 with trapez rule for voltage measurement
	Real Fac3 = mDeltaT / (2 * mTm);
	Real Vmeas = (1 / (1 + Fac3)) * (V + mPrevVoltage - **mVmeasPrev);

	// V diff in pu
	Real deltaV = (mRefVolt - Vmeas) / mRefVolt;

	if (Math::abs(deltaV) > mDeadband) {
		if (**mViolationCounter > mMechSwitchDelay) {
			// change suszeptance one step
			if (deltaV > 0 && (mTapPos > mMinPos)) {
				// undervoltage

				mTapPos = mTapPos - 1;
				mTapPos = (mTapPos < mMinPos) ? mMinPos : mTapPos;
				**mViolationCounter = 0;
				SPDLOG_LOGGER_DEBUG(mSLog, "Time: {}"
					"\nDecreasing Tap. Reason: Undervoltage"
					"\nNew Tap Position: {}", time, mTapPos);
			}
			else if(deltaV < 0 && (mTapPos < mMaxPos)) {
				// overvoltage
				mTapPos = mTapPos + 1;
				mTapPos = (mTapPos > mMaxPos) ? mMaxPos : mTapPos;
				**mViolationCounter = 0;
				SPDLOG_LOGGER_DEBUG(mSLog, "Time: {}"
					"\nIncreasing Tap. Reason: Overvoltag"
					"\nNew Tap Position: {}", time, mTapPos);
			}

			if (**mViolationCounter == 0) {
				// new value for suszeptance
				if (mTapPos > 0) {
					// inductor is active
					mInductiveMode = true;
					Real inductance = 1 / ((mTapPos / mMaxPos) * mBN * omega);
					SPDLOG_LOGGER_DEBUG(mSLog, "New inductance: {}", inductance);
					mSubInductor->updateInductance(inductance, mDeltaT);
					mValueChange = true;
					setSwitchState();
				}
				else if (mTapPos < 0) {
					// capacitor is active
					mInductiveMode = false;
					Real capacitance = ((mTapPos / mMinPos) * mBN) / omega;
					SPDLOG_LOGGER_DEBUG(mSLog, "New capacitance: {}", capacitance);
					mSubCapacitor->updateCapacitance(capacitance, mDeltaT);
					mValueChange = true;
					setSwitchState();

				}
				else if (mTapPos = 0) {
					// open both
					SPDLOG_LOGGER_DEBUG(mSLog, "Time: {}"
						"Tap Position: 0. Open both elements", time);
					mSubInductorSwitch->open();
					mSubCapacitorSwitch->open();
				}
			}
		}
		else {
			// increase counter
			**mViolationCounter = **mViolationCounter + mDeltaT;
		}
	}
	else {
		// reset counter
		**mViolationCounter = 0;
	}

	// save states
	mPrevVoltage = V;
	**mVmeasPrev = Vmeas;
}

void DP::Ph1::SVC::setSwitchState() {
	// set switches according to current mode of svc
	if (mInductiveMode) {
		if (!mSubInductorSwitch->mnaIsClosed()) {
			SPDLOG_LOGGER_DEBUG(mSLog, "Inductive Mode: Closed Inductor Switch");
			mSubInductorSwitch->close();
		}
		if (mSubCapacitorSwitch->mnaIsClosed()) {
			mSubCapacitorSwitch->open();
			SPDLOG_LOGGER_DEBUG(mSLog, "Inductive Mode: Opened Capacitor Switch");
		}
	}
	else {
		if (mSubInductorSwitch->mnaIsClosed()) {
			mSubInductorSwitch->open();
			SPDLOG_LOGGER_DEBUG(mSLog, "Capacitive Mode: Openend Inductor Switch");
		}
		if (!mSubCapacitorSwitch->mnaIsClosed()) {
			mSubCapacitorSwitch->close();
			SPDLOG_LOGGER_DEBUG(mSLog, "Capacitive Mode: Closed Capcitor Switch");
		}
	}
}
