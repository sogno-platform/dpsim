/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_AvVoltageSourceInverterDQ.h>

using namespace CPS;


DP::Ph1::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel, Bool withTrafo) : 
	Base::AvVoltageSourceInverterDQ(uid, name, logLevel), 
	SimPowerComp<Complex>(uid, name, logLevel), 
	TopologicalPowerComp(uid, name, logLevel) {
	if (withTrafo) {
		setVirtualNodeNumber(5);
		mConnectionTransformer = DP::Ph1::Transformer::make(mName + "_trans", Logger::Level::debug);
		mSubComponents.push_back(mConnectionTransformer);
	} else {
		setVirtualNodeNumber(4);
	}
	mWithConnectionTransformer = withTrafo;	
	setTerminalNumber(1);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	// additional input variables
	addAttribute<Matrix>("Vcdq", &mVcdq, Flags::read | Flags::write);
	addAttribute<Matrix>("Ircdq", &mIrcdq, Flags::read | Flags::write);

	// additional output variables
	addAttribute<Matrix>("Vsdq", &mVsdq, Flags::read | Flags::write);

	// additional variables for logging
	addAttribute<Real>("omega", &mOmegaInst, Flags::read | Flags::write);
	addAttribute<Real>("freq", &mFreqInst, Flags::read | Flags::write);
	addAttribute<Bool>("ctrl_on", &mCtrlOn, Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr DP::Ph1::AvVoltageSourceInverterDQ::clone(String name) {
	auto copy = DP::Ph1::AvVoltageSourceInverterDQ::make(name, mLogLevel);
	copy->setParameters(mOmegaN, mVnom, mPref, mQref);
	return copy;
}

void DP::Ph1::AvVoltageSourceInverterDQ::addGenProfile(std::vector<Real>* genProfile) {
	mGenProfile = genProfile;
}

void DP::Ph1::AvVoltageSourceInverterDQ::addAggregatedGenProfile(std::vector<Real>* genProfile, Real customerNumber) {
	std::transform(genProfile->begin(), genProfile->end(), genProfile->begin(),
					std::bind1st(std::multiplies<Real>(), customerNumber));
	mGenProfile = genProfile;
}

void DP::Ph1::AvVoltageSourceInverterDQ::ctrlReceiver(Attribute<Real>::Ptr qrefInput){
	mQRefInput = qrefInput;
}

void DP::Ph1::AvVoltageSourceInverterDQ::initialize(Matrix frequencies) {
	mFrequencies = frequencies;
	mNumFreqs = static_cast<UInt>(mFrequencies.size());

	mIntfVoltage = MatrixComp::Zero(1, mNumFreqs);
	mIntfCurrent = MatrixComp::Zero(1, mNumFreqs);
}

void DP::Ph1::AvVoltageSourceInverterDQ::updateMonitoredValues(const Matrix& leftVector, Real time) {
	Complex vcdq, ircdq;

	vcdq = rotatingFrame2to1(Math::complexFromVectorElement(leftVector, mVirtualNodes[4]->matrixNodeIndex()), mThetaPLL, mOmegaN * time);
	ircdq = rotatingFrame2to1(-1. * mSubResistorC->attribute<MatrixComp>("i_intf")->get()(0, 0), mThetaPLL, mOmegaN * time);

	mVcdq(0, 0) = vcdq.real();
	mVcdq(1, 0) = vcdq.imag();

	mIrcdq(0, 0) = ircdq.real();
	mIrcdq(1, 0) = ircdq.imag();

	mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, mTerminals[0]->node()->matrixNodeIndex());

	updateBMatrixStateSpaceModel();
}


void DP::Ph1::AvVoltageSourceInverterDQ::initializeModel(Real omega, Real timeStep,
	Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;
	mOmegaN = omega;
	mOmegaCutoff = omega;

	// initialize current and voltage inputs to state space model
	// done here to ensure quantites are already initialized by initializeFromPowerFlow
	MatrixComp Irc = - mSubResistorC->attribute<MatrixComp>("i_intf")->get();
	mIrcdq(0, 0) = Irc(0, 0).real();
	mIrcdq(1, 0) = Irc(0, 0).imag();
	mVcdq(0, 0) = mVirtualNodes[4]->initialSingleVoltage().real();
	mVcdq(1, 0) = mVirtualNodes[4]->initialSingleVoltage().imag();
	mU = Matrix::Zero(7, 1);
	
	// update B matrix due to its dependence on Irc
	updateBMatrixStateSpaceModel();

	// initialization of input
	mU << mOmegaN, mPref, mQref, mVcdq(0, 0), mVcdq(1, 0), mIrcdq(0, 0), mIrcdq(1, 0);
	mSLog->info("Initialization of input: \n" + Logger::matrixToString(mU));

	// initialization of states
	mThetaPLL = mThetaPLLInit;
	mPhiPLL = mPhiPLLInit;
	mP = mPInit;
	mQ = mQInit;
	mPhi_d = mPhi_dInit;
	mPhi_q = mPhi_qInit;
	mGamma_d = mGamma_dInit;
	mGamma_q = mGamma_qInit;	
	mStates << mThetaPLL, mPhiPLL, mP, mQ, mPhi_d, mPhi_q, mGamma_d, mGamma_q;
	mSLog->info("Initialization of states: \n" + Logger::matrixToString(mStates));

	// initialization of output
	mVsdq = mC * mStates + mD * mU;
	mSLog->info("Initialization of output: \n" + Logger::matrixToString(mVsdq));
}

void DP::Ph1::AvVoltageSourceInverterDQ::updatePowerGeneration() {
	if(mIsLoad){
		if (mCurrentLoad != mLoadProfile.end()) {
			mPref = (*mCurrentLoad).p * -1;
			// Q_load is not updated
			mQref = (*mCurrentLoad).q * -1;

			++mCurrentLoad;
		}
		return;
	}

	if (mCurrentPower != mGenProfile->end()) {
		mPref = *mCurrentPower;
		++mCurrentPower;
	}
}

void DP::Ph1::AvVoltageSourceInverterDQ::step(Real time, Int timeStepCount) {
	Matrix newStates = Matrix::Zero(8, 1);
	Matrix newU = Matrix::Zero(7, 1);

	if (mBehaviour == Behaviour::Simulation && (mGenProfile || (!mLoadProfile.empty()))) {
		if(timeStepCount % mProfileUndateRate  == 0)
			updatePowerGeneration();
	}

	newU << mOmegaN, mPref, mQref, mVcdq(0, 0), mVcdq(1, 0), mIrcdq(0, 0), mIrcdq(1, 0);
	newStates = Math::StateSpaceTrapezoidal(mStates, mA, mB, mTimeStep, newU, mU);

	if (mCtrlOn) {
		// update states
		mThetaPLL = newStates(0, 0);
		mPhiPLL = newStates(1, 0);
		mP = newStates(2, 0);
		mQ = newStates(3, 0);
		mPhi_d = newStates(4, 0);
		mPhi_q = newStates(5, 0);
		mGamma_d = newStates(6, 0);
		mGamma_q = newStates(7, 0);

		// update measurements ( for additional loggers)
		mOmegaInst = (newStates(0, 0) - mStates(0,0))/mTimeStep;
		mFreqInst = mOmegaInst / 2 / PI;

		mStates = newStates;
		mU = newU;

		// new output
		mVsdq = mC * mStates + mD * mU;
	}
	else {
		mThetaPLL = newStates(0, 0);
		// update measurements ( for additional loggers)
		mOmegaInst = (time > 0) ? mThetaPLL / time : 0;
		mFreqInst = mOmegaInst / 2 / PI;
		mPhiPLL = newStates(1, 0);
		mP = newStates(2, 0);
		mQ = newStates(3, 0);
		mStates(0, 0) = newStates(0, 0);
		mStates(1, 0) = newStates(1, 0);
		mU = newU;

		mVsdq(0, 0) = mVirtualNodes[1]->initialSingleVoltage().real();
		mVsdq(1, 0) = mVirtualNodes[1]->initialSingleVoltage().imag();
	}
}

void DP::Ph1::AvVoltageSourceInverterDQ::updateBMatrixStateSpaceModel() {
	mB.coeffRef(2, 3) = mOmegaCutoff * mIrcdq(0, 0);
	mB.coeffRef(2, 4) = mOmegaCutoff * mIrcdq(1, 0);
	mB.coeffRef(3, 3) = -mOmegaCutoff * mIrcdq(1, 0);
	mB.coeffRef(3, 4) = mOmegaCutoff * mIrcdq(0, 0);
}

Complex DP::Ph1::AvVoltageSourceInverterDQ::rotatingFrame2to1(Complex f2, Real theta1, Real theta2) {
	Real delta = theta2 - theta1;
	Real f1_real = f2.real() * cos(delta) - f2.imag() * sin(delta);
	Real f1_imag = f2.real() * sin(delta) + f2.imag() * cos(delta);
	return Complex(f1_real, f1_imag);
}

void DP::Ph1::AvVoltageSourceInverterDQ::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	// set initial interface quantities
	mIntfVoltage(0, 0) = initialSingleVoltage(0);
	mIntfCurrent(0, 0) = - std::conj(Complex(mPref, mQref) / mIntfVoltage(0,0));
	
	Complex filterInterfaceInitialVoltage;
	Complex filterInterfaceInitialCurrent;

	if (mWithConnectionTransformer) {		
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter)
		// TODO: check possibility of more accurate solution as current only approximated
		filterInterfaceInitialVoltage = (mIntfVoltage(0, 0) - Complex(mTransformerResistance, mTransformerInductance*mOmegaN)*mIntfCurrent(0, 0)) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
		filterInterfaceInitialCurrent = mIntfCurrent(0, 0) * Complex(mTransformerRatioAbs, mTransformerRatioPhase);

		// connect and init transformer
		mVirtualNodes[4]->setInitialVoltage(filterInterfaceInitialVoltage);		
		mConnectionTransformer->connect({ mTerminals[0]->node(), mVirtualNodes[4] });
		mConnectionTransformer->setParameters(mTransformerRatioAbs, mTransformerRatioPhase, mTransformerResistance, mTransformerInductance);
		mConnectionTransformer->initialize(mFrequencies);
		mConnectionTransformer->initializeFromPowerflow(frequency);
	} else {
		// if no transformer used, filter interface equal to inverter interface
		filterInterfaceInitialVoltage = mIntfVoltage(0, 0);
		filterInterfaceInitialCurrent = mIntfCurrent(0, 0);
	}

	// derive initialization quantities of filter
	Complex vcInit = filterInterfaceInitialVoltage - filterInterfaceInitialCurrent * mRc;
	Complex icfInit = vcInit * Complex(0., 2. * PI * frequency * mCf);
	Complex vfInit = vcInit - (filterInterfaceInitialCurrent - icfInit) * Complex(0., 2. * PI * frequency * mLf);
	Complex vsInit = vfInit - (filterInterfaceInitialCurrent - icfInit) * Complex(mRf, 0);
	mVirtualNodes[1]->setInitialVoltage(vsInit);
	mVirtualNodes[2]->setInitialVoltage(vfInit);
	mVirtualNodes[3]->setInitialVoltage(vcInit);

	// Create sub components
	mSubResistorF = DP::Ph1::Resistor::make(mName + "_resF", mLogLevel);
	mSubResistorC = DP::Ph1::Resistor::make(mName + "_resC", mLogLevel);
	mSubCapacitorF = DP::Ph1::Capacitor::make(mName + "_capF", mLogLevel);
	mSubInductorF = DP::Ph1::Inductor::make(mName + "_indF", mLogLevel);
	mSubCtrledVoltageSource = DP::Ph1::ControlledVoltageSource::make(mName + "_src", mLogLevel);

	// set filter parameters
	mSubResistorC->setParameters(mRc);
	mSubResistorF->setParameters(mRf);
	mSubInductorF->setParameters(mLf);
	mSubCapacitorF->setParameters(mCf);
	mSubCtrledVoltageSource->setParameters(mIntfVoltage);

	// connect subcomponents
	mSubCtrledVoltageSource->connect({ SimNode::GND, mVirtualNodes[1] });
	mSubCtrledVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubResistorF->connect({ mVirtualNodes[1], mVirtualNodes[2] });
	mSubInductorF->connect({ mVirtualNodes[2], mVirtualNodes[3] });
	mSubCapacitorF->connect({ mVirtualNodes[3], SimNode::GND });
	if (mWithConnectionTransformer)
		mSubResistorC->connect({ mVirtualNodes[3],  mVirtualNodes[4]});
	else
		mSubResistorC->connect({ mVirtualNodes[3],  mTerminals[0]->node()});

	// initialize subcomponents
	mSubCtrledVoltageSource->initialize(mFrequencies);
	mSubResistorF->initialize(mFrequencies);
	mSubInductorF->initialize(mFrequencies);
	mSubCapacitorF->initialize(mFrequencies);
	mSubResistorC->initialize(mFrequencies);

	//mSubCtrledVoltageSource->initializeFromPowerflow(frequency);
	mSubResistorF->initializeFromPowerflow(frequency);
	mSubInductorF->initializeFromPowerflow(frequency);
	mSubCapacitorF->initializeFromPowerflow(frequency);
	mSubResistorC->initializeFromPowerflow(frequency);
	
	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex());
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mTimeStep = timeStep;

	// set powers from profiles
	if (mGenProfile)
		mCurrentPower = mGenProfile->begin();
	if(!mLoadProfile.empty())
		mCurrentLoad = mLoadProfile.begin();

	MNAInterface::List subComps({ mSubResistorF, mSubInductorF, mSubCapacitorF, mSubResistorC, mSubCtrledVoltageSource });

	mSubResistorF->mnaInitialize(omega, timeStep, leftVector);
	mSubInductorF->mnaInitialize(omega, timeStep, leftVector);
	mSubCapacitorF->mnaInitialize(omega, timeStep, leftVector);
	mSubResistorC->mnaInitialize(omega, timeStep, leftVector);
	mSubCtrledVoltageSource->mnaInitialize(omega, timeStep, leftVector);
	initializeModel(omega, timeStep, leftVector);

	mRightVectorStamps.push_back(&mSubCapacitorF->attribute<Matrix>("right_vector")->get());
	mRightVectorStamps.push_back(&mSubInductorF->attribute<Matrix>("right_vector")->get());
	mRightVectorStamps.push_back(&mSubCtrledVoltageSource->attribute<Matrix>("right_vector")->get());

	// add tasks
	for (auto comp : subComps) {
		for (auto task : comp->mnaTasks())
			mMnaTasks.push_back(task);
	}
	if (mWithConnectionTransformer) {
		mConnectionTransformer->mnaInitialize(omega, timeStep, leftVector);
		mRightVectorStamps.push_back(&mConnectionTransformer->attribute<Matrix>("right_vector")->get());
		for (auto task : mConnectionTransformer->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<AddBStep>(*this));
	if(mCoveeCtrled)
		mMnaTasks.push_back(std::make_shared<CtrlStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}


void DP::Ph1::AvVoltageSourceInverterDQ::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubCtrledVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
	mSubResistorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubCapacitorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubResistorC->mnaApplySystemMatrixStamp(systemMatrix);
	if (mWithConnectionTransformer)
		mConnectionTransformer->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;
}

void DP::Ph1::AvVoltageSourceInverterDQ::updateSetPoint(Real time){
	if(mQRefInput)
		mQref = mQRefInput->get();
}

void DP::Ph1::AvVoltageSourceInverterDQ::MnaPreStep::execute(Real time, Int timeStepCount) {
	if (mAvVoltageSourceInverterDQ.mCtrlOn) {
		MatrixComp vsDqOmegaS = MatrixComp::Zero(1,1);
		vsDqOmegaS(0,0) = mAvVoltageSourceInverterDQ.rotatingFrame2to1(Complex(mAvVoltageSourceInverterDQ.mVsdq(0, 0), mAvVoltageSourceInverterDQ.mVsdq(1, 0)), mAvVoltageSourceInverterDQ.mOmegaN * time, mAvVoltageSourceInverterDQ.mThetaPLL);
		mAvVoltageSourceInverterDQ.mSubCtrledVoltageSource->setParameters(vsDqOmegaS);
	}
	else
		mAvVoltageSourceInverterDQ.mSubCtrledVoltageSource->setParameters(mAvVoltageSourceInverterDQ.mVsdq);
}

void DP::Ph1::AvVoltageSourceInverterDQ::MnaPostStep::execute(Real time, Int timeStepCount) {
	mAvVoltageSourceInverterDQ.mnaUpdateCurrent(*mLeftVector);
	mAvVoltageSourceInverterDQ.updateMonitoredValues(*mLeftVector, time);
	mAvVoltageSourceInverterDQ.step(time, timeStepCount);
}

void DP::Ph1::AvVoltageSourceInverterDQ::CtrlStep::execute(Real time, Int timeStepCount){
	mAvVoltageSourceInverterDQ.updateSetPoint(time);
}


void DP::Ph1::AvVoltageSourceInverterDQ::AddBStep::execute(Real time, Int timeStepCount) {
	mAvVoltageSourceInverterDQ.mnaApplyRightSideVectorStamp(mAvVoltageSourceInverterDQ.mRightVector);
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaUpdateCurrent(const Matrix& leftvector) {
	if (mWithConnectionTransformer)
		mIntfCurrent = mConnectionTransformer->attribute<MatrixComp>("i_intf")->get();
	else
		mIntfCurrent = mSubResistorC->attribute<MatrixComp>("i_intf")->get();
}

