/**
 * @file
 * @author  Junjie Zhang <junjie.zhang@rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_AvVoltageSourceInverterDQ.h>

using namespace CPS;

SP::Ph1::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel)
	:PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(4);
	setTerminalNumber(1);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Real>("p", &mP, Flags::read | Flags::write);
	addAttribute<Real>("q", &mQ, Flags::read | Flags::write);
	addAttribute<Real>("P_ref", &mPref, Flags::read | Flags::write);
	addAttribute<Real>("Q_ref", &mQref, Flags::read | Flags::write);
	addAttribute<Real>("theta", &mThetaPLL, Flags::read | Flags::write);
	addAttribute<Real>("phipll", &mPhiPLL, Flags::read | Flags::write);
	addAttribute<Real>("phid", &mPhi_d, Flags::read | Flags::write);
	addAttribute<Real>("phiq", &mPhi_q, Flags::read | Flags::write);
	addAttribute<Real>("gammad", &mGamma_d, Flags::read | Flags::write);
	addAttribute<Real>("gammaq", &mGamma_q, Flags::read | Flags::write);
	addAttribute<Complex>("vnom", &mVoltNom, Flags::read | Flags::write);
	addAttribute<Bool>("ctrl_on", &mCtrlOn, Flags::read | Flags::write);

	//additional loggers
	addAttribute<Matrix>("Vsdq", &mVsdq, Flags::read | Flags::write);
	addAttribute<Matrix>("Vcdq", &mVcdq, Flags::read | Flags::write);
	addAttribute<Matrix>("igdq", &mIgdq, Flags::read | Flags::write);
	addAttribute<Matrix>("ifdq", &mIfdq, Flags::read | Flags::write);
	addAttribute<Real>("omega", &mOmegaInst, Flags::read | Flags::write);
	addAttribute<Real>("freq", &mFreqInst, Flags::read | Flags::write);
	mSLog->info(
		"created {:s} instance"
		"\nName:{:s}",
		this->type(),
		this->name()
	);
}

void SP::Ph1::AvVoltageSourceInverterDQ::setParameters(Real sysOmega, Complex sysVoltNom, Real Pref, Real Qref, Real Kp_pll, Real Ki_pll,
	Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Lf, Real Cf,
	Real Rf, Real Rc) {
	mPref = Pref;
	mQref = Qref;

	mKpPLL = Kp_pll;
	mKiPLL = Ki_pll;

	mKiPowerCtrld = Ki_powerCtrl;
	mKiPowerCtrlq = Ki_powerCtrl;

	mKpPowerCtrld = Kp_powerCtrl;
	mKpPowerCtrlq = Kp_powerCtrl;

	mKiCurrCtrld = Ki_currCtrl;
	mKiCurrCtrlq = Ki_currCtrl;

	mKpCurrCtrld = Kp_currCtrl;
	mKpCurrCtrlq = Kp_currCtrl;

	mVoltNom = sysVoltNom;

	mOmegaN = sysOmega;
	mOmegaCutoff = sysOmega;

	mLf = Lf;
	mCf = Cf;
	mRf = Rf;
	mRc = Rc;

	mPFAvVoltageSourceInverter = SP::Ph1::Load::make(mName + "_pf", mName + "_pf", mLogLevel);
	mPFAvVoltageSourceInverter->setParameters(mPref, mQref, 0);
	mPFAvVoltageSourceInverter->modifyPowerFlowBusType(PowerflowBusType::PQ);

	parametersSet = true;
}


PowerComponent<Complex>::Ptr SP::Ph1::AvVoltageSourceInverterDQ::clone(String name) {
	auto copy = AvVoltageSourceInverterDQ::make(name, mLogLevel);
	copy->setParameters(mOmegaN, mVoltNom, mPref, mQref, mKpPLL, mKiPLL,
		mKpPowerCtrld, mKiPowerCtrld, mKpCurrCtrld, mKiCurrCtrld, mLf, mCf,
		mRf, mRc);
	return copy;
}

void SP::Ph1::AvVoltageSourceInverterDQ::addGenProfile(std::vector<Real>* genProfile) {
	mGenProfile = genProfile;
}

void SP::Ph1::AvVoltageSourceInverterDQ::addAggregatedGenProfile(std::vector<Real>* genProfile, Real customerNumber) {
	// TODO bind1st is deprecated
	//std::transform(genProfile->begin(), genProfile->end(), genProfile->begin(),
	//	std::bind1st(std::multiplies<Real>(), customerNumber));
	mGenProfile = genProfile;
}

void SP::Ph1::AvVoltageSourceInverterDQ::updateMonitoredValues(const Matrix& leftVector, Real time) {
	MatrixComp VcSP = MatrixComp::Zero(1, 1);
	MatrixComp IfSP = MatrixComp::Zero(1, 1);
	MatrixComp IgSP = MatrixComp::Zero(1, 1);

	// minus sign before currents due to the connection
	VcSP(0, 0) = Math::complexFromVectorElement(leftVector, mSubCapacitorF->simNode(0));
	IfSP(0, 0) = -1. * mSubResistorF->attribute<MatrixComp>("i_intf")->get()(0, 0);
	IgSP(0, 0) = -1. * mSubResistorC->attribute<MatrixComp>("i_intf")->get()(0, 0);

	Complex vcdq, ifdq, igdq;

	if (mBehaviour == Behaviour::Initialization) {
		mThetaSInit = mOmegaN * time;

		vcdq = rotatingFrame2to1(VcSP(0, 0), mThetaPLL, mOmegaN * time);
		ifdq = rotatingFrame2to1(IfSP(0, 0), mThetaPLL, mOmegaN * time);
		igdq = rotatingFrame2to1(IgSP(0, 0), mThetaPLL, mOmegaN * time);
	}
	else {
		vcdq = rotatingFrame2to1(VcSP(0, 0), mThetaPLL, mThetaSInit + mOmegaN * time);
		ifdq = rotatingFrame2to1(IfSP(0, 0), mThetaPLL, mThetaSInit + mOmegaN * time);
		igdq = rotatingFrame2to1(IgSP(0, 0), mThetaPLL, mThetaSInit + mOmegaN * time);
	}

	mVcdq(0, 0) = vcdq.real();
	mVcdq(1, 0) = vcdq.imag();

	mIfdq(0, 0) = ifdq.real();
	mIfdq(1, 0) = ifdq.imag();

	mIgdq(0, 0) = igdq.real();
	mIgdq(1, 0) = igdq.imag();

	mIntfVoltage = mSubCtrledVoltageSource->attribute<MatrixComp>("i_intf")->get();
	// update B matrices with new Igdq
	updateLinearizedModel();
}


void SP::Ph1::AvVoltageSourceInverterDQ::initializeModel(Real omega, Real timeStep,
	Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;
	mOmegaN = omega;
	mOmegaCutoff = omega;

	// initialize mInftCurrent
	// do it here to make sure resistor current is already initialized (initializeFromPowerFlow)
	mIntfCurrent = mSubResistorC->attribute<MatrixComp>("i_intf")->get();

	//initialize input vector
	MatrixComp IgSP = -mSubResistorC->attribute<MatrixComp>("i_intf")->get();
	mIgdq(0, 0) = IgSP(0, 0).real();
	mIgdq(1, 0) = IgSP(0, 0).imag();
	MatrixComp IfSP = -mSubResistorF->attribute<MatrixComp>("i_intf")->get();
	mIfdq(0, 0) = IfSP(0, 0).real();
	mIfdq(1, 0) = IfSP(0, 0).imag();
	mVcdq(0, 0) = mVirtualNodes[3]->initialSingleVoltage().real();
	mVcdq(1, 0) = mVirtualNodes[3]->initialSingleVoltage().imag();

	mU = Matrix::Zero(7, 1);
	mU <<
		mOmegaN, mPref, mQref, mVcdq(0, 0), mVcdq(1, 0), mIfdq(0, 0), mIfdq(1, 0);

	// create ABCD matrices
	mA = Matrix::Zero(8, 8);
	mB = Matrix::Zero(8, 7);
	mC = Matrix::Zero(2, 8);
	mD = Matrix::Zero(2, 7);

	mA <<
		0, mKiPLL, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, -mOmegaCutoff, 0, 0, 0, 0, 0,
		0, 0, 0, -mOmegaCutoff, 0, 0, 0, 0,
		0, 0, -1, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, -mKpPowerCtrld, 0, mKiPowerCtrld, 0, 0, 0,
		0, 0, 0, mKpPowerCtrlq, 0, mKiPowerCtrlq, 0, 0;

	mB <<
		1, 0, 0, 0, mKpPLL, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 3. / 2. * mOmegaCutoff * mIgdq(0, 0), 3. / 2. * mOmegaCutoff * mIgdq(1, 0), 0, 0,
		0, 0, 0, -3. / 2. * mOmegaCutoff * mIgdq(1, 0), 3. / 2. * mOmegaCutoff * mIgdq(0, 0), 0, 0,
		0, 1, 0, 0, 0, 0, 0,
		0, 0, -1, 0, 0, 0, 0,
		0, mKpPowerCtrld, 0, 0, 0, -1, 0,
		0, 0, -mKpPowerCtrlq, 0, 0, 0, -1;

	mC <<
		0, 0, -mKpPowerCtrld * mKpCurrCtrld, 0, mKpCurrCtrld * mKiPowerCtrld, 0, mKiCurrCtrld, 0,
		0, 0, 0, mKpPowerCtrlq * mKpCurrCtrlq, 0, mKpCurrCtrlq*mKiPowerCtrlq, 0, mKiCurrCtrlq;

	mD <<
		0, mKpCurrCtrld*mKpPowerCtrld, 0, 0, 0, -mKpCurrCtrld, 0,
		0, 0, -mKpCurrCtrlq * mKpPowerCtrlq, 0, 0, 0, -mKpCurrCtrlq;

	// initialize states
	mThetaPLL = 0;
	mPhiPLL = 0;
	mP = 0;
	mQ = 0;
	mPhi_d = 0;
	mPhi_q = 0;
	mGamma_d = 0;
	mGamma_q = 0;

	mStates = Matrix::Zero(8, 1);
	mStates <<
		mThetaPLL, mPhiPLL, mP, mQ, mPhi_d, mPhi_q, mGamma_d, mGamma_q;
	// initialize output
	mVsdq = mC * mStates + mD * mU;
}
void SP::Ph1::AvVoltageSourceInverterDQ::updatePowerGeneration() {
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
void SP::Ph1::AvVoltageSourceInverterDQ::step(Real time, Int timeStepCount) {
	Matrix newStates = Matrix::Zero(8, 1);
	Matrix newU = Matrix::Zero(7, 1);
	if (mBehaviour == Behaviour::Simulation && (mGenProfile || (!mLoadProfile.empty()))) {
		if(timeStepCount % mProfileUndateRate  == 0)
			updatePowerGeneration();
	}
	newU <<
		mOmegaN, mPref, mQref, mVcdq(0, 0), mVcdq(1, 0), mIfdq(0, 0), mIfdq(1, 0);

	newStates = Math::StateSpaceTrapezoidal(mStates, mA, mB, mTimeStep, newU, mU);

	if (mCtrlOn) {
		// update states
		mThetaPLL = newStates(0, 0);
		// update measurements ( for additional loggers)
		mOmegaInst = (time > 0) ? mThetaPLL / time : 0;
		mFreqInst = mOmegaInst / 2 / PI;
		mPhiPLL = newStates(1, 0);
		mP = newStates(2, 0);
		mQ = newStates(3, 0);
		mPhi_d = newStates(4, 0);
		mPhi_q = newStates(5, 0);
		mGamma_d = newStates(6, 0);
		mGamma_q = newStates(7, 0);

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

void SP::Ph1::AvVoltageSourceInverterDQ::updateLinearizedModel() {
	mB.coeffRef(2, 3) = 3. / 2. * mOmegaCutoff * mIgdq(0, 0);
	mB.coeffRef(2, 4) = 3. / 2. * mOmegaCutoff * mIgdq(1, 0);
	mB.coeffRef(3, 3) = -3. / 2. * mOmegaCutoff * mIgdq(1, 0);
	mB.coeffRef(3, 4) = 3. / 2. * mOmegaCutoff * mIgdq(0, 0);
}

Complex SP::Ph1::AvVoltageSourceInverterDQ::rotatingFrame2to1(Complex f2, Real theta1, Real theta2) {
	Real delta = theta2 - theta1;
	Real f1_real = f2.real() * cos(delta) - f2.imag() * sin(delta);
	Real f1_imag = f2.real() * sin(delta) + f2.imag() * cos(delta);
	return Complex(f1_real, f1_imag);
}

void SP::Ph1::AvVoltageSourceInverterDQ::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	// set initial voltages
	//mIntfVoltage(0, 0) = initialSingleVoltage(0);

	//mVirtualNodes[1]->setInitialVoltage(mIntfVoltage(0, 0));
	//mVirtualNodes[2]->setInitialVoltage(mIntfVoltage(0, 0));
	//mVirtualNodes[3]->setInitialVoltage(mIntfVoltage(0, 0));
		// set initial voltages
	mIntfVoltage(0, 0) = initialSingleVoltage(0);
	//if (mBehaviour == Behaviour::Initialization) {
	mIntfCurrent(0, 0) = -std::conj(Complex(mPref, mQref) / mIntfVoltage(0, 0));
	Complex vcInit = mIntfVoltage(0, 0) - mIntfCurrent(0, 0) * mRc;
	Complex icfInit = vcInit * Complex(0., 2. * PI * frequency * mCf);
	Complex vfInit = vcInit - (mIntfCurrent(0, 0) + icfInit) * Complex(0., 2. * PI * frequency * mLf);
	Complex vsInit = vfInit - (mIntfCurrent(0, 0) + icfInit) * mRf;
	mVirtualNodes[1]->setInitialVoltage(vsInit);
	mVirtualNodes[2]->setInitialVoltage(vfInit);
	mVirtualNodes[3]->setInitialVoltage(vcInit);


	// Create sub components
	mSubResistorF = SP::Ph1::Resistor::make(mName + "_resF", Logger::Level::off);
	mSubResistorC = SP::Ph1::Resistor::make(mName + "_resC", Logger::Level::off);
	mSubCapacitorF = SP::Ph1::Capacitor::make(mName + "_capF", Logger::Level::off);
	mSubInductorF = SP::Ph1::Inductor::make(mName + "_indF", Logger::Level::off);
	mSubCtrledVoltageSource = SP::Ph1::ControlledVoltageSource::make(mName + "_src", Logger::Level::off);

	//
	mSubResistorC->setParameters(mRc);
	mSubResistorF->setParameters(mRf);
	mSubInductorF->setParameters(mLf);
	mSubCapacitorF->setParameters(mCf);
	mSubCtrledVoltageSource->setParameters(mIntfVoltage);

	//
	mSubCtrledVoltageSource->connect({ Node::GND, mVirtualNodes[1] });
	mSubCtrledVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubResistorF->connect({ mVirtualNodes[1], mVirtualNodes[2] });
	mSubInductorF->connect({ mVirtualNodes[2], mVirtualNodes[3] });
	mSubCapacitorF->connect({ mVirtualNodes[3], Node::GND });
	mSubResistorC->connect({ mVirtualNodes[3], mTerminals[0]->node() });

	//
	mSubResistorF->initializeFromPowerflow(frequency);
	mSubInductorF->initializeFromPowerflow(frequency);
	mSubCapacitorF->initializeFromPowerflow(frequency);
	mSubResistorC->initializeFromPowerflow(frequency);
	//mSubCtrledVoltageSource->initializeFromPowerflow(frequency);
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
		mTerminals[0]->node()->name(), mTerminals[0]->node()->simNode());
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	if (mGenProfile)
		mCurrentPower = mGenProfile->begin();
	if(!mLoadProfile.empty())
		mCurrentLoad = mLoadProfile.begin();
	MNAInterface::List subComps({ mSubResistorF, mSubInductorF, mSubCapacitorF, mSubResistorC, mSubCtrledVoltageSource });
	mTimeStep = timeStep;
	initializeModel(omega, timeStep, leftVector);
	mSubResistorF->mnaInitialize(omega, timeStep, leftVector);
	mSubInductorF->mnaInitialize(omega, timeStep, leftVector);
	mSubCapacitorF->mnaInitialize(omega, timeStep, leftVector);
	mSubResistorC->mnaInitialize(omega, timeStep, leftVector);
	mSubCtrledVoltageSource->mnaInitialize(omega, timeStep, leftVector);

	mRightVectorStamps.push_back(&mSubCtrledVoltageSource->attribute<Matrix>("right_vector")->get());

	for (auto comp : subComps) {
		for (auto task : comp->mnaTasks()) {
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


void SP::Ph1::AvVoltageSourceInverterDQ::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubCtrledVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
	mSubResistorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubCapacitorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubResistorC->mnaApplySystemMatrixStamp(systemMatrix);
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;
}

MatrixComp SP::Ph1::AvVoltageSourceInverterDQ::dqToSP(Real time) {
	MatrixComp vsDqS(1, 1);;
	if (mBehaviour == Behaviour::Initialization) {
		vsDqS(0, 0) = rotatingFrame2to1(Complex(mVsdq(0, 0), mVsdq(1, 0)), mOmegaN * time, mThetaPLL);
	}
	else
	{
		vsDqS(0, 0) = rotatingFrame2to1(Complex(mVsdq(0, 0), mVsdq(1, 0)),
			mThetaSInit + mOmegaN * time, mThetaPLL);
	}
	return vsDqS;
}

void SP::Ph1::AvVoltageSourceInverterDQ::updateSetPoint(Real time){
	if(mQRefInput)
		mQref = mQRefInput->get();
}


void SP::Ph1::AvVoltageSourceInverterDQ::MnaPreStep::execute(Real time, Int timeStepCount) {
	// shift EMT voltage into SP
	MatrixComp vsDqOmegaS = mAvVoltageSourceInverterDQ.dqToSP(time);
	// update voltage of subVoltage source
	mAvVoltageSourceInverterDQ.mSubCtrledVoltageSource->setParameters(vsDqOmegaS);
}

void SP::Ph1::AvVoltageSourceInverterDQ::MnaPostStep::execute(Real time, Int timeStepCount) {
	mAvVoltageSourceInverterDQ.mnaUpdateCurrent(*mLeftVector);
	// update Vcabc, Igabc
	mAvVoltageSourceInverterDQ.updateMonitoredValues(*mLeftVector, time);
	mAvVoltageSourceInverterDQ.step(time, timeStepCount);
}

void SP::Ph1::AvVoltageSourceInverterDQ::CtrlStep::execute(Real time, Int timeStepCount){
	mAvVoltageSourceInverterDQ.updateSetPoint(time);
}

void SP::Ph1::AvVoltageSourceInverterDQ::AddBStep::execute(Real time, Int timeStepCount) {
	mAvVoltageSourceInverterDQ.mnaApplyRightSideVectorStamp(mAvVoltageSourceInverterDQ.mRightVector);
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaUpdateCurrent(const Matrix& leftvector) {
	mIntfCurrent = mSubResistorC->attribute<MatrixComp>("i_intf")->get();
	mSLog->debug("Current {:s}", Logger::phasorToString(mIntfCurrent(0, 0)));
}

void SP::Ph1::AvVoltageSourceInverterDQ::ctrlReceiver(Attribute<Real>::Ptr qrefInput){
	mQRefInput = qrefInput;
}

// power flow functions
void SP::Ph1::AvVoltageSourceInverterDQ::updatePQ(Real time){
	if(mQRefInput)
		mQref = mQRefInput->get();
	if(mPFAvVoltageSourceInverter)
		mPFAvVoltageSourceInverter->updatePQ(time);
	mPFAvVoltageSourceInverter = SP::Ph1::Load::make(mName + "_pf", mName + "_pf", mLogLevel);
	mPFAvVoltageSourceInverter->setParameters(mPref, mQref, 0);
	mPFAvVoltageSourceInverter->modifyPowerFlowBusType(PowerflowBusType::PQ);
}

void SP::Ph1::AvVoltageSourceInverterDQ::modifyPowerFlowBusType(PowerflowBusType powerflowBusType) {
	mPFAvVoltageSourceInverter->modifyPowerFlowBusType(powerflowBusType);
}

void SP::Ph1::AvVoltageSourceInverterDQ::PowerFlowStep::execute(Real time, Int timeStepCount){
	mAvVoltageSourceInverterDQ.updatePQ(time);
}

void SP::Ph1::AvVoltageSourceInverterDQ::pfBusInitialize(){
	mPFTasks.clear();
	mPFTasks.push_back(std::make_shared<PowerFlowStep>(*this));
}
