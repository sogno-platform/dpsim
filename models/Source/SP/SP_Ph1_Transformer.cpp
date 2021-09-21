/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_Transformer.h>

using namespace CPS;

// #### General ####
SP::Ph1::Transformer::Transformer(String uid, String name, Logger::Level logLevel, Bool withResistiveLosses)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	if (withResistiveLosses)
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);
	setTerminalNumber(2);

	addAttribute<Real>("nominal_voltage_end1", &mNominalVoltageEnd1, Flags::read | Flags::write);
	addAttribute<Real>("nominal_voltage_end2", &mNominalVoltageEnd2, Flags::read | Flags::write);
	addAttribute<Real>("base_voltage", &mBaseVoltage, Flags::read | Flags::write);
	addAttribute<Real>("S", &mRatedPower, Flags::write | Flags::read);
	addAttribute<Complex>("ratio", &mRatio, Flags::write | Flags::read);
	addAttribute<Real>("R", &mResistance, Flags::write | Flags::read);
	addAttribute<Real>("L", &mInductance, Flags::write | Flags::read);
	addAttribute<Bool>("nodal_injection_stored", &mStoreNodalPowerInjection, Flags::read | Flags::write);
	addAttribute<Real>("p_inj", &mActivePowerInjection, Flags::read | Flags::write);
	addAttribute<Real>("q_inj", &mReactivePowerInjection, Flags::read | Flags::write);

	addAttribute<Complex>("current", &mCurrent(0), Flags::read | Flags::write);
	addAttribute<Complex>("current_1", &mCurrent(1), Flags::read | Flags::write);
	addAttribute<Real>("p_branch", &mActivePowerBranch(0), Flags::read | Flags::write);
	addAttribute<Real>("q_branch", &mReactivePowerBranch(0), Flags::read | Flags::write);
	addAttribute<Real>("p_branch_1", &mActivePowerBranch(1), Flags::read | Flags::write);
	addAttribute<Real>("q_branch_1", &mReactivePowerBranch(1), Flags::read | Flags::write);
}


void SP::Ph1::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	// Note: to be consistent impedance values must be referred to high voltage side (and base voltage set to higher voltage)
	Base::Ph1::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);

	mSLog->info("Nominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]", mNominalVoltageEnd1, mNominalVoltageEnd2);
	mSLog->info("Resistance={} [Ohm] Inductance={} [H] (referred to primary side)", mResistance, mInductance);
    mSLog->info("Tap Ratio={} [/] Phase Shift={} [deg]", std::abs(mRatio), std::arg(mRatio));

	mRatioAbs = std::abs(mRatio);
	mRatioPhase = std::arg(mRatio);
	mConductance = 1 / mResistance;

	mParametersSet = true;
}

void SP::Ph1::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	mRatedPower = ratedPower;
	mSLog->info("Rated Power ={} [W]", mRatedPower);

	SP::Ph1::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);
}


SimPowerComp<Complex>::Ptr SP::Ph1::Transformer::clone(String name) {
	auto copy = Transformer::make(name, mLogLevel);
	copy->setParameters(mNominalVoltageEnd1, mNominalVoltageEnd2, mRatedPower,
		std::abs(mRatio), std::arg(mRatio), mResistance, mInductance);
	return copy;
}

void SP::Ph1::Transformer::initializeFromNodesAndTerminals(Real frequency) {
	mNominalOmega = 2. * PI * frequency;
	mReactance = mNominalOmega * mInductance;
	mSLog->info("Reactance={} [Ohm] (referred to primary side)", mReactance);

	// Component parameters are referred to high voltage side.
	// Switch terminals if transformer is connected the other way around.
	if (Math::abs(mRatio) < 1.) {
		mRatio = 1. / mRatio;
		std::shared_ptr<SimTerminal<Complex>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
	}

	// Set initial voltage of virtual node in between
	mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(1) * mRatio);

	// Static calculations from load flow data
	Complex impedance = { mResistance, mReactance };
	mIntfVoltage(0, 0) = mVirtualNodes[0]->initialSingleVoltage() - initialSingleVoltage(0);
	mIntfCurrent(0, 0) = mIntfVoltage(0, 0) / impedance;

	// Create series sub components
	mSubInductor = std::make_shared<SP::Ph1::Inductor>(mUID + "_ind", mName + "_ind", Logger::Level::off);
	mSubInductor->setParameters(mInductance);

	if (mNumVirtualNodes == 3) {
		mVirtualNodes[2]->setInitialVoltage(initialSingleVoltage(0));
		mSubResistor = std::make_shared<SP::Ph1::Resistor>(mUID + "_res", mName + "_res", Logger::Level::off);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ node(0), mVirtualNodes[2] });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
		mSubInductor->connect({ mVirtualNodes[2], mVirtualNodes[0] });
	} else {
		mSubInductor->connect({ node(0), mVirtualNodes[0] });
	}
	mSubInductor->initialize(mFrequencies);
	mSubInductor->initializeFromNodesAndTerminals(frequency);

	// Create parallel sub components
	// A snubber conductance is added on the low voltage side (resistance scaled with lower voltage side)
	mSnubberResistance = mNominalVoltageEnd1 > mNominalVoltageEnd2 ? std::abs(mNominalVoltageEnd2)*1e6 : std::abs(mNominalVoltageEnd1)*1e6;
	mSubSnubResistor = std::make_shared<SP::Ph1::Resistor>(mUID + "_snub_res", mName + "_snub_res", Logger::Level::off);
	mSubSnubResistor->setParameters(mSnubberResistance);
	mSubSnubResistor->connect({ node(1), SP::SimNode::GND });
	mSubSnubResistor->initialize(mFrequencies);
	mSubSnubResistor->initializeFromNodesAndTerminals(frequency);
	mSLog->info("Snubber Resistance={} [Ohm] (connected to LV side)", mSnubberResistance);

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\nVirtual Node 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}


// #### Powerflow section ####

void SP::Ph1::Transformer::setBaseVoltage(Real baseVoltage) {
	// Note: to be consistent set base voltage to higher voltage (and impedance values must be referred to high voltage side)
	// TODO: use attribute setter for setting base voltage
    mBaseVoltage = baseVoltage;
}

void SP::Ph1::Transformer::calculatePerUnitParameters(Real baseApparentPower, Real baseOmega) {
	mSLog->info("#### Calculate Per Unit Parameters for {}", mName);
    mBaseApparentPower = baseApparentPower;
	mBaseOmega = baseOmega;
    mSLog->info("Base Power={} [VA]  Base Omega={} [1/s]", baseApparentPower, baseOmega);

	mBaseImpedance = mBaseVoltage * mBaseVoltage / mBaseApparentPower;
	mBaseAdmittance = 1.0 / mBaseImpedance;
	mBaseCurrent = baseApparentPower / (mBaseVoltage*sqrt(3)); // I_base=(S_threephase/3)/(V_line_to_line/sqrt(3))
	mSLog->info("Base Voltage={} [V]  Base Impedance={} [Ohm]", mBaseVoltage, mBaseImpedance);

	mResistancePerUnit = mResistance / mBaseImpedance;
	mReactancePerUnit = mReactance / mBaseImpedance;
    mSLog->info("Resistance={} [pu]  Reactance={} [pu]", mResistancePerUnit, mReactancePerUnit);

	mBaseInductance = mBaseImpedance / mBaseOmega;
	mInductancePerUnit = mInductance / mBaseInductance;
	mMagnetizing = mMagnetizingReactance / mBaseImpedance;
	// omega per unit=1, hence 1.0*mInductancePerUnit.
	mLeakagePerUnit = Complex(mResistancePerUnit,1.*mInductancePerUnit);
	mMagnetizingPerUnit = mMagnetizing / mBaseImpedance;

    mRatioAbsPerUnit = mRatioAbs / mNominalVoltageEnd1 * mNominalVoltageEnd2;
    mSLog->info("Tap Ratio={} [pu]", mRatioAbsPerUnit);

	// set snubber resistance
	if (mBehaviour == Behaviour::Initialization) {
		Real snubberResistance = 1e3;
		mSubSnubResistor = std::make_shared<SP::Ph1::Resistor>(mUID + "_snub_res", mName + "_snub_res", mLogLevel);
		mSubSnubResistor->setParameters(snubberResistance);
		mSubSnubResistor->connect({ node(1), SP::SimNode::GND });
		mSubSnubResistor->initializeFromNodesAndTerminals(mBaseOmega);
	}
	if (mSubSnubResistor) {
		mSubSnubResistor->setBaseVoltage(mBaseVoltage);
		mSubSnubResistor->calculatePerUnitParameters(baseApparentPower);
	}

	mSLog->info("Leakage Impedance Per Unit={} [Ohm] ", mLeakagePerUnit);
}

void SP::Ph1::Transformer::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) {
	// calculate matrix stamp
	mY_element = MatrixComp(2, 2);
	Complex y = Complex(1, 0) / mLeakagePerUnit;
	Complex ys = Complex(1, 0) / mMagnetizingPerUnit;
	mY_element(0, 0) = (y + ys);
	mY_element(0, 1) = -y*mRatioAbsPerUnit;
	mY_element(1, 0) = -y*mRatioAbsPerUnit;
	mY_element(1, 1) = (y + ys)*std::pow(mRatioAbsPerUnit, 2);

	//check for inf or nan
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			if (std::isinf(mY_element.coeff(i, j).real()) || std::isinf(mY_element.coeff(i, j).imag())) {
				std::cout << mY_element << std::endl;
				std::cout << "Zm:" << mMagnetizing << std::endl;
				std::cout << "Zl:" << mLeakage << std::endl;
				std::cout << "tap:" << mRatioAbsPerUnit << std::endl;
				std::stringstream ss;
				ss << "Transformer>>" << this->name() << ": infinite or nan values in the element Y at: " << i << "," << j;
				throw std::invalid_argument(ss.str());
			}

	//set the circuit matrix values
	Y.coeffRef(this->matrixNodeIndex(0), this->matrixNodeIndex(0)) += mY_element.coeff(0, 0);
	Y.coeffRef(this->matrixNodeIndex(0), this->matrixNodeIndex(1)) += mY_element.coeff(0, 1);
	Y.coeffRef(this->matrixNodeIndex(1), this->matrixNodeIndex(1)) += mY_element.coeff(1, 1);
	Y.coeffRef(this->matrixNodeIndex(1), this->matrixNodeIndex(0)) += mY_element.coeff(1, 0);

	mSLog->info("#### Y matrix stamping: {}", mY_element);
}


void SP::Ph1::Transformer::updateBranchFlow(VectorComp& current, VectorComp& powerflow) {
	mCurrent = current * mBaseCurrent;
	mActivePowerBranch = powerflow.real()*mBaseApparentPower;
	mReactivePowerBranch = powerflow.imag()*mBaseApparentPower;
}


void SP::Ph1::Transformer::storeNodalInjection(Complex powerInjection) {
	mActivePowerInjection = std::real(powerInjection)*mBaseApparentPower;
	mReactivePowerInjection = std::imag(powerInjection)*mBaseApparentPower;
	mStoreNodalPowerInjection = true;
}

MatrixComp SP::Ph1::Transformer::Y_element() {
	return mY_element;
}

// #### MNA Section ####

void SP::Ph1::Transformer::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	auto subComponents = MNAInterface::List({ mSubInductor, mSubSnubResistor });

	if (mSubResistor)
		subComponents.push_back(mSubResistor);

	for (auto comp : subComponents) {
		comp->mnaInitialize(omega, timeStep, leftVector);

		for (auto task : comp->mnaTasks())
			mMnaTasks.push_back(task);

	}
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mSLog->info(
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\nTerminal 1 connected to {:s} = sim node {:d}",
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
		mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}


void SP::Ph1::Transformer::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Ideal transformer equations
	if (terminalNotGrounded(0)) {
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[1]->matrixNodeIndex(), Complex(-1.0, 0));
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), Complex(1.0, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(1), mVirtualNodes[1]->matrixNodeIndex(), mRatio);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), matrixNodeIndex(1), -mRatio);
	}

	// Add inductive part to system matrix
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSnubResistor->mnaApplySystemMatrixStamp(systemMatrix);

	if (mNumVirtualNodes == 3) {
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	}

	if (terminalNotGrounded(0)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[1]->matrixNodeIndex());
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex());
	}
	if (terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(mRatio),
			matrixNodeIndex(1), mVirtualNodes[1]->matrixNodeIndex());
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-mRatio),
			mVirtualNodes[1]->matrixNodeIndex(), matrixNodeIndex(1));
	}
}


void SP::Ph1::Transformer::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	if (mSubResistor)
		this->mSubResistor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	this->mSubInductor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	this->mSubSnubResistor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("i_intf"));
}

void SP::Ph1::Transformer::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	if (mSubResistor)
		this->mSubResistor->mnaPostStep(time, timeStepCount, leftVector);
	this->mSubInductor->mnaPostStep(time, timeStepCount, leftVector);
	this->mSubSnubResistor->mnaPostStep(time, timeStepCount, leftVector);
	// post-step of component itself
	this->mnaUpdateVoltage(*leftVector);
	this->mnaUpdateCurrent(*leftVector);
}

void SP::Ph1::Transformer::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = mSubInductor->intfCurrent()(0, 0);
	mSLog->debug("Current {:s}", Logger::phasorToString(mIntfCurrent(0, 0)));

}

void SP::Ph1::Transformer::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage(0, 0) = 0;
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex());
	mSLog->debug("Voltage {:s}", Logger::phasorToString(mIntfVoltage(0, 0)));
}
