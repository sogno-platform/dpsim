/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_PiLine.h>

using namespace CPS;

SP::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {

	mSLog->info("Create {} {}", this->type(), name);
	mSLog->flush();

	setVirtualNodeNumber(1);
    setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Real>("base_Voltage", &mBaseVoltage, Flags::read | Flags::write);
	addAttribute<Real>("R_series", &mSeriesRes, Flags::read | Flags::write);
	addAttribute<Real>("L_series", &mSeriesInd, Flags::read | Flags::write);
	addAttribute<Real>("C_parallel", &mParallelCap, Flags::read | Flags::write);
	addAttribute<Real>("G_parallel", &mParallelCond, Flags::read | Flags::write);
	addAttribute<Complex>("current", &mCurrent(0), Flags::read | Flags::write);
	addAttribute<Complex>("current_1", &mCurrent(1), Flags::read | Flags::write);
	addAttribute<Real>("p_branch", &mActivePowerBranch(0), Flags::read | Flags::write);
	addAttribute<Real>("q_branch", &mReactivePowerBranch(0), Flags::read | Flags::write);
	addAttribute<Real>("p_branch_1", &mActivePowerBranch(1), Flags::read | Flags::write);
	addAttribute<Real>("q_branch_1", &mReactivePowerBranch(1), Flags::read | Flags::write);
	addAttribute<Bool>("nodal_injection_stored", &mStoreNodalPowerInjection, Flags::read | Flags::write);
	addAttribute<Real>("p_inj", &mActivePowerInjection, Flags::read | Flags::write);
	addAttribute<Real>("q_inj", &mReactivePowerInjection, Flags::read | Flags::write);
}

void SP::Ph1::PiLine::setParameters(Real resistance, Real inductance, Real capacitance, Real conductance) {

	mSeriesRes = resistance;
	mSeriesInd = inductance;
	mSLog->info("Resistance={} [Ohm] Inductance={} [H]", mSeriesRes, mSeriesInd);

    if(capacitance > 0){
        mParallelCap = capacitance;
    }else{
        mParallelCap = 1e-12;
        mSLog->warn("Zero value for Capacitance, setting default value of C={} [F]", mParallelCap);
    }
    if(conductance > 0){
        mParallelCond = conductance;
    }else{
        if (mBehaviour == Behaviour::Initialization)
			mParallelCond = (conductance >= 0) ? conductance : 1e-6; // init mode for initFromPowerFlow of mna system components
		else
			mParallelCond = (conductance > 0) ? conductance : 1e-6;
        mSLog->warn("Zero value for Conductance, setting default value of G={} [S]", mParallelCond);
    }
    mSLog->info("Capacitance={} [F] Conductance={} [S]", mParallelCap, mParallelCond);
	mSLog->flush();
	mParametersSet = true;

}

SimPowerComp<Complex>::Ptr SP::Ph1::PiLine::clone(String name) {
	auto copy = PiLine::make(name, mLogLevel);
	copy->setParameters(mSeriesRes, mSeriesInd, mParallelCap, mParallelCond);
	return copy;
}

// #### Powerflow section ####
void SP::Ph1::PiLine::setBaseVoltage(Real baseVoltage) {
    mBaseVoltage = baseVoltage;
}

void SP::Ph1::PiLine::calculatePerUnitParameters(Real baseApparentPower, Real baseOmega) {
    mSLog->info("#### Calculate Per Unit Parameters for {}", mName);
	mBaseApparentPower = baseApparentPower;
	mBaseOmega = baseOmega;
    mSLog->info("Base Power={} [VA]  Base Omega={} [1/s]", baseApparentPower, baseOmega);

	mBaseImpedance = (mBaseVoltage * mBaseVoltage) / mBaseApparentPower;
	mBaseAdmittance = 1.0 / mBaseImpedance;
	mBaseInductance = mBaseImpedance / mBaseOmega;
	mBaseCapacitance = 1.0 / mBaseOmega / mBaseImpedance;
	mBaseCurrent = baseApparentPower / (mBaseVoltage*sqrt(3)); // I_base=(S_threephase/3)/(V_line_to_line/sqrt(3))
	mSLog->info("Base Voltage={} [V]  Base Impedance={} [Ohm]", mBaseVoltage, mBaseImpedance);

    mSeriesResPerUnit = mSeriesRes / mBaseImpedance;
	mSeriesIndPerUnit = mSeriesInd / mBaseInductance;
	mParallelCapPerUnit = mParallelCap / mBaseCapacitance;
	mParallelCondPerUnit = mParallelCond / mBaseAdmittance;

	mSLog->info("Resistance={} [pu] Reactance={} [pu]", mSeriesResPerUnit, 1. * mSeriesIndPerUnit);
	mSLog->info("Susceptance={} [pu] Conductance={} [pu]", 1. * mParallelCapPerUnit, mParallelCondPerUnit);
	mSLog->flush();
}

void SP::Ph1::PiLine::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) {
	int bus1 = this->matrixNodeIndex(0);
	int bus2 = this->matrixNodeIndex(1);

	//create the element admittance matrix
	Complex y = Complex(1, 0) / Complex(mSeriesResPerUnit, 1. * mSeriesIndPerUnit);
	Complex ys = Complex(mParallelCondPerUnit, 1. * mParallelCapPerUnit) / Complex(2, 0);

	//Fill the internal matrix
	mY_element = MatrixComp(2, 2);
	mY_element(0, 0) = y + ys;
	mY_element(0, 1) = -y;
	mY_element(1, 0) = -y;
	mY_element(1, 1) = y + ys;

	//check for inf or nan
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			if (std::isinf(mY_element.coeff(i, j).real()) || std::isinf(mY_element.coeff(i, j).imag())) {
				std::cout << mY_element << std::endl;
				std::stringstream ss;
				ss << "Line>>" << this->name() << ": infinite or nan values in the element Y at: " << i << "," << j;
				throw std::invalid_argument(ss.str());
				std::cout << "Line>>" << this->name() << ": infinite or nan values in the element Y at: " << i << "," << j << std::endl;
			}

	//set the circuit matrix values
	Y.coeffRef(bus1, bus1) += mY_element.coeff(0, 0);
	Y.coeffRef(bus1, bus2) += mY_element.coeff(0, 1);
	Y.coeffRef(bus2, bus2) += mY_element.coeff(1, 1);
	Y.coeffRef(bus2, bus1) += mY_element.coeff(1, 0);

	mSLog->info("#### PF Y matrix stamping #### ");
	mSLog->info("{}", mY_element);
	mSLog->flush();
}

void SP::Ph1::PiLine::updateBranchFlow(VectorComp& current, VectorComp& powerflow) {
	mCurrent = current * mBaseCurrent;
	mActivePowerBranch = powerflow.real()*mBaseApparentPower;
	mReactivePowerBranch = powerflow.imag()*mBaseApparentPower;
}

void SP::Ph1::PiLine::storeNodalInjection(Complex powerInjection) {
	mActivePowerInjection = std::real(powerInjection)*mBaseApparentPower;
	mReactivePowerInjection = std::imag(powerInjection)*mBaseApparentPower;
	mStoreNodalPowerInjection = true;
}

MatrixComp SP::Ph1::PiLine::Y_element() {
	return mY_element;
}

void SP::Ph1::PiLine::initializeFromNodesAndTerminals(Real frequency) {

	// By default there is always a small conductance to ground to
	// avoid problems with floating nodes.
	mParallelCond = (mParallelCond >= 0) ? mParallelCond : 1e-6;

	// Static calculation
	Real omega = 2. * PI * frequency;
	Complex impedance = { mSeriesRes, omega * mSeriesInd };
	mIntfVoltage(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent(0, 0) = mIntfVoltage(0, 0) / impedance;

	// Initialization of virtual node
	mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(0) + mIntfCurrent(0, 0) * mSeriesRes);

	// Create series sub components
	mSubSeriesResistor = std::make_shared<SP::Ph1::Resistor>(mName + "_res", mLogLevel);
	mSubSeriesResistor->setParameters(mSeriesRes);
	mSubSeriesResistor->connect({ mTerminals[0]->node(), mVirtualNodes[0] });
	mSubSeriesResistor->initialize(mFrequencies);
	mSubSeriesResistor->initializeFromNodesAndTerminals(frequency);

	mSubSeriesInductor = std::make_shared<SP::Ph1::Inductor>(mName + "_ind", mLogLevel);
	mSubSeriesInductor->setParameters(mSeriesInd);
	mSubSeriesInductor->connect({ mVirtualNodes[0], mTerminals[1]->node() });
	mSubSeriesInductor->initialize(mFrequencies);
	mSubSeriesInductor->initializeFromNodesAndTerminals(frequency);

	// Create parallel sub components
	if (mParallelCond >= 0) {
		mSubParallelResistor0 = std::make_shared<SP::Ph1::Resistor>(mName + "_con0", mLogLevel);
		mSubParallelResistor0->setParameters(2. / mParallelCond);
		mSubParallelResistor0->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node() });
		mSubParallelResistor0->initialize(mFrequencies);
		mSubParallelResistor0->initializeFromNodesAndTerminals(frequency);

		mSubParallelResistor1 = std::make_shared<SP::Ph1::Resistor>(mName + "_con1", mLogLevel);
		mSubParallelResistor1->setParameters(2. / mParallelCond);
		mSubParallelResistor1->connect(SimNode::List{ SimNode::GND, mTerminals[1]->node() });
		mSubParallelResistor1->initialize(mFrequencies);
		mSubParallelResistor1->initializeFromNodesAndTerminals(frequency);
	}

	if (mParallelCap >= 0) {
		mSubParallelCapacitor0 = std::make_shared<SP::Ph1::Capacitor>(mName + "_cap0", mLogLevel);
		mSubParallelCapacitor0->setParameters(mParallelCap / 2.);
		mSubParallelCapacitor0->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node() });
		mSubParallelCapacitor0->initialize(mFrequencies);
		mSubParallelCapacitor0->initializeFromNodesAndTerminals(frequency);

		mSubParallelCapacitor1 = std::make_shared<SP::Ph1::Capacitor>(mName + "_cap1", mLogLevel);
		mSubParallelCapacitor1->setParameters(mParallelCap / 2.);
		mSubParallelCapacitor1->connect(SimNode::List{ SimNode::GND, mTerminals[1]->node() });
		mSubParallelCapacitor1->initialize(mFrequencies);
		mSubParallelCapacitor1->initializeFromNodesAndTerminals(frequency);
	}

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

void SP::Ph1::PiLine::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	MNAInterface::List subComps({ mSubSeriesResistor, mSubSeriesInductor });

	mSubSeriesResistor->mnaInitialize(omega, timeStep, leftVector);
	mSubSeriesInductor->mnaInitialize(omega, timeStep, leftVector);
	mRightVectorStamps.push_back(&mSubSeriesInductor->attribute<Matrix>("right_vector")->get());

	mSubParallelResistor0->mnaInitialize(omega, timeStep, leftVector);
	mSubParallelResistor1->mnaInitialize(omega, timeStep, leftVector);
	subComps.push_back(mSubParallelResistor0);
	subComps.push_back(mSubParallelResistor1);

	if (mParallelCap >= 0) {
		mSubParallelCapacitor0->mnaInitialize(omega, timeStep, leftVector);
		mSubParallelCapacitor1->mnaInitialize(omega, timeStep, leftVector);
		mRightVectorStamps.push_back(&mSubParallelCapacitor0->attribute<Matrix>("right_vector")->get());
		mRightVectorStamps.push_back(&mSubParallelCapacitor1->attribute<Matrix>("right_vector")->get());
		subComps.push_back(mSubParallelCapacitor0);
		subComps.push_back(mSubParallelCapacitor1);
	}

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void SP::Ph1::PiLine::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubSeriesResistor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSeriesInductor->mnaApplySystemMatrixStamp(systemMatrix);

	mSubParallelResistor0->mnaApplySystemMatrixStamp(systemMatrix);
	mSubParallelResistor1->mnaApplySystemMatrixStamp(systemMatrix);

	if (mParallelCap >= 0) {
		mSubParallelCapacitor0->mnaApplySystemMatrixStamp(systemMatrix);
		mSubParallelCapacitor1->mnaApplySystemMatrixStamp(systemMatrix);
	}
}

void SP::Ph1::PiLine::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;
}

void SP::Ph1::PiLine::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	this->mSubSeriesResistor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	this->mSubSeriesInductor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	if (this->mParallelCap >= 0) {
		this->mSubParallelCapacitor0->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
		this->mSubParallelCapacitor1->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	}

	this->mSubParallelResistor0->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	this->mSubParallelResistor1->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);

	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("i_intf"));
}

void SP::Ph1::PiLine::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	this->mSubSeriesResistor->mnaPostStep(time, timeStepCount, leftVector);
	this->mSubSeriesInductor->mnaPostStep(time, timeStepCount, leftVector);
	if (this->mParallelCap >= 0) {
		this->mSubParallelCapacitor0->mnaPostStep(time, timeStepCount, leftVector);
		this->mSubParallelCapacitor1->mnaPostStep(time, timeStepCount, leftVector);
	}

	this->mSubParallelResistor0->mnaPostStep(time, timeStepCount, leftVector);
	this->mSubParallelResistor1->mnaPostStep(time, timeStepCount, leftVector);

	// post-step of component itself
	this->mnaUpdateVoltage(*leftVector);
	this->mnaUpdateCurrent(*leftVector);
}

void SP::Ph1::PiLine::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0, 0) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void SP::Ph1::PiLine::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = mSubSeriesInductor->intfCurrent()(0, 0);
}

MNAInterface::List SP::Ph1::PiLine::mnaTearGroundComponents() {
	MNAInterface::List gndComponents;

	gndComponents.push_back(mSubParallelResistor0);
	gndComponents.push_back(mSubParallelResistor1);

	if (mParallelCap >= 0) {
		gndComponents.push_back(mSubParallelCapacitor0);
		gndComponents.push_back(mSubParallelCapacitor1);
	}

	return gndComponents;
}

void SP::Ph1::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
	mSubSeriesResistor->mnaTearSetIdx(mTearIdx);
	mSubSeriesResistor->mnaTearInitialize(omega, timeStep);
	mSubSeriesInductor->mnaTearSetIdx(mTearIdx);
	mSubSeriesInductor->mnaTearInitialize(omega, timeStep);
}

void SP::Ph1::PiLine::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
	mSubSeriesResistor->mnaTearApplyMatrixStamp(tearMatrix);
	mSubSeriesInductor->mnaTearApplyMatrixStamp(tearMatrix);
}

void SP::Ph1::PiLine::mnaTearApplyVoltageStamp(Matrix& voltageVector) {
	mSubSeriesInductor->mnaTearApplyVoltageStamp(voltageVector);
}

void SP::Ph1::PiLine::mnaTearPostStep(Complex voltage, Complex current) {
	mSubSeriesInductor->mnaTearPostStep(voltage - current * mSeriesRes, current);
}
