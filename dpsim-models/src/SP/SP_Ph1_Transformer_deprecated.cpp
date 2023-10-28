/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_Transformer.h>

using namespace CPS;

// #### General ####
SP::Ph1::Transformer::Transformer(String uid, String name, Logger::Level logLevel, Bool withResistiveLosses)
	: Base::Ph1::Transformer(mAttributes), CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	mBaseVoltage(mAttributes->create<Real>("base_Voltage")),
	mCurrent(mAttributes->create<MatrixComp>("current_vector")),
	mActivePowerBranch(mAttributes->create<Matrix>("p_branch_vector")),
	mReactivePowerBranch(mAttributes->create<Matrix>("q_branch_vector")),
	mActivePowerInjection(mAttributes->create<Real>("p_inj")),
	mReactivePowerInjection(mAttributes->create<Real>("q_inj")) {
	if (withResistiveLosses)
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
	setTerminalNumber(2);

	**mCurrent = MatrixComp::Zero(2,1);
	**mActivePowerBranch = Matrix::Zero(2,1);
	**mReactivePowerBranch = Matrix::Zero(2,1);
}


void SP::Ph1::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	// Note: to be consistent impedance values must be referred to high voltage side (and base voltage set to higher voltage)
	Base::Ph1::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);

	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]", **mNominalVoltageEnd1, **mNominalVoltageEnd2);
	SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [Ohm] Inductance={} [H] (referred to primary side)", **mResistance, **mInductance);
    SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [/] Phase Shift={} [deg]", std::abs(**mRatio), std::arg(**mRatio));
	SPDLOG_LOGGER_INFO(mSLog, "Rated Power ={} [W]", **mRatedPower);

	mRatioAbs = std::abs(**mRatio);
	mRatioPhase = std::arg(**mRatio);

	mParametersSet = true;
}

void SP::Ph1::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	**mRatedPower = ratedPower;
	SPDLOG_LOGGER_INFO(mSLog, "Rated Power ={} [W]", **mRatedPower);

	SP::Ph1::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);
}


/// DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr SP::Ph1::Transformer::clone(String name) {
	auto copy = Transformer::make(name, mLogLevel);
	copy->setParameters(**mNominalVoltageEnd1, **mNominalVoltageEnd2, **mRatedPower,
		std::abs(**mRatio), std::arg(**mRatio), **mResistance, **mInductance);
	return copy;
}

void SP::Ph1::Transformer::initializeFromNodesAndTerminals(Real frequency) {
	mNominalOmega = 2. * PI * frequency;
	mReactance = mNominalOmega * **mInductance;
	SPDLOG_LOGGER_INFO(mSLog, "Reactance={} [Ohm] (referred to primary side)", mReactance);

	// Component parameters are referred to higher voltage side.
	// Switch terminals to have terminal 0 at higher voltage side
	// if transformer is connected the other way around.
	if (Math::abs(**mRatio) < 1.) {
		**mRatio = 1. / **mRatio;
		mRatioAbs = std::abs(**mRatio);
		mRatioPhase = std::arg(**mRatio);
		std::shared_ptr<SimTerminal<Complex>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
		Real tmpVolt = **mNominalVoltageEnd1;
		**mNominalVoltageEnd1 = **mNominalVoltageEnd2;
		**mNominalVoltageEnd2 = tmpVolt;
		SPDLOG_LOGGER_INFO(mSLog, "Switching terminals to have first terminal at higher voltage side. Updated parameters: ");
		SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage End 1 = {} [V] Nominal Voltage End 2 = {} [V]", **mNominalVoltageEnd1, **mNominalVoltageEnd2);
		SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio = {} [ ] Phase Shift = {} [deg]", mRatioAbs, mRatioPhase);
	}

	// Set initial voltage of virtual node in between
	mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(1) * **mRatio);

	// Static calculations from load flow data
	Complex impedance = { **mResistance, mReactance };
	(**mIntfVoltage)(0, 0) = mVirtualNodes[0]->initialSingleVoltage() - initialSingleVoltage(0);
	(**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / impedance;

	// Create series sub components
	mSubInductor = std::make_shared<SP::Ph1::Inductor>(**mUID + "_ind", **mName + "_ind", Logger::Level::off);
	mSubInductor->setParameters(**mInductance);
	addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

	if (mNumVirtualNodes == 3) {
		mVirtualNodes[2]->setInitialVoltage(initialSingleVoltage(0));
		mSubResistor = std::make_shared<SP::Ph1::Resistor>(**mUID + "_res", **mName + "_res", Logger::Level::off);
		mSubResistor->setParameters(**mResistance);
		mSubResistor->connect({ node(0), mVirtualNodes[2] });
		mSubInductor->connect({ mVirtualNodes[2], mVirtualNodes[0] });
		addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	} else {
		mSubInductor->connect({ node(0), mVirtualNodes[0] });
	}

	// Create parallel sub components for init and mna behaviour
	if (mBehaviour == TopologicalPowerComp::Behaviour::Initialization || mBehaviour == TopologicalPowerComp::Behaviour::MNASimulation) {

		Real pSnub = P_SNUB_TRANSFORMER * **mRatedPower;
		Real qSnub = Q_SNUB_TRANSFORMER * **mRatedPower;

		// A snubber conductance is added on the higher voltage side
		mSnubberResistance1 = std::pow(std::abs(**mNominalVoltageEnd1),2) / pSnub;
		mSubSnubResistor1 = std::make_shared<SP::Ph1::Resistor>(**mName + "_snub_res1", mLogLevel);
		mSubSnubResistor1->setParameters(mSnubberResistance1);
		mSubSnubResistor1->connect({ node(0), SP::SimNode::GND });
		SPDLOG_LOGGER_INFO(mSLog, "Snubber Resistance 1 (connected to higher voltage side {}) = {} [Ohm]", node(0)->name(), Logger::realToString(mSnubberResistance1));
		mSubSnubResistor1->setBaseVoltage(**mNominalVoltageEnd1);
		addMNASubComponent(mSubSnubResistor1, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

		// A snubber conductance is added on the lower voltage side
		mSnubberResistance2 = std::pow(std::abs(**mNominalVoltageEnd2),2) / pSnub;
		mSubSnubResistor2 = std::make_shared<SP::Ph1::Resistor>(**mName + "_snub_res2", mLogLevel);
		mSubSnubResistor2->setParameters(mSnubberResistance2);
		mSubSnubResistor2->connect({ node(1), SP::SimNode::GND });
		SPDLOG_LOGGER_INFO(mSLog, "Snubber Resistance 2 (connected to lower voltage side {}) = {} [Ohm]", node(1)->name(), Logger::realToString(mSnubberResistance2));
		mSubSnubResistor2->setBaseVoltage(**mNominalVoltageEnd2);
		addMNASubComponent(mSubSnubResistor2, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

		// // A snubber capacitance is added to higher voltage side (not used as capacitor at high voltage side made it worse)
		// mSnubberCapacitance1 = qSnub / std::pow(std::abs(mNominalVoltageEnd1),2) / mNominalOmega;
		// mSubSnubCapacitor1 = std::make_shared<SP::Ph1::Capacitor>(**mName + "_snub_cap1", mLogLevel);
		// mSubSnubCapacitor1->setParameters(mSnubberCapacitance1);
		// mSubSnubCapacitor1->connect({ node(0), SP::SimNode::GND });
		// SPDLOG_LOGGER_INFO(mSLog, "Snubber Capacitance 1 (connected to higher voltage side {}) = \n{} [F] \n ", node(0)->name(), Logger::realToString(mSnubberCapacitance1));
		// mSubSnubCapacitor1->setBaseVoltage(mNominalVoltageEnd1);
		// mSubComponents.push_back(mSubSnubCapacitor1);

		// A snubber capacitance is added to lower voltage side
		mSnubberCapacitance2 = qSnub / std::pow(std::abs(**mNominalVoltageEnd2),2) / mNominalOmega;
		mSubSnubCapacitor2 = std::make_shared<SP::Ph1::Capacitor>(**mName + "_snub_cap2", mLogLevel);
		mSubSnubCapacitor2->setParameters(mSnubberCapacitance2);
		mSubSnubCapacitor2->connect({ node(1), SP::SimNode::GND });
		SPDLOG_LOGGER_INFO(mSLog, "Snubber Capacitance 2 (connected to lower voltage side {}) = {} [F]", node(1)->name(), Logger::realToString(mSnubberCapacitance2));
		mSubSnubCapacitor2->setBaseVoltage(**mNominalVoltageEnd2);
		addMNASubComponent(mSubSnubCapacitor2, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	// Initialize electrical subcomponents
	SPDLOG_LOGGER_INFO(mSLog, "Electrical subcomponents: ");
	for (auto subcomp: mSubComponents) {
		SPDLOG_LOGGER_INFO(mSLog, "- {}", subcomp->name());
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\nVirtual Node 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		Logger::phasorToString((**mIntfCurrent)(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}


// #### Powerflow section ####

void SP::Ph1::Transformer::setBaseVoltage(Real baseVoltage) {
	// Note: to be consistent set base voltage to higher voltage (and impedance values must be referred to high voltage side)
	// TODO: use attribute setter for setting base voltage
    **mBaseVoltage = baseVoltage;
}

void SP::Ph1::Transformer::calculatePerUnitParameters(Real baseApparentPower, Real baseOmega) {
	SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}", **mName);
    mBaseApparentPower = baseApparentPower;
	mBaseOmega = baseOmega;
    SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]  Base Omega={} [1/s]", baseApparentPower, baseOmega);

	mBaseImpedance = **mBaseVoltage * **mBaseVoltage / mBaseApparentPower;
	mBaseAdmittance = 1.0 / mBaseImpedance;
	mBaseCurrent = baseApparentPower / (**mBaseVoltage * sqrt(3)); // I_base=(S_threephase/3)/(V_line_to_line/sqrt(3))
	SPDLOG_LOGGER_INFO(mSLog, "Base Voltage={} [V]  Base Impedance={} [Ohm]", **mBaseVoltage, mBaseImpedance);

	mResistancePerUnit = **mResistance / mBaseImpedance;
	mReactancePerUnit = mReactance / mBaseImpedance;
    SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [pu]  Reactance={} [pu]", mResistancePerUnit, mReactancePerUnit);

	mBaseInductance = mBaseImpedance / mBaseOmega;
	mInductancePerUnit = **mInductance / mBaseInductance;
	// omega per unit=1, hence 1.0*mInductancePerUnit.
	mLeakagePerUnit = Complex(mResistancePerUnit,1.*mInductancePerUnit);
	SPDLOG_LOGGER_INFO(mSLog, "Leakage Impedance={} [pu] ", mLeakagePerUnit);

    mRatioAbsPerUnit = mRatioAbs / **mNominalVoltageEnd1 * **mNominalVoltageEnd2;
    SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [pu]", mRatioAbsPerUnit);

	// Calculate per unit parameters of subcomps
	if (mSubSnubResistor1)
		mSubSnubResistor1->calculatePerUnitParameters(mBaseApparentPower);
	if (mSubSnubResistor2)
		mSubSnubResistor2->calculatePerUnitParameters(mBaseApparentPower);
	if (mSubSnubCapacitor1)
		mSubSnubCapacitor1->calculatePerUnitParameters(mBaseApparentPower);
	if (mSubSnubCapacitor2)
		mSubSnubCapacitor2->calculatePerUnitParameters(mBaseApparentPower);
}

void SP::Ph1::Transformer::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) {
	// calculate matrix stamp
	mY_element = MatrixComp(2, 2);
	Complex y = Complex(1, 0) / mLeakagePerUnit;

	mY_element(0, 0) = y;
	mY_element(0, 1) = -y*mRatioAbsPerUnit;
	mY_element(1, 0) = -y*mRatioAbsPerUnit;
	mY_element(1, 1) = y*std::pow(mRatioAbsPerUnit, 2);

	//check for inf or nan
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			if (std::isinf(mY_element.coeff(i, j).real()) || std::isinf(mY_element.coeff(i, j).imag())) {
				std::cout << mY_element << std::endl;
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

	SPDLOG_LOGGER_INFO(mSLog, "#### Y matrix stamping: {}", mY_element);

	if (mSubSnubResistor1)
		mSubSnubResistor1->pfApplyAdmittanceMatrixStamp(Y);
	if (mSubSnubResistor2)
		mSubSnubResistor2->pfApplyAdmittanceMatrixStamp(Y);
	if (mSubSnubCapacitor1)
		mSubSnubCapacitor1->pfApplyAdmittanceMatrixStamp(Y);
	if (mSubSnubCapacitor2)
		mSubSnubCapacitor2->pfApplyAdmittanceMatrixStamp(Y);
}


void SP::Ph1::Transformer::updateBranchFlow(VectorComp& current, VectorComp& powerflow) {
	**mCurrent = current * mBaseCurrent;
	**mActivePowerBranch = powerflow.real()*mBaseApparentPower;
	**mReactivePowerBranch = powerflow.imag()*mBaseApparentPower;
}


void SP::Ph1::Transformer::storeNodalInjection(Complex powerInjection) {
	**mActivePowerInjection = std::real(powerInjection)*mBaseApparentPower;
	**mReactivePowerInjection = std::imag(powerInjection)*mBaseApparentPower;
}

MatrixComp SP::Ph1::Transformer::Y_element() {
	return mY_element;
}

// #### MNA Section ####

void SP::Ph1::Transformer::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	SPDLOG_LOGGER_INFO(mSLog,
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\nTerminal 1 connected to {:s} = sim node {:d}",
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
		mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}


void SP::Ph1::Transformer::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	// Ideal transformer equations
	if (terminalNotGrounded(0)) {
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[1]->matrixNodeIndex(), Complex(-1.0, 0));
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), Complex(1.0, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(1), mVirtualNodes[1]->matrixNodeIndex(), **mRatio);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), matrixNodeIndex(1), - **mRatio);
	}

	// Add subcomps to system matrix
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);

	if (terminalNotGrounded(0)) {
		SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[1]->matrixNodeIndex());
		SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex());
	}
	if (terminalNotGrounded(1)) {
		SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})", Logger::complexToString(**mRatio),
			matrixNodeIndex(1), mVirtualNodes[1]->matrixNodeIndex());
		SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})", Logger::complexToString(- **mRatio),
			mVirtualNodes[1]->matrixNodeIndex(), matrixNodeIndex(1));
	}
}

void SP::Ph1::Transformer::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mRightVector);
}

void SP::Ph1::Transformer::mnaParentPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void SP::Ph1::Transformer::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::Transformer::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void SP::Ph1::Transformer::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = mSubInductor->intfCurrent()(0, 0);
	SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString((**mIntfCurrent)(0, 0)));

}

void SP::Ph1::Transformer::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	(**mIntfVoltage)(0, 0) = 0;
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	(**mIntfVoltage)(0, 0) = (**mIntfVoltage)(0, 0) - Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex());
	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString((**mIntfVoltage)(0, 0)));
}
