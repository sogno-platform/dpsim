/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_Transformer.h>

using namespace CPS;

DP::Ph1::Transformer::Transformer(String uid, String name,
	Logger::Level logLevel, Bool withResistiveLosses)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	if (withResistiveLosses)
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);

	setTerminalNumber(2);

	mSLog->info("Create {} {}", this->type(), name);
	**mIntfVoltage = MatrixComp::Zero(1,1);
	**mIntfCurrent = MatrixComp::Zero(1,1);

	///FIXME: Initialization should happen in the base class declaring the attribute. However, this base class is currently not an AttributeList...
	mNominalVoltageEnd1 = Attribute<Real>::create("nominal_voltage_end1", mAttributes);
	mNominalVoltageEnd2 = Attribute<Real>::create("nominal_voltage_end2", mAttributes);
	mRatedPower = Attribute<Real>::create("S", mAttributes);
	mRatio = Attribute<Complex>::create("ratio", mAttributes);
	mResistance = Attribute<Real>::create("R", mAttributes);
	mInductance = Attribute<Real>::create("L", mAttributes);
}

/// DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr DP::Ph1::Transformer::clone(String name) {
	auto copy = Transformer::make(name, mLogLevel);
	copy->setParameters(**mNominalVoltageEnd1, **mNominalVoltageEnd2, std::abs(**mRatio), std::arg(**mRatio), **mResistance, **mInductance);
	return copy;
}

void DP::Ph1::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs, Real ratioPhase,
	Real resistance, Real inductance) {

	Base::Ph1::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);

	mSLog->info("Nominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]", **mNominalVoltageEnd1, **mNominalVoltageEnd2);
	mSLog->info("Resistance={} [Ohm] Inductance={} [Ohm] (referred to primary side)", **mResistance, **mInductance);
    mSLog->info("Tap Ratio={} [ ] Phase Shift={} [deg]", std::abs(**mRatio), std::arg(**mRatio));
	mSLog->info("Rated Power ={} [W]", **mRatedPower);

	mParametersSet = true;
}

void DP::Ph1::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	**mRatedPower = ratedPower;
	mSLog->info("Rated Power ={} [W]", **mRatedPower);

	DP::Ph1::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);
}

void DP::Ph1::Transformer::initializeFromNodesAndTerminals(Real frequency) {

	// Component parameters are referred to higher voltage side.
	// Switch terminals to have terminal 0 at higher voltage side
	// if transformer is connected the other way around.
	if (Math::abs(**mRatio) < 1.) {
		**mRatio = 1. / **mRatio;
		std::shared_ptr<SimTerminal<Complex>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
		Real tmpVolt = **mNominalVoltageEnd1;
		**mNominalVoltageEnd1 = **mNominalVoltageEnd2;
		**mNominalVoltageEnd2 = tmpVolt;
		mSLog->info("Switching terminals to have first terminal at higher voltage side. Updated parameters: ");
		mSLog->info("Nominal Voltage End 1 = {} [V] Nominal Voltage End 2 = {} [V]", **mNominalVoltageEnd1, **mNominalVoltageEnd2);
		mSLog->info("Tap Ratio = {} [ ] Phase Shift = {} [deg]", std::abs(**mRatio), std::arg(**mRatio));
	}

	// Set initial voltage of virtual node in between
	mVirtualNodes[0]->setInitialVoltage( initialSingleVoltage(1) * **mRatio );

	// Static calculations from load flow data
	Real omega = 2.*PI* frequency;
	Complex impedance = { **mResistance, omega * **mInductance };
	mSLog->info("Reactance={} [Ohm] (referred to primary side)", omega * **mInductance );
	(**mIntfVoltage)(0,0) = mVirtualNodes[0]->initialSingleVoltage() - initialSingleVoltage(0);
	(**mIntfCurrent)(0,0) = (**mIntfVoltage)(0,0) / impedance;

	// Create series sub components
	mSubInductor = std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
	mSubComponents.push_back(mSubInductor);
	mSubInductor->setParameters(**mInductance);

	if (mNumVirtualNodes == 3) {
		mVirtualNodes[2]->setInitialVoltage(initialSingleVoltage(0));
		mSubResistor = std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);
		mSubResistor->setParameters(**mResistance);
		mSubResistor->connect({node(0), mVirtualNodes[2]});
		mSubInductor->connect({mVirtualNodes[2], mVirtualNodes[0]});
		mSubComponents.push_back(mSubResistor);
	} else {
		mSubInductor->connect({node(0), mVirtualNodes[0]});
	}
	
	// Create parallel sub components 
	Real pSnub = P_SNUB_TRANSFORMER * **mRatedPower;
	Real qSnub = Q_SNUB_TRANSFORMER * **mRatedPower;

	// A snubber conductance is added on the higher voltage side
	mSnubberResistance1 = std::pow(std::abs(**mNominalVoltageEnd1),2) / pSnub;
	mSubSnubResistor1 = std::make_shared<DP::Ph1::Resistor>(**mName + "_snub_res1", mLogLevel);
	mSubSnubResistor1->setParameters(mSnubberResistance1);
	mSubSnubResistor1->connect({ node(0), DP::SimNode::GND });
	mSLog->info("Snubber Resistance 1 (connected to higher voltage side {}) = {} [Ohm]", node(0)->name(), Logger::realToString(mSnubberResistance1));
	mSubComponents.push_back(mSubSnubResistor1);

	// A snubber conductance is added on the lower voltage side
	mSnubberResistance2 = std::pow(std::abs(**mNominalVoltageEnd2),2) / pSnub;
	mSubSnubResistor2 = std::make_shared<DP::Ph1::Resistor>(**mName + "_snub_res2", mLogLevel);
	mSubSnubResistor2->setParameters(mSnubberResistance2);
	mSubSnubResistor2->connect({ node(1), DP::SimNode::GND });
	mSLog->info("Snubber Resistance 2 (connected to lower voltage side {}) = {} [Ohm]", node(1)->name(), Logger::realToString(mSnubberResistance2));
	mSubComponents.push_back(mSubSnubResistor2);

	// // A snubber capacitance is added to higher voltage side (not used as capacitor at high voltage side made it worse)
	// mSnubberCapacitance1 = qSnub / std::pow(std::abs(mNominalVoltageEnd1),2) / omega;
	// mSubSnubCapacitor1 = std::make_shared<DP::Ph1::Capacitor>(**mName + "_snub_cap1", mLogLevel);
	// mSubSnubCapacitor1->setParameters(mSnubberCapacitance1);
	// mSubSnubCapacitor1->connect({ node(0), DP::SimNode::GND });
	// mSLog->info("Snubber Capacitance 1 (connected to higher voltage side {}) = \n{} [F] \n ", node(0)->name(), Logger::realToString(mSnubberCapacitance1));
	// mSubComponents.push_back(mSubSnubCapacitor1);

	// A snubber capacitance is added to lower voltage side 
	mSnubberCapacitance2 = qSnub / std::pow(std::abs(**mNominalVoltageEnd2),2) / omega;
	mSubSnubCapacitor2 = std::make_shared<DP::Ph1::Capacitor>(**mName + "_snub_cap2", mLogLevel);
	mSubSnubCapacitor2->setParameters(mSnubberCapacitance2);
	mSubSnubCapacitor2->connect({ node(1), DP::SimNode::GND });
	mSLog->info("Snubber Capacitance 2 (connected to lower voltage side {}) = {} [F]", node(1)->name(), Logger::realToString(mSnubberCapacitance2));
	mSubComponents.push_back(mSubSnubCapacitor2);

	// Initialize electrical subcomponents
	mSLog->info("Electrical subcomponents: ");
	for (auto subcomp: mSubComponents) {
		mSLog->info("- {}", subcomp->name());
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\nVirtual Node 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

void DP::Ph1::Transformer::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaInitialize(omega, timeStep, leftVector);

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mSLog->info(
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\nTerminal 1 connected to {:s} = sim node {:d}",
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
		mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}

void DP::Ph1::Transformer::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
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
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->matrixNodeIndex(),  mVirtualNodes[1]->matrixNodeIndex());
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex());
	}
	if (terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(**mRatio),
			matrixNodeIndex(1), mVirtualNodes[1]->matrixNodeIndex());
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(- **mRatio),
			mVirtualNodes[1]->matrixNodeIndex(), matrixNodeIndex(1));
	}
}

void DP::Ph1::Transformer::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Add subcomps to right side vector
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::Transformer::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(this->attribute("i_intf"));
	prevStepDependencies.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("right_vector"));
}

void DP::Ph1::Transformer::mnaPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	this->mnaApplyRightSideVectorStamp(**this->mRightVector);
}

void DP::Ph1::Transformer::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("i_intf"));
}

void DP::Ph1::Transformer::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaPostStep(time, timeStepCount, leftVector);
	// post-step of component itself
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void DP::Ph1::Transformer::mnaUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0,0) = mSubInductor->intfCurrent()(0, 0);
}

void DP::Ph1::Transformer::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	(**mIntfVoltage)(0, 0) = 0;
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	(**mIntfVoltage)(0, 0) = (**mIntfVoltage)(0, 0) - Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex());
	mSLog->debug("Voltage {:s}", Logger::phasorToString((**mIntfVoltage)(0, 0)));
}

