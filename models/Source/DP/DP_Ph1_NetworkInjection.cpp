/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_NetworkInjection.h>

using namespace CPS;

DP::Ph1::NetworkInjection::NetworkInjection(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(0);
	setTerminalNumber(1);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	// Create electrical sub components
	mSubVoltageSource = std::make_shared<DP::Ph1::VoltageSource>(mName + "_vs", mLogLevel);
	mSubComponents.push_back(mSubVoltageSource);
	mSLog->info("Electrical subcomponents: ");
	for (auto subcomp: mSubComponents)
		mSLog->info("- {}", subcomp->name());

	addAttributeRef<Complex>("V_ref", mSubVoltageSource->attribute<Complex>("V_ref"), Flags::read | Flags::write);
	addAttributeRef<Real>("f_src", mSubVoltageSource->attribute<Real>("f_src"), Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr DP::Ph1::NetworkInjection::clone(String name) {
	auto copy = NetworkInjection::make(name, mLogLevel);
	copy->setParameters(attribute<Complex>("V_ref")->get());
	return copy;
}

void DP::Ph1::NetworkInjection::setParameters(Complex voltageRef, Real srcFreq) {
	mParametersSet = true;

	mSubVoltageSource->setParameters(voltageRef, srcFreq);

	setAttributeRef("V_ref", mSubVoltageSource->attribute<Complex>("V_ref"));
	setAttributeRef("f_src", mSubVoltageSource->attribute<Real>("f_src"));

	mSLog->info("\nVoltage Ref={:s} [V]"
				"\nFrequency={:s} [Hz]",
				Logger::phasorToString(voltageRef),
				Logger::realToString(srcFreq));
}

void DP::Ph1::NetworkInjection::setParameters(Complex initialPhasor, Real freqStart, Real rocof, Real timeStart, Real duration, bool useAbsoluteCalc) {
	mParametersSet = true;

	mSubVoltageSource->setParameters(initialPhasor, freqStart, rocof, timeStart, duration, useAbsoluteCalc);

	setAttributeRef("V_ref", mSubVoltageSource->attribute<Complex>("V_ref"));
	setAttributeRef("f_src", mSubVoltageSource->attribute<Real>("f_src"));

	mSLog->info("\nVoltage Ref={:s} [V]"
				"\nFrequency={:s} [Hz]",
				Logger::phasorToString(initialPhasor),
				Logger::realToString(freqStart));
}

void DP::Ph1::NetworkInjection::setParameters(Complex initialPhasor, Real modulationFrequency, Real modulationAmplitude, Real baseFrequency /*= 0.0*/, bool zigzag /*= false*/) {
	mParametersSet = true;

	mSubVoltageSource->setParameters(initialPhasor, modulationFrequency, modulationAmplitude, baseFrequency, zigzag);

	setAttributeRef("V_ref", mSubVoltageSource->attribute<Complex>("V_ref"));
	setAttributeRef("f_src", mSubVoltageSource->attribute<Real>("f_src"));

	mSLog->info("\nVoltage Ref={:s} [V]"
				"\nFrequency={:s} [Hz]",
				Logger::phasorToString(initialPhasor),
				Logger::realToString(baseFrequency));
}

void DP::Ph1::NetworkInjection::initializeFromNodesAndTerminals(Real frequency) {
	// Connect electrical subcomponents
	mSubVoltageSource->connect({ SimNode::GND, node(0) });

	// Initialize electrical subcomponents
	for (auto subcomp: mSubComponents) {
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}
}

// #### MNA functions ####

void DP::Ph1::NetworkInjection::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	// initialize electrical subcomponents
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaInitialize(omega, timeStep, leftVector);

	// collect right side vectors of subcomponents
	mRightVectorStamps.push_back(&mSubVoltageSource->attribute<Matrix>("right_vector")->get());

	// collect tasks
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::NetworkInjection::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::NetworkInjection::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;

	mSLog->debug("Right Side Vector: {:s}",
				Logger::matrixToString(rightVector));
}

void DP::Ph1::NetworkInjection::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(attribute("i_intf"));
	prevStepDependencies.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void DP::Ph1::NetworkInjection::mnaPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(mRightVector);
}

void DP::Ph1::NetworkInjection::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void DP::Ph1::NetworkInjection::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	for (auto subcomp: mSubComponents)
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
			mnasubcomp->mnaPostStep(time, timeStepCount, leftVector);
	// post-step of component itself
	mnaUpdateCurrent(*leftVector);
	mnaUpdateVoltage(*leftVector);
}

void DP::Ph1::NetworkInjection::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage = mSubVoltageSource->attribute<MatrixComp>("v_intf")->get();
}

void DP::Ph1::NetworkInjection::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mSubVoltageSource->attribute<MatrixComp>("i_intf")->get();
}

void DP::Ph1::NetworkInjection::daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off){
	/* new state vector definintion:
		state[0]=node0_voltage
		state[1]=node1_voltage
		....
		state[n]=noden_voltage
		state[n+1]=component0_voltage
		state[n+2]=component0_inductance (not yet implemented)
		...
		state[m-1]=componentm_voltage
		state[m]=componentm_inductance
	*/

    int Pos1 = matrixNodeIndex(0);
    int Pos2 = matrixNodeIndex(1);
	int c_offset = off[0]+off[1]; //current offset for component
	int n_offset_1 = c_offset + Pos1 +1;// current offset for first nodal equation
	int n_offset_2 = c_offset + Pos2 +1;// current offset for second nodal equation
	resid[c_offset] = (state[Pos2]-state[Pos1]) - state[c_offset]; // Voltage equation for Resistor
	//resid[++c_offset] = ; //TODO : add inductance equation
	resid[n_offset_1] += mIntfCurrent(0, 0).real();
	resid[n_offset_2] += mIntfCurrent(0, 0).real();
	off[1] += 1;
}

Complex DP::Ph1::NetworkInjection::daeInitialize() {
	mIntfVoltage(0,0) = mSubVoltageSource->attribute<Complex>("v_intf")->get();
	return mSubVoltageSource->attribute<Complex>("v_intf")->get();
}
