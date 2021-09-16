/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_Transformer.h>

using namespace CPS;

EMT::Ph3::Transformer::Transformer(String uid, String name,
	Logger::Level logLevel, Bool withResistiveLosses)
	: SimPowerComp<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	if (withResistiveLosses)
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);

	setTerminalNumber(2);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(1, 1);

	addAttribute<Complex>("ratio", &mRatio, Flags::write | Flags::read);
	addAttribute<Matrix>("R", &mResistance, Flags::write | Flags::read);
	addAttribute<Matrix>("L", &mInductance, Flags::write | Flags::read);
	addAttribute<Matrix>("Rsnubber", &mSnubberResistance, Flags::write | Flags::read);
}

SimPowerComp<Real>::Ptr EMT::Ph3::Transformer::clone(String name) {
	auto copy = Transformer::make(name, mLogLevel);
	copy->setParameters(mNominalVoltageEnd1, mNominalVoltageEnd2, std::abs(mRatio), std::arg(mRatio), mResistance, mInductance);
	return copy;
}

void EMT::Ph3::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs, Real ratioPhase,
	Matrix resistance, Matrix inductance) {

	Base::Ph3::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);

	mSLog->info("Nominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]", mNominalVoltageEnd1, mNominalVoltageEnd2);
	mSLog->info("Resistance={} [Ohm] Inductance={} [Ohm] (referred to primary side)", mResistance, mInductance);
    mSLog->info("Tap Ratio={} [ ] Phase Shift={} [deg]", std::abs(mRatio), std::arg(mRatio));

	mParametersSet = true;
}

void EMT::Ph3::Transformer::initializeFromNodesAndTerminals(Real frequency) {

	// Component parameters are referred to high voltage side.
	// Switch terminals if transformer is connected the other way around.
	if (Math::abs(mRatio) < 1.) {
		mRatio = 1. / mRatio;
		std::shared_ptr<SimTerminal<Real>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
	}

	// Set initial voltage of virtual node in between
	mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(1) * mRatio);

	// Static calculations from load flow data
	Real omega = 2. * PI * frequency;
	MatrixComp impedance = MatrixComp::Zero(3, 3);
	impedance <<
		Complex(mResistance(0, 0), omega * mInductance(0, 0)), Complex(mResistance(0, 1), omega * mInductance(0, 1)), Complex(mResistance(0, 2), omega * mInductance(0, 2)),
		Complex(mResistance(1, 0), omega * mInductance(1, 0)), Complex(mResistance(1, 1), omega * mInductance(1, 1)), Complex(mResistance(1, 2), omega * mInductance(1, 2)),
		Complex(mResistance(2, 0), omega * mInductance(2, 0)), Complex(mResistance(2, 1), omega * mInductance(2, 1)), Complex(mResistance(2, 2), omega * mInductance(2, 2));
	mSLog->info("Reactance={} [Ohm] (referred to primary side)", Logger::matrixToString(omega * mInductance));

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = mVirtualNodes[0]->initialSingleVoltage() - RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

	MatrixComp iInit = impedance.inverse() * vInitABC;
	mIntfCurrent = iInit.real();
	mIntfVoltage = vInitABC.real();

	// Create series sub components
	mSubInductor = std::make_shared<EMT::Ph3::Inductor>(mName + "_ind", mLogLevel);
	mSubInductor->setParameters(mInductance);

	if (mNumVirtualNodes == 3) {
		mVirtualNodes[2]->setInitialVoltage(initialSingleVoltage(0));
		mSubResistor = std::make_shared<EMT::Ph3::Resistor>(mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ node(0), mVirtualNodes[2] });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
		mSubInductor->connect({ mVirtualNodes[2], mVirtualNodes[0] });
	}
	else {
		mSubInductor->connect({ node(0), mVirtualNodes[0] });
	}
	mSubInductor->initialize(mFrequencies);
	mSubInductor->initializeFromNodesAndTerminals(frequency);

	// Create parallel sub components
	// A snubber conductance is added on the low voltage side (resistance approximately scaled with LV side voltage)
	Real snubberResistance = mNominalVoltageEnd1 > mNominalVoltageEnd2 ? std::abs(mNominalVoltageEnd2)*1e6 : std::abs(mNominalVoltageEnd1)*1e6;
	mSnubberResistance = Matrix::Identity(3,3)*(snubberResistance);
	mSubSnubResistor = std::make_shared<EMT::Ph3::Resistor>(mName + "_snub_res", mLogLevel);
	mSubSnubResistor->setParameters(mSnubberResistance);
	mSubSnubResistor->connect({ node(1), EMT::SimNode::GND });
	mSubSnubResistor->initialize(mFrequencies);
	mSubSnubResistor->initializeFromNodesAndTerminals(frequency);
	mSLog->info("Snubber Resistance={} [Ohm] (connected to LV side)", Logger::matrixToString(mSnubberResistance));

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\nVirtual Node 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(mIntfVoltage),
		Logger::matrixToString(mIntfCurrent),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * mVirtualNodes[0]->initialSingleVoltage()));
}

void EMT::Ph3::Transformer::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	auto subComponents = MNAInterface::List({ mSubInductor, mSubSnubResistor });
	if (mSubResistor)
		subComponents.push_back(mSubResistor);
	for (auto comp : subComponents) {
		comp->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : comp->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mSLog->info(
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\nTerminal 1 connected to {:s} = sim node {:d}",
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
		mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}

void EMT::Ph3::Transformer::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Ideal transformer equations
	if (terminalNotGrounded(0)) {
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), -1.);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), -1.);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), -1.);

		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), 1.);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1.);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1.);

	}
	if (terminalNotGrounded(1)) {
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(1, 0), mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), mRatio.real());
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(1, 1), mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), mRatio.real());
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(1, 2), mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), mRatio.real());
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(1, 0), -mRatio.real());
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(1, 1), -mRatio.real());
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(1, 2), -mRatio.real());
	}

	// Add inductive part to system matrix
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSnubResistor->mnaApplySystemMatrixStamp(systemMatrix);

	if (mNumVirtualNodes == 3) {
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	}

	if (terminalNotGrounded(0)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[1]->matrixNodeIndex(PhaseType::A));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[1]->matrixNodeIndex(PhaseType::B));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[1]->matrixNodeIndex(PhaseType::C));

		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
	}
	if (terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(mRatio),
			matrixNodeIndex(1, 0), mVirtualNodes[1]->matrixNodeIndex(PhaseType::A));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(mRatio),
			matrixNodeIndex(1, 1), mVirtualNodes[1]->matrixNodeIndex(PhaseType::B));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(mRatio),
			matrixNodeIndex(1, 2), mVirtualNodes[1]->matrixNodeIndex(PhaseType::C));

		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-mRatio),
			mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(1, 0));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-mRatio),
			mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(1, 1));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-mRatio),
			mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(1, 2));
	}
}

void EMT::Ph3::Transformer::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
}

void EMT::Ph3::Transformer::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	mSubInductor->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(attribute("i_intf"));
	prevStepDependencies.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void EMT::Ph3::Transformer::mnaPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents
	mSubInductor->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(mRightVector);
}

void EMT::Ph3::Transformer::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	if (mSubResistor)
		mSubResistor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	mSubInductor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	mSubSnubResistor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void EMT::Ph3::Transformer::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	if (mSubResistor)
		mSubResistor->mnaPostStep(time, timeStepCount, leftVector);
	mSubInductor->mnaPostStep(time, timeStepCount, leftVector);
	mSubSnubResistor->mnaPostStep(time, timeStepCount, leftVector);
	// post-step of component itself
	mnaUpdateVoltage(*leftVector);
	mnaUpdateCurrent(*leftVector);
}

void EMT::Ph3::Transformer::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mSubInductor->intfCurrent();
}

void EMT::Ph3::Transformer::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage = Matrix::Zero(3, 1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
		mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
		mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
		mIntfVoltage(1, 0) = mIntfVoltage(1, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
		mIntfVoltage(2, 0) = mIntfVoltage(2, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	}
}

