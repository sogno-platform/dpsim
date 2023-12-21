/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_RXLoad.h>

using namespace CPS;

EMT::Ph3::RXLoad::RXLoad(String uid, String name, Logger::Level logLevel)
	: CompositePowerComp<Real>(uid, name, true, true, logLevel),
	mActivePower(mAttributes->create<Matrix>("P")),
	mReactivePower(mAttributes->create<Matrix>("Q")),
	mNomVoltage(mAttributes->create<Real>("V_nom")) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
	mSLog->flush();
}

EMT::Ph3::RXLoad::RXLoad(String name,
	Logger::Level logLevel)
	: RXLoad(name, name, logLevel) {
}

EMT::Ph3::RXLoad::RXLoad(String name,
	Matrix activePower, Matrix reactivePower, Real nominalVoltage,
	Logger::Level logLevel)
	: RXLoad(name, logLevel) {

	setParameters(activePower, reactivePower, nominalVoltage);
}

void EMT::Ph3::RXLoad::setParameters(Real activePower, Real reactivePower) {
	this->setParameters(Math::singlePhaseParameterToThreePhase(activePower / 3.), 
						Math::singlePhaseParameterToThreePhase(reactivePower / 3.));
}

void EMT::Ph3::RXLoad::setParameters(Matrix activePower, Matrix reactivePower) {
	**mActivePower = activePower;
	**mReactivePower = reactivePower;

	// complex power
	mPower = MatrixComp::Zero(3, 3);
	mPower(0, 0) = { (**mActivePower)(0, 0), (**mReactivePower)(0, 0) };
	mPower(1, 1) = { (**mActivePower)(1, 1), (**mReactivePower)(1, 1) };
	mPower(2, 2) = { (**mActivePower)(2, 2), (**mReactivePower)(2, 2) };

	mInitPowerFromTerminal = false;

	SPDLOG_LOGGER_INFO(mSLog, 
			"\nActive Power [W]: {}"
			"\nReactive Power [VAr]: {}",
			Logger::matrixToString(**mActivePower),
			Logger::matrixToString(**mReactivePower));
	mSLog->flush();
}

void EMT::Ph3::RXLoad::setParameters(Real activePower, Real reactivePower, Real nominalVoltage) {
	
	this->setParameters(Math::singlePhaseParameterToThreePhase(activePower / 3.), 
						Math::singlePhaseParameterToThreePhase(reactivePower / 3.));
	**mNomVoltage = nominalVoltage;
	mInitVoltageFromNode = false;
	
	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
}

void EMT::Ph3::RXLoad::setParameters(Matrix activePower, Matrix reactivePower, Real nominalVoltage) {
	
	setParameters(activePower, reactivePower);
	**mNomVoltage = nominalVoltage;
	mInitVoltageFromNode = false;
	
	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
}

void EMT::Ph3::RXLoad::initializeFromNodesAndTerminals(Real frequency) {

	if (mInitPowerFromTerminal) {
		**mActivePower = Matrix::Zero(3, 3);
		(**mActivePower)(0, 0) = mTerminals[0]->singleActivePower() / 3.;
		(**mActivePower)(1, 1) = mTerminals[0]->singleActivePower() / 3.;
		(**mActivePower)(2, 2) = mTerminals[0]->singleActivePower() / 3.;

		**mReactivePower = Matrix::Zero(3, 3);
		(**mReactivePower)(0, 0) = mTerminals[0]->singleReactivePower() / 3.;
		(**mReactivePower)(1, 1) = mTerminals[0]->singleReactivePower() / 3.;
		(**mReactivePower)(2, 2) = mTerminals[0]->singleReactivePower() / 3.;

		// complex power
		mPower = MatrixComp::Zero(3, 3);
		mPower(0, 0) = { (**mActivePower)(0, 0), (**mReactivePower)(0, 0) };
		mPower(1, 1) = { (**mActivePower)(1, 1), (**mReactivePower)(1, 1) };
		mPower(2, 2) = { (**mActivePower)(2, 2), (**mReactivePower)(2, 2) };

		SPDLOG_LOGGER_INFO(mSLog, "\nActive Power [W]: {}"
					"\nReactive Power [VAr]: {}",
					Logger::matrixToString(**mActivePower),
					Logger::matrixToString(**mReactivePower));
		
	}
	if (mInitVoltageFromNode) {
		**mNomVoltage = std::abs(initialSingleVoltage(0));
		SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
	}

	if ((**mActivePower)(0,0) != 0) {
		mResistance = std::pow(**mNomVoltage / sqrt(3), 2) * (**mActivePower).inverse();
		mSubResistor = std::make_shared<EMT::Ph3::Resistor>(**mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
		**mIntfCurrent += mSubResistor->intfCurrent();
	}

	if ((**mReactivePower)(0, 0) != 0)
		mReactance = std::pow(**mNomVoltage / sqrt(3), 2) * (**mReactivePower).inverse();
	else
		mReactance = Matrix::Zero(1, 1);

	if (mReactance(0,0) > 0) {
		mInductance = mReactance / (2 * PI * frequency);

		mSubInductor = std::make_shared<EMT::Ph3::Inductor>(**mName + "_ind", mLogLevel);
		mSubInductor->setParameters(mInductance);
		mSubInductor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubInductor->initialize(mFrequencies);
		mSubInductor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
		**mIntfCurrent += mSubInductor->intfCurrent();
	}
	else if (mReactance(0,0) < 0) {
		mCapacitance = -1 / (2 * PI * frequency) * mReactance.inverse();

		mSubCapacitor = std::make_shared<EMT::Ph3::Capacitor>(**mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(mCapacitance);
		mSubCapacitor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubCapacitor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
		**mIntfCurrent += mSubCapacitor->intfCurrent();
	}

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * mTerminals[0]->initialSingleVoltage();
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	**mIntfVoltage = vInitABC.real();

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nActive Power: {:s}"
		"\nReactive Power: {:s}"
		"\nResistance: {:s}"
		"\nReactance: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(**mIntfVoltage),
		Logger::matrixToString(**mIntfCurrent),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
		Logger::matrixToString(**mActivePower),
		Logger::matrixToString(**mReactivePower),
		Logger::matrixToString(mResistance),
		Logger::matrixToString(mReactance));
	mSLog->flush();

}

void EMT::Ph3::RXLoad::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	modifiedAttributes.push_back(mRightVector);
};

void EMT::Ph3::RXLoad::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfCurrent);
	modifiedAttributes.push_back(mIntfVoltage);
};

void EMT::Ph3::RXLoad::mnaParentPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::RXLoad::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::RXLoad::mnaCompUpdateVoltage(const Matrix& leftVector) {
	**mIntfVoltage = Matrix::Zero(3, 1);
	(**mIntfVoltage)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	(**mIntfVoltage)(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
	(**mIntfVoltage)(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}

void EMT::Ph3::RXLoad::mnaCompUpdateCurrent(const Matrix& leftVector) {
	**mIntfCurrent = Matrix::Zero(3, 1);
	for (auto& subc : mSubComponents) {
		**mIntfCurrent += subc->intfCurrent();
	}
}
