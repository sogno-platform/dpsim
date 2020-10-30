/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_VoltageSource.h>
#include <cps/CIM/Reader.h>


using namespace CPS;

EMT::Ph3::VoltageSource::VoltageSource(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	addAttribute<MatrixComp>("V_ref", Flags::read | Flags::write);
	addAttribute<Real>("f_src", Flags::read | Flags::write);
}

void EMT::Ph3::VoltageSource::setParameters(MatrixComp voltageRef, Real srcFreq) {
	attribute<MatrixComp>("V_ref")->set(voltageRef);
	attribute<Real>("f_src")->set(srcFreq);

	mSLog->info("\nVoltage reference phasor [V]: {:s}"
				"\nFrequency [Hz]: {:s}", 
				Logger::matrixCompToString(voltageRef), 
				Logger::realToString(srcFreq));
	mParametersSet = true;
}

void EMT::Ph3::VoltageSource::initializeFromNodesAndTerminals(Real frequency) {
	mVoltageRef = attribute<MatrixComp>("V_ref");
	mSrcFreq = attribute<Real>("f_src");

	mSLog->info("\n--- Initialization from node voltages ---");
	// TODO: this approach currently does not work, if voltage ref set from outside without using setParameters,
	// since mParametersSet remains false then
	if (!mParametersSet) {
		mVoltageRef->set(CIM::Reader::singlePhaseVariableToThreePhase(initialSingleVoltage(1) - initialSingleVoltage(0)));
		mSrcFreq->set(frequency);

		mSLog->info("\nReference voltage: {:s}"
					"\nTerminal 0 voltage: {:s}"
					"\nTerminal 1 voltage: {:s}",
					Logger::matrixCompToString(mVoltageRef->get()),
					Logger::phasorToString(initialSingleVoltage(0)));
	} else {
		mSLog->info("\nInitialization from node voltages omitted (parameter already set)."
					"\nReference voltage: {:s}",
					Logger::matrixCompToString(mVoltageRef->get()));
	}
	mSLog->info("\n--- Initialization from node voltages ---");
	mSLog->flush();
}

SimPowerComp<Real>::Ptr EMT::Ph3::VoltageSource::clone(String name) {
	auto copy = VoltageSource::make(name, mLogLevel);
	copy->setParameters(attribute<MatrixComp>("V_ref")->get(), attribute<Real>("f_src")->get());
	return copy;
}


void EMT::Ph3::VoltageSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);

	updateMatrixNodeIndices();

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

}

void EMT::Ph3::VoltageSource::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), -1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0), -1);

		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), -1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1), -1);

		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), -1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2), -1);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), 1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(1, 0), 1);

		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(1, 1), 1);

		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(1, 2), 1);
	}


	// if (terminalNotGrounded(0)) {
	// 	mLog.debug() << "Add " << -1 << " to " << matrixNodeIndex(0) << "," << mVirtualNodes[0]->matrixNodeIndex() << std::endl;
	// 	mLog.debug() << "Add " << -1 << " to " << mVirtualNodes[0]->matrixNodeIndex() << "," << matrixNodeIndex(0) << std::endl;
	// }
	// if (terminalNotGrounded(1)) {
	// 	mLog.debug() << "Add " << 1 << " to " << matrixNodeIndex(1) << "," << mVirtualNodes[0]->matrixNodeIndex() << std::endl;
	// 	mLog.debug() << "Add " << 1 << " to " << mVirtualNodes[0]->matrixNodeIndex() << "," << matrixNodeIndex(1) << std::endl;
	// }
}

void EMT::Ph3::VoltageSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mIntfVoltage(0, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mIntfVoltage(1, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mIntfVoltage(2, 0));
}

void EMT::Ph3::VoltageSource::updateVoltage(Real time) {
	if (mSrcFreq->get() < 0) {
		mIntfVoltage = RMS3PH_TO_PEAK1PH * mVoltageRef->get().real();
	}
	else {
		mIntfVoltage(0, 0) =
			RMS3PH_TO_PEAK1PH * Math::abs(mVoltageRef->get()(0, 0)) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())(0, 0));
		mIntfVoltage(1, 0) =
			RMS3PH_TO_PEAK1PH * Math::abs(mVoltageRef->get()(1, 0)) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())(1, 0));
		mIntfVoltage(2, 0) =
			RMS3PH_TO_PEAK1PH * Math::abs(mVoltageRef->get()(2, 0)) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())(2, 0));
	}

	mSLog->debug(
		"\nUpdate Voltage: {:s}",
		Logger::matrixToString(mIntfVoltage)
	);
}

void EMT::Ph3::VoltageSource::updateVoltage(Matrix vabc) {
	mIntfVoltage=vabc;
}

void EMT::Ph3::VoltageSource::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(attribute("V_ref"));
	modifiedAttributes.push_back(attribute("right_vector"));
	modifiedAttributes.push_back(attribute("v_intf"));
}

void EMT::Ph3::VoltageSource::mnaPreStep(Real time, Int timeStepCount) {
	updateVoltage(time);
	mnaApplyRightSideVectorStamp(mRightVector);
}

void EMT::Ph3::VoltageSource::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("i_intf"));
};

void EMT::Ph3::VoltageSource::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaUpdateCurrent(*leftVector);
}

void EMT::Ph3::VoltageSource::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
	mIntfCurrent(1, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
	mIntfCurrent(2, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
}
