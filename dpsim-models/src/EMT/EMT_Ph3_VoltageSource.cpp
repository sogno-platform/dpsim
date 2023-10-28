/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_VoltageSource.h>


using namespace CPS;

EMT::Ph3::VoltageSource::VoltageSource(String uid, String name, Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, true, true, logLevel),
	mVoltageRef(mAttributes->createDynamic<MatrixComp>("V_ref")), // rms-value, phase-to-phase
	mSrcFreq(mAttributes->createDynamic<Real>("f_src")) {
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
}

void EMT::Ph3::VoltageSource::setParameters(MatrixComp voltageRef, Real srcFreq) {
	auto srcSigSine = Signal::SineWaveGenerator::make(**mName + "_sw");
	// Complex(1,0) is used as initialPhasor, since magnitude and phase of V_ref are taken into account by updateVoltage
	srcSigSine->mFreq->setReference(mSrcFreq);
	srcSigSine->setParameters(Complex(1,0), srcFreq);
	mSrcSig = srcSigSine;

	**mVoltageRef = voltageRef;

	SPDLOG_LOGGER_INFO(mSLog, "\nVoltage reference phasor [V]: {:s}"
				"\nFrequency [Hz]: {:s}",
				Logger::matrixCompToString(voltageRef),
				Logger::realToString(srcFreq));

	mParametersSet = true;
}

void EMT::Ph3::VoltageSource::setParameters(MatrixComp voltageRef, Real freqStart, Real rocof, Real timeStart, Real duration, bool smoothRamp) {
	auto srcSigFreqRamp = Signal::FrequencyRampGenerator::make(**mName + "_fr");
	srcSigFreqRamp->mFreq->setReference(mSrcFreq);
	// Complex(1,0) is used as initialPhasor, since magnitude and phase of V_ref are taken into account by updateVoltage
	srcSigFreqRamp->setParameters(Complex(1,0), freqStart, rocof, timeStart, duration, smoothRamp);
	mSrcSig = srcSigFreqRamp;

	**mVoltageRef = voltageRef;

	mParametersSet = true;
}

void EMT::Ph3::VoltageSource::setParameters(MatrixComp voltageRef, Real modulationFrequency, Real modulationAmplitude, Real baseFrequency /*= 0.0*/, bool zigzag /*= false*/) {
    auto srcSigFm = Signal::CosineFMGenerator::make(**mName + "_fm");
	// Complex(1,0) is used as initialPhasor, since magnitude and phase of V_ref are taken into account by updateVoltage
	srcSigFm->mFreq->setReference(mSrcFreq);
	srcSigFm->setParameters(Complex(1,0), modulationFrequency, modulationAmplitude, baseFrequency, zigzag);
	mSrcSig = srcSigFm;

	**mVoltageRef = voltageRef;

	mParametersSet = true;
}

void EMT::Ph3::VoltageSource::initializeFromNodesAndTerminals(Real frequency) {
	SPDLOG_LOGGER_INFO(mSLog, "\n--- Initialization from node voltages ---");
	// TODO: this approach currently overwrites voltage reference set from outside, when not using setParameters
	if (!mParametersSet) {
		auto srcSigSine = Signal::SineWaveGenerator::make(**mName + "_sw");
		srcSigSine->mFreq->setReference(mSrcFreq);
		// Complex(1,0) is used as initialPhasor for signal generator as only phase is used
		srcSigSine->setParameters(Complex(1,0), frequency);
		mSrcSig = srcSigSine;

		**mVoltageRef = CPS::Math::singlePhaseVariableToThreePhase(initialSingleVoltage(1) - initialSingleVoltage(0));

		SPDLOG_LOGGER_INFO(mSLog, "\nReference voltage: {:s}"
					"\nTerminal 0 voltage: {:s}"
					"\nTerminal 1 voltage: {:s}",
					Logger::matrixCompToString(**mVoltageRef),
					Logger::phasorToString(initialSingleVoltage(0)),
					Logger::phasorToString(initialSingleVoltage(1)));
	} else {
		SPDLOG_LOGGER_INFO(mSLog, "\nInitialization from node voltages omitted (parameter already set)."
					"\nReference voltage: {:s}",
					Logger::matrixCompToString(**mVoltageRef));
	}
	SPDLOG_LOGGER_INFO(mSLog, "\n--- Initialization from node voltages ---");
	mSLog->flush();
}

SimPowerComp<Real>::Ptr EMT::Ph3::VoltageSource::clone(String name) {
	auto copy = VoltageSource::make(name, mLogLevel);
	copy->setParameters(**mVoltageRef, **mSrcFreq);
	return copy;
}


void EMT::Ph3::VoltageSource::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();

	mTimeStep = timeStep;
}

void EMT::Ph3::VoltageSource::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
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

void EMT::Ph3::VoltageSource::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), (**mIntfVoltage)(0, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), (**mIntfVoltage)(1, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), (**mIntfVoltage)(2, 0));
}

void EMT::Ph3::VoltageSource::updateVoltage(Real time) {
	if(mSrcSig != nullptr) {
		mSrcSig->step(time);
		for(int i = 0; i < 3; i++) {
			(**mIntfVoltage)(i, 0) = RMS3PH_TO_PEAK1PH * Math::abs((**mVoltageRef)(i, 0))
				* cos(Math::phase(mSrcSig->getSignal()) + Math::phase((**mVoltageRef)(i, 0)));
		}
	} else {
		**mIntfVoltage = RMS3PH_TO_PEAK1PH * (**mVoltageRef).real();
	}
	SPDLOG_LOGGER_DEBUG(mSLog,
		"\nUpdate Voltage: {:s}",
		Logger::matrixToString(**mIntfVoltage)
	);
}

void EMT::Ph3::VoltageSource::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mVoltageRef);
	modifiedAttributes.push_back(mRightVector);
	modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph3::VoltageSource::mnaCompPreStep(Real time, Int timeStepCount) {
	updateVoltage(time);
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::VoltageSource::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfCurrent);
};

void EMT::Ph3::VoltageSource::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::VoltageSource::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
	(**mIntfCurrent)(1, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
	(**mIntfCurrent)(2, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
}
