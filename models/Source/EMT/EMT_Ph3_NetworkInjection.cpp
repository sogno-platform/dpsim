/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_NetworkInjection.h>

using namespace CPS;

EMT::Ph3::NetworkInjection::NetworkInjection(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel), TopologicalPowerComp(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(1);
	setTerminalNumber(1);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	mSLog->info("Create {} {}", this->type(), name);

	addAttribute<MatrixComp>("V_ref", Flags::read | Flags::write);
	addAttribute<Real>("f_src", Flags::read | Flags::write);
	mSLog->flush();
}

SimPowerComp<Real>::Ptr EMT::Ph3::NetworkInjection::clone(String name) {
	auto copy = NetworkInjection::make(name, mLogLevel);
	copy->setParameters(attribute<MatrixComp>("V_ref")->get());
	return copy;
}


void EMT::Ph3::NetworkInjection::setParameters(MatrixComp voltageRef, Real srcFreq) {
	attribute<MatrixComp>("V_ref")->set(voltageRef);
	attribute<Real>("f_src")->set(srcFreq);

	parametersSet = true;
}

void EMT::Ph3::NetworkInjection::initializeFromPowerflow(Real frequency) {
	mVoltageRef = attribute<MatrixComp>("V_ref");
	mSrcFreq = attribute<Real>("f_src");
	mSrcFreq->set(frequency);
	//if (mVoltageRef->) {
		MatrixComp vInitABC = MatrixComp::Zero(3, 1);
		vInitABC(0, 0) = initialSingleVoltage(0);
		vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
		vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

		//mVoltageRef->set(Complex(std::abs(initialSingleVoltage(0).real()), std::abs(initialSingleVoltage(0).imag())));
		mVoltageRef->set(vInitABC);
	//}
	mSLog->info(
		"\n--- Initialization from node voltages ---"
		"\nReference voltage: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from node voltages ---",
		Logger::matrixCompToString(mVoltageRef->get()),
		Logger::phasorToString(initialSingleVoltage(0)));
	mSLog->flush();
}

// #### MNA functions ####

void EMT::Ph3::NetworkInjection::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mIntfVoltage = mVoltageRef->get().real();
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

//void EMT::Ph3::NetworkInjection::mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) {
//	MNAInterface::mnaInitialize(omega, timeStep);
//	updateMatrixNodeIndices();
//
//	mIntfVoltage(0, 0) = mVoltageRef->get();
//
//	mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
//	mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
//	mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);
//}

void EMT::Ph3::NetworkInjection::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// TODO: error when setMatrixElement applied here
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0), 1);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), 1);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1), 1);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2), 1);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1);

	mSLog->info("-- Stamp ---");
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
	mSLog->flush();
}

//void EMT::Ph3::NetworkInjection::mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) {
//	Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0), Complex(1, 0));
//	Math::setMatrixElement(systemMatrix, matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0));
//	mSLog->info("-- Stamp frequency {:d} ---", freqIdx);
//	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex());
//	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0));
//}

void EMT::Ph3::NetworkInjection::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mIntfVoltage(0, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mIntfVoltage(1, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mIntfVoltage(2, 0));
	mSLog->debug("Add phase A {:f} to source vector at {:d}",
		"Add phase B {:f} to source vector at {:d}",
		"Add phase C {:f} to source vector at {:d}",
		mIntfVoltage(0, 0),
		mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
		mIntfVoltage(1, 0),
		mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
		mIntfVoltage(2, 0),
		mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
	mSLog->flush();
}

//void EMT::Ph3::NetworkInjection::mnaApplyRightSideVectorStampHarm(Matrix& rightVector) {
//	for (UInt freq = 0; freq < mNumFreqs; freq++) {
//		// TODO: Is this correct with two nodes not gnd?
//		Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(), mIntfVoltage(0, freq), 1, 0, freq);
//		SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
//			Logger::complexToString(mIntfVoltage(0, freq)), mVirtualNodes[0]->matrixNodeIndex());
//	}
//}

void EMT::Ph3::NetworkInjection::updateVoltage(Real time) {
	if (mSrcFreq->get() < 0) {
		mIntfVoltage = mVoltageRef->get().real();
	}
	else {
		mIntfVoltage(0, 0) =
			Math::abs(mVoltageRef->get()(0, 0)) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())(0, 0));
		mIntfVoltage(1, 0) =
			Math::abs(mVoltageRef->get()(1, 0)) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())(1, 0));
		mIntfVoltage(2, 0) =
			Math::abs(mVoltageRef->get()(2, 0)) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())(2, 0));
	}
}

void EMT::Ph3::NetworkInjection::MnaPreStep::execute(Real time, Int timeStepCount) {
	mNetworkInjection.updateVoltage(time);
	mNetworkInjection.mnaApplyRightSideVectorStamp(mNetworkInjection.mRightVector);
}

//void EMT::Ph3::NetworkInjection::MnaPreStepHarm::execute(Real time, Int timeStepCount) {
//	mNetworkInjection.updateVoltage(time);
//	mNetworkInjection.mnaApplyRightSideVectorStampHarm(mNetworkInjection.mRightVector);
//}

void EMT::Ph3::NetworkInjection::MnaPostStep::execute(Real time, Int timeStepCount) {
	mNetworkInjection.mnaUpdateCurrent(*mLeftVector);
}

//void EMT::Ph3::NetworkInjection::MnaPostStepHarm::execute(Real time, Int timeStepCount) {
//	mNetworkInjection.mnaUpdateCurrent(*mLeftVectors[0]);
//}

void EMT::Ph3::NetworkInjection::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
	mIntfCurrent(1, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
	mIntfCurrent(2, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
}

//void EMT::Ph3::NetworkInjection::daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off) {
//	/* new state vector definintion:
//		state[0]=node0_voltage
//		state[1]=node1_voltage
//		....
//		state[n]=noden_voltage
//		state[n+1]=component0_voltage
//		state[n+2]=component0_inductance (not yet implemented)
//		...
//		state[m-1]=componentm_voltage
//		state[m]=componentm_inductance
//	*/
//
//	int Pos1 = matrixNodeIndex(0);
//	int Pos2 = matrixNodeIndex(1);
//	int c_offset = off[0] + off[1]; //current offset for component
//	int n_offset_1 = c_offset + Pos1 + 1;// current offset for first nodal equation
//	int n_offset_2 = c_offset + Pos2 + 1;// current offset for second nodal equation
//	resid[c_offset] = (state[Pos2] - state[Pos1]) - state[c_offset]; // Voltage equation for Resistor
//	//resid[++c_offset] = ; //TODO : add inductance equation
//	resid[n_offset_1] += mIntfCurrent(0, 0).real();
//	resid[n_offset_2] += mIntfCurrent(0, 0).real();
//	off[1] += 1;
//}
//
//Complex EMT::Ph3::NetworkInjection::daeInitialize() {
//	mIntfVoltage(0, 0) = mVoltageRef->get();
//	return mVoltageRef->get();
//}
