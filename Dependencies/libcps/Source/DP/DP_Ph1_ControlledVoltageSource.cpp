/**
 * @file
 * @author  Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_ControlledVoltageSource.h>

using namespace CPS;

DP::Ph1::ControlledVoltageSource::ControlledVoltageSource(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);
}

void DP::Ph1::ControlledVoltageSource::setParameters(MatrixComp voltageRefABC) {
	mIntfVoltage = voltageRefABC;
}

PowerComponent<Complex>::Ptr DP::Ph1::ControlledVoltageSource::clone(String name) {
	auto copy = ControlledVoltageSource::make(name, mLogLevel);
	copy->setParameters(attribute<MatrixComp>("v_intf")->get());
	return copy;
}


void DP::Ph1::ControlledVoltageSource::initialize(Matrix frequencies) {

	mFrequencies = frequencies;
	mNumFreqs = static_cast<UInt>(mFrequencies.size());

	mIntfVoltage = MatrixComp::Zero(1, mNumFreqs);
	mIntfCurrent = MatrixComp::Zero(1, mNumFreqs);
}


void DP::Ph1::ControlledVoltageSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	checkForUnconnectedTerminals();

	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::ControlledVoltageSource::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		if (terminalNotGrounded(0)) {
			Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(0), Complex(-1, 0), mNumFreqs, freq);
			Math::setMatrixElement(systemMatrix, simNode(0), mVirtualNodes[0]->simNode(), Complex(-1, 0), mNumFreqs, freq);
		}
		if (terminalNotGrounded(1)) {
			Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(1), Complex(1, 0), mNumFreqs, freq);
			Math::setMatrixElement(systemMatrix, simNode(1), mVirtualNodes[0]->simNode(), Complex(1, 0), mNumFreqs, freq);
		}

		mSLog->info("-- Stamp frequency {:d} ---", freq);
		if (terminalNotGrounded(0)) {
			mSLog->info("Add {:f} to system at ({:d},{:d})", -1., simNode(0), mVirtualNodes[0]->simNode());
			mSLog->info("Add {:f} to system at ({:d},{:d})", -1., mVirtualNodes[0]->simNode(), simNode(0));
		}
		if (terminalNotGrounded(1)) {
			mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->simNode(), simNode(1));
			mSLog->info("Add {:f} to system at ({:d},{:d})", 1., simNode(1), mVirtualNodes[0]->simNode());
		}
	}
}

void DP::Ph1::ControlledVoltageSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0, 0), mNumFreqs);
	SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
		Logger::complexToString(mIntfVoltage(0, 0)), mVirtualNodes[0]->simNode());
}

void DP::Ph1::ControlledVoltageSource::MnaPreStep::execute(Real time, Int timeStepCount) {
	mControlledVoltageSource.mnaApplyRightSideVectorStamp(mControlledVoltageSource.mRightVector);
}

void DP::Ph1::ControlledVoltageSource::MnaPostStep::execute(Real time, Int timeStepCount) {
	mControlledVoltageSource.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph1::ControlledVoltageSource::mnaUpdateCurrent(const Matrix& leftVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfCurrent(0, freq) = Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->simNode(), mNumFreqs, freq);
	}
}
