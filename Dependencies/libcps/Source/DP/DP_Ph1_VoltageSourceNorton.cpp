/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/DP/DP_Ph1_VoltageSourceNorton.h>

using namespace CPS;

DP::Ph1::VoltageSourceNorton::VoltageSourceNorton(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Complex>("V_ref", &mVoltageRef, Flags::read | Flags::write);
	addAttribute<Real>("R", &mResistance, Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr DP::Ph1::VoltageSourceNorton::clone(String name) {
	auto copy = VoltageSourceNorton::make(name, mLogLevel);
	copy->setParameters(mVoltageRef, mSrcFreq, mResistance);
	return copy;
}

void DP::Ph1::VoltageSourceNorton::setParameters(Complex voltage, Real srcFreq, Real resistance) {
	Base::Ph1::VoltageSource::setParameters(voltage, srcFreq);

	mResistance = resistance;
	mConductance = 1. / mResistance;
	mEquivCurrent = mVoltageRef / mResistance;
}

void DP::Ph1::VoltageSourceNorton::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mIntfVoltage(0, 0) = attributeComplex("V_ref")->get();
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void DP::Ph1::VoltageSourceNorton::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Apply matrix stamp for equivalent resistance
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(0), Complex(mConductance, 0));
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(1), Complex(mConductance, 0));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(1), Complex(-mConductance, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(0), Complex(-mConductance, 0));
	}

	if (terminalNotGrounded(0))
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(mConductance, 0)),
			simNode(0), simNode(0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(mConductance, 0)),
			simNode(1), simNode(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-mConductance, 0)),
			simNode(0), simNode(1));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-mConductance, 0)),
			simNode(1), simNode(0));
	}
}

void DP::Ph1::VoltageSourceNorton::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mEquivCurrent = mIntfVoltage(0, 0) / mResistance;

	// Apply matrix stamp for equivalent current source
	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, simNode(0), -mEquivCurrent);
	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, simNode(1), mEquivCurrent);
}

void DP::Ph1::VoltageSourceNorton::updateState(Real time) {
	if (mSrcFreq >= 0) {
		mIntfVoltage(0,0) = Complex(
			Math::abs(mVoltageRef) * cos(time * 2.*PI*mSrcFreq + Math::phase(mVoltageRef)),
			Math::abs(mVoltageRef) * sin(time * 2.*PI*mSrcFreq + Math::phase(mVoltageRef)));
	}
	else {
		// If source frequency -1, use system frequency.
		mIntfVoltage(0,0) = mVoltageRef;
	}
}

void DP::Ph1::VoltageSourceNorton::MnaPreStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.updateState(time);
	mVoltageSource.mnaApplyRightSideVectorStamp(mVoltageSource.mRightVector);
}

void DP::Ph1::VoltageSourceNorton::MnaPostStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.mnaUpdateVoltage(*mLeftVector);
	mVoltageSource.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph1::VoltageSourceNorton::mnaUpdateVoltage(const Matrix& leftVector) {
	// Calculate v1 - v0
	mIntfVoltage(0, 0) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, simNode(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, simNode(0));
}

void DP::Ph1::VoltageSourceNorton::mnaUpdateCurrent(const Matrix& leftVector) {
	// TODO: verify signs
	mIntfCurrent(0,0) = mEquivCurrent - mIntfVoltage(0,0) / mResistance;
}

