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

#include <cps/DP/DP_Ph1_Inverter.h>
#include <cps/MathUtils.h>
#include <cmath>

using namespace CPS;
using namespace std;

DP::Ph1::Inverter::Inverter(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setTerminalNumber(1);
	setVirtualNodeNumber(1);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);
}

void DP::Ph1::Inverter::setParameters(std::vector<Int> carrierHarms, std::vector<Int> modulHarms,
	Real inputVoltage, Real ratio, Real phase) {
	mCarHarms = carrierHarms;
	mModHarms = modulHarms;
	mVin = inputVoltage;
	mModIdx = ratio;
	mPhaseMod = phase;
}

void DP::Ph1::Inverter::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();
	//mIntfVoltage(0,0) =  initialSingleVoltage(0));
}

void DP::Ph1::Inverter::generateFrequencies() {
	for (Int m = 2; m <= mMaxCarrierHarm; m = m+2) {
		mCarHarms.push_back(m);
		mSLog->info("Add carrier harmonic {0}", m);
	}
	for (Int n = -mMaxModulHarm; n <= mMaxModulHarm; n = n+2) {
		mModHarms.push_back(n);
		mSLog->info("Add modulation harmonic {0}", n);
	}
}

void DP::Ph1::Inverter::initialize(Matrix frequencies) {
	PowerComponent<Complex>::initialize(frequencies);

	mSLog->info("\n--- Initialization ---");

	// Check that both lists have the same length
	mCarHarNum = static_cast<UInt>(mCarHarms.size());
	mModHarNum = static_cast<UInt>(mModHarms.size());
	if (mCarHarNum != mModHarNum) {
		throw std::invalid_argument("Number of carrier and modulation harmonics must be equal.");
	}
	mHarNum = mCarHarNum;

	mPhasorMags = Matrix::Zero(mHarNum, 1);
	mPhasorPhases = Matrix::Zero(mHarNum, 1);
	mPhasorFreqs = Matrix::Zero(mHarNum, 1);

	for (UInt h = 0; h < mHarNum; h++ ) {
		mPhasorFreqs(h, 0) = mCarHarms[h]*mFreqCar + mModHarms[h]*mFreqMod;
		mPhasorPhases(h, 0) = mCarHarms[h]*mPhaseCar + mModHarms[h]*mPhaseMod;
	}

	// Precompute factorials
	for (Int f = 0; f <= mMaxBesselSumIdx; f++) {
		mFactorials.push_back(factorial(f));
	}

	for (UInt h = 0; h < mHarNum; h++) {
		for (Int f = 0; f <= mMaxBesselSumIdx; f++)
			mMultInvFactorials[f+mModHarms[h]] = multInvFactorial(f+mModHarms[h]);
	}

	mSLog->info(
		"\nFrequencies: \n{}"
		"\nPhases: \n{}"
		"\n--- End of initialization ---",
		mPhasorFreqs, mPhasorPhases);
}

void DP::Ph1::Inverter::calculatePhasors() {
	// Compute fundamental content of grid frequency
	mVfund = mModIdx * mVin;
	mIntfVoltage(0,0) = mVfund * -1i;

	// Compute sideband harmonics for even multiplies of carrier frequency m
	// and odd reference signal multiplies n
	for (UInt h = 0; h < mHarNum; h++ ) {
		Real Jn = besselFirstKind_n_opt(mModHarms[h], mMaxBesselSumIdx, mCarHarms[h]*mModIdx*PI/2.);
		//mPhasorMags(h, 0) = (4.*mVin/PI) * (Jn/mCarHarms[h]) * cos(mCarHarms[h] * PI/2.);
		//mIntfVoltage(0, h+1) = mPhasorMags(h, 0) * -1i;
		mIntfVoltage(0, h+1) = -1i * (4.*mVin/PI) * (Jn/mCarHarms[h]) * cos(mCarHarms[h] * PI/2.);
	}

	SPDLOG_LOGGER_DEBUG(mSLog,
		"\n--- Phasor calculation ---"
		"\n{}"
		"\n{}"
		"\n--- Phasor calculation end ---",
		mPhasorMags, mIntfVoltage);
}

// #### MNA functions ####

void DP::Ph1::Inverter::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	calculatePhasors();
}

void DP::Ph1::Inverter::mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
	mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);

	calculatePhasors();
}

void DP::Ph1::Inverter::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSLog->info("--- Stamping into system matrix ---");

	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mSLog->info("Stamp frequency {:d}", freq);
		if (terminalNotGrounded(0)) {
			Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(0), Complex(1, 0), mNumFreqs, freq);
			Math::setMatrixElement(systemMatrix, simNode(0), mVirtualNodes[0]->simNode(), Complex(1, 0), mNumFreqs, freq);
		}

		if (terminalNotGrounded(0)) {
			mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->simNode(), simNode(0));
			mSLog->info("Add {:f} to system at ({:d},{:d})", 1., simNode(0), mVirtualNodes[0]->simNode());
		}
	}
	mSLog->info("--- Stamping into system matrix end ---");
}

void DP::Ph1::Inverter::mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) {
	mSLog->info("Stamp frequency {:d}", freqIdx);
	if (terminalNotGrounded(0)) {
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(0), Complex(1, 0));
		Math::setMatrixElement(systemMatrix, simNode(0), mVirtualNodes[0]->simNode(), Complex(1, 0));
	}

	if (terminalNotGrounded(0)) {
		mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->simNode(), simNode(0));
		mSLog->info("Add {:f} to system at ({:d},{:d})", 1., simNode(0), mVirtualNodes[0]->simNode());
	}
}

void DP::Ph1::Inverter::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	SPDLOG_LOGGER_DEBUG(mSLog, "Stamp harmonics into source vector");
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		if (terminalNotGrounded(0)) {
			Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0,freq), mNumFreqs, freq);

			SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}, harmonic {:d}",
				Logger::complexToString(mIntfVoltage(0,freq)), mVirtualNodes[0]->simNode(), freq);
		}
	}
}

void DP::Ph1::Inverter::mnaApplyRightSideVectorStampHarm(Matrix& rightVector) {
	SPDLOG_LOGGER_DEBUG(mSLog, "Stamp harmonics into source vector");
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		if (terminalNotGrounded(0)) {
			Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0,freq), 1, 0, freq);
		}
	}
}

void DP::Ph1::Inverter::mnaApplyRightSideVectorStampHarm(Matrix& rightVector, Int freq) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0,freq));
}


void DP::Ph1::Inverter::MnaPreStep::execute(Real time, Int timeStepCount) {
	mInverter.calculatePhasors();
	mInverter.mnaApplyRightSideVectorStamp(mInverter.mRightVector);
}

void DP::Ph1::Inverter::MnaPreStepHarm::execute(Real time, Int timeStepCount) {
	mInverter.calculatePhasors();
	mInverter.mnaApplyRightSideVectorStampHarm(mInverter.mRightVector);
}

void DP::Ph1::Inverter::MnaPostStep::execute(Real time, Int timeStepCount) {

}

void DP::Ph1::Inverter::MnaPostStepHarm::execute(Real time, Int timeStepCount) {

}


