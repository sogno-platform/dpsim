/**
 * @file
 * @author Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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

#include <cps/DP/DP_Ph1_NetworkInjection.h>

using namespace CPS;

DP::Ph1::NetworkInjection::NetworkInjection(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(1);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Complex>("V_ref", Flags::read | Flags::write);
	addAttribute<Real>("f_src", Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr DP::Ph1::NetworkInjection::clone(String name) {
	auto copy = NetworkInjection::make(name, mLogLevel);
	copy->setParameters(attribute<Complex>("V_ref")->get());
	return copy;
}

void DP::Ph1::NetworkInjection::initialize(Matrix frequencies) {
	checkForUnconnectedTerminals();

	mFrequencies = frequencies;
	mNumFreqs = static_cast<UInt>(mFrequencies.size());

	mIntfVoltage = MatrixComp::Zero(1, mNumFreqs);
	mIntfCurrent = MatrixComp::Zero(1, mNumFreqs);
}

void DP::Ph1::NetworkInjection::setParameters(Complex voltageRef, Real srcFreq) {
	attribute<Complex>("V_ref")->set(voltageRef);
	attribute<Real>("f_src")->set(srcFreq);
}

void DP::Ph1::NetworkInjection::initializeFromPowerflow(Real frequency) {
	mVoltageRef = attribute<Complex>("V_ref");
	mSrcFreq = attribute<Real>("f_src");
	if (mVoltageRef->get() == Complex(0, 0))
		//mVoltageRef->set(Complex(std::abs(initialSingleVoltage(0).real()), std::abs(initialSingleVoltage(0).imag())));
		mVoltageRef->set(initialSingleVoltage(0));
	mSLog->info(
		"\n--- Initialization from node voltages ---"
		"\nVoltage across: {:e}<{:e}"
		"\nTerminal 0 voltage: {:e}<{:e}"
		"\n--- Initialization from node voltages ---",
		std::abs(mVoltageRef->get()), std::arg(mVoltageRef->get()),
		std::abs(initialSingleVoltage(0)), std::arg(initialSingleVoltage(0)));
}

// #### MNA functions ####

void DP::Ph1::NetworkInjection::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mIntfVoltage(0,0) = mVoltageRef->get();
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::NetworkInjection::mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mIntfVoltage(0,0) = mVoltageRef->get();

	mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
	mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);
}

void DP::Ph1::NetworkInjection::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
			Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(0), Complex(1, 0), mNumFreqs, freq);
			Math::setMatrixElement(systemMatrix, simNode(0), mVirtualNodes[0]->simNode(), Complex(1, 0), mNumFreqs, freq);
		mSLog->info("-- Stamp frequency {:d} ---", freq);
			mSLog->info("Add {:f} to system at ({:d},{:d})", 1., simNode(0), mVirtualNodes[0]->simNode());
			mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->simNode(), simNode(0));
	}
}

void DP::Ph1::NetworkInjection::mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) {
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(0), Complex(1, 0));
		Math::setMatrixElement(systemMatrix, simNode(0), mVirtualNodes[0]->simNode(), Complex(1, 0));
	mSLog->info("-- Stamp frequency {:d} ---", freqIdx);
		mSLog->info("Add {:f} to system at ({:d},{:d})", 1., simNode(0), mVirtualNodes[0]->simNode());
		mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->simNode(), simNode(0));
}

void DP::Ph1::NetworkInjection::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// TODO: Is this correct with two nodes not gnd?
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0,0), mNumFreqs);
	SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
		Logger::complexToString(mIntfVoltage(0,0)), mVirtualNodes[0]->simNode());
}

void DP::Ph1::NetworkInjection::mnaApplyRightSideVectorStampHarm(Matrix& rightVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		// TODO: Is this correct with two nodes not gnd?
		Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0,freq), 1, 0, freq);
		SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
			Logger::complexToString(mIntfVoltage(0,freq)), mVirtualNodes[0]->simNode());
	}
}

void DP::Ph1::NetworkInjection::updateVoltage(Real time) {
	if (mSrcFreq->get() < 0) {
		mIntfVoltage(0,0) = mVoltageRef->get();
	}
	else {
		mIntfVoltage(0,0) = Complex(
			Math::abs(mVoltageRef->get()) * cos(time * 2.*PI*mSrcFreq->get() + Math::phase(mVoltageRef->get())),
			Math::abs(mVoltageRef->get()) * sin(time * 2.*PI*mSrcFreq->get() + Math::phase(mVoltageRef->get())));
	}
}

void DP::Ph1::NetworkInjection::MnaPreStep::execute(Real time, Int timeStepCount) {
	mNetworkInjection.updateVoltage(time);
	mNetworkInjection.mnaApplyRightSideVectorStamp(mNetworkInjection.mRightVector);
}

void DP::Ph1::NetworkInjection::MnaPreStepHarm::execute(Real time, Int timeStepCount) {
	mNetworkInjection.updateVoltage(time);
	mNetworkInjection.mnaApplyRightSideVectorStampHarm(mNetworkInjection.mRightVector);
}

void DP::Ph1::NetworkInjection::MnaPostStep::execute(Real time, Int timeStepCount) {
	mNetworkInjection.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph1::NetworkInjection::MnaPostStepHarm::execute(Real time, Int timeStepCount) {
	mNetworkInjection.mnaUpdateCurrent(*mLeftVectors[0]);
}

void DP::Ph1::NetworkInjection::mnaUpdateCurrent(const Matrix& leftVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfCurrent(0,freq) = Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->simNode(), mNumFreqs, freq);
	}
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

    int Pos1 = simNode(0);
    int Pos2 = simNode(1);
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
	mIntfVoltage(0,0) = mVoltageRef->get();
	return mVoltageRef->get();
}
