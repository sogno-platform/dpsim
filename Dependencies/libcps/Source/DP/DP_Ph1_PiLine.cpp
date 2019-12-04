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

#include <cps/DP/DP_Ph1_PiLine.h>

using namespace CPS;

DP::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Real>("R_series", &mSeriesRes, Flags::read | Flags::write);
	addAttribute<Real>("L_series", &mSeriesInd, Flags::read | Flags::write);
	addAttribute<Real>("C_parallel", &mParallelCap, Flags::read | Flags::write);
	addAttribute<Real>("G_parallel", &mParallelCond, Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr DP::Ph1::PiLine::clone(String name) {
	auto copy = PiLine::make(name, mLogLevel);
	copy->setParameters(mSeriesRes, mSeriesInd, mParallelCap, mParallelCond);
	return copy;
}

void DP::Ph1::PiLine::initialize(Matrix frequencies) {
	PowerComponent<Complex>::initialize(frequencies);
}

void DP::Ph1::PiLine::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	// By default there is always a small conductance to ground to
	// avoid problems with floating nodes.
	mParallelCond = (mParallelCond >= 0) ? mParallelCond : 1e-6;

	// Static calculation
	Real omega = 2.*PI * frequency;
	Complex impedance = { mSeriesRes, omega * mSeriesInd };
	mIntfVoltage(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / impedance;

	// Initialization of virtual node
	mVirtualNodes[0]->setInitialVoltage( initialSingleVoltage(0) + mIntfCurrent(0,0) * mSeriesRes );

	// Create series sub components
	mSubSeriesResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_res", mLogLevel);
	mSubSeriesResistor->setParameters(mSeriesRes);
	mSubSeriesResistor->connect({ mTerminals[0]->node(), mVirtualNodes[0] });
	mSubSeriesResistor->initialize(mFrequencies);
	mSubSeriesResistor->initializeFromPowerflow(frequency);

	mSubSeriesInductor = std::make_shared<DP::Ph1::Inductor>(mName + "_ind", mLogLevel);
	mSubSeriesInductor->setParameters(mSeriesInd);
	mSubSeriesInductor->connect({ mVirtualNodes[0], mTerminals[1]->node() });
	mSubSeriesInductor->initialize(mFrequencies);
	mSubSeriesInductor->initializeFromPowerflow(frequency);

	// Create parallel sub components
	if (mParallelCond >= 0) {
		mSubParallelResistor0 = std::make_shared<DP::Ph1::Resistor>(mName + "_con0", mLogLevel);
		mSubParallelResistor0->setParameters(2./mParallelCond);
		mSubParallelResistor0->connect(Node::List{ Node::GND, mTerminals[0]->node() });
		mSubParallelResistor0->initialize(mFrequencies);
		mSubParallelResistor0->initializeFromPowerflow(frequency);

		mSubParallelResistor1 = std::make_shared<DP::Ph1::Resistor>(mName + "_con1", mLogLevel);
		mSubParallelResistor1->setParameters(2./mParallelCond);
		mSubParallelResistor1->connect(Node::List{ Node::GND, mTerminals[1]->node() });
		mSubParallelResistor1->initialize(mFrequencies);
		mSubParallelResistor1->initializeFromPowerflow(frequency);
	}

	if (mParallelCap >= 0) {
		mSubParallelCapacitor0 = std::make_shared<DP::Ph1::Capacitor>(mName + "_cap0", mLogLevel);
		mSubParallelCapacitor0->setParameters(mParallelCap/2.);
		mSubParallelCapacitor0->connect(Node::List{ Node::GND, mTerminals[0]->node() });
		mSubParallelCapacitor0->initialize(mFrequencies);
		mSubParallelCapacitor0->initializeFromPowerflow(frequency);

		mSubParallelCapacitor1 = std::make_shared<DP::Ph1::Capacitor>(mName + "_cap1", mLogLevel);
		mSubParallelCapacitor1->setParameters(mParallelCap/2.);
		mSubParallelCapacitor1->connect(Node::List{ Node::GND, mTerminals[1]->node() });
		mSubParallelCapacitor1->initialize(mFrequencies);
		mSubParallelCapacitor1->initializeFromPowerflow(frequency);
	}

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\nVirtual Node 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

void DP::Ph1::PiLine::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	MNAInterface::List subComps({mSubSeriesResistor, mSubSeriesInductor});

	mSubSeriesResistor->mnaInitialize(omega, timeStep, leftVector);
	mSubSeriesInductor->mnaInitialize(omega, timeStep, leftVector);
	mRightVectorStamps.push_back(&mSubSeriesInductor->attribute<Matrix>("right_vector")->get());
	if (mParallelCond >= 0) {
		mSubParallelResistor0->mnaInitialize(omega, timeStep, leftVector);
		mSubParallelResistor1->mnaInitialize(omega, timeStep, leftVector);
		subComps.push_back(mSubParallelResistor0);
		subComps.push_back(mSubParallelResistor1);
	}
	if (mParallelCap >= 0) {
		mSubParallelCapacitor0->mnaInitialize(omega, timeStep, leftVector);
		mSubParallelCapacitor1->mnaInitialize(omega, timeStep, leftVector);
		mRightVectorStamps.push_back(&mSubParallelCapacitor0->attribute<Matrix>("right_vector")->get());
		mRightVectorStamps.push_back(&mSubParallelCapacitor1->attribute<Matrix>("right_vector")->get());
		subComps.push_back(mSubParallelCapacitor0);
		subComps.push_back(mSubParallelCapacitor1);
	}
	for (auto comp : subComps) {
		for (auto task : comp->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::PiLine::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubSeriesResistor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSeriesInductor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mParallelCond >= 0) {
		mSubParallelResistor0->mnaApplySystemMatrixStamp(systemMatrix);
		mSubParallelResistor1->mnaApplySystemMatrixStamp(systemMatrix);
	}
	if (mParallelCap >= 0) {
		mSubParallelCapacitor0->mnaApplySystemMatrixStamp(systemMatrix);
		mSubParallelCapacitor1->mnaApplySystemMatrixStamp(systemMatrix);
	}
}

void DP::Ph1::PiLine::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;
}

void DP::Ph1::PiLine::MnaPreStep::execute(Real time, Int timeStepCount) {
	mLine.mnaApplyRightSideVectorStamp(mLine.mRightVector);
}

void DP::Ph1::PiLine::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLine.mnaUpdateVoltage(*mLeftVector);
	mLine.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph1::PiLine::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0, 0) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, simNode(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, simNode(0));
}

void DP::Ph1::PiLine::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0,0) = mSubSeriesInductor->intfCurrent()(0, 0);
}

MNAInterface::List DP::Ph1::PiLine::mnaTearGroundComponents() {
	MNAInterface::List gndComponents;

	if (mParallelCond >= 0) {
		gndComponents.push_back(mSubParallelResistor0);
		gndComponents.push_back(mSubParallelResistor1);
	}

	if (mParallelCap >= 0) {
		gndComponents.push_back(mSubParallelCapacitor0);
		gndComponents.push_back(mSubParallelCapacitor1);
	}

	return gndComponents;
}

void DP::Ph1::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
	mSubSeriesResistor->mnaTearSetIdx(mTearIdx);
	mSubSeriesResistor->mnaTearInitialize(omega, timeStep);
	mSubSeriesInductor->mnaTearSetIdx(mTearIdx);
	mSubSeriesInductor->mnaTearInitialize(omega, timeStep);
}

void DP::Ph1::PiLine::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
	mSubSeriesResistor->mnaTearApplyMatrixStamp(tearMatrix);
	mSubSeriesInductor->mnaTearApplyMatrixStamp(tearMatrix);
}

void DP::Ph1::PiLine::mnaTearApplyVoltageStamp(Matrix& voltageVector) {
	mSubSeriesInductor->mnaTearApplyVoltageStamp(voltageVector);
}

void DP::Ph1::PiLine::mnaTearPostStep(Complex voltage, Complex current) {
	mSubSeriesInductor->mnaTearPostStep(voltage - current * mSeriesRes, current);
}
