/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 *         Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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

#include <cps/EMT/EMT_Ph3_PiLine.h>
#define SHIFT_TO_PHASE_B Complex(cos(-2 * M_PI / 3), sin(-2 * M_PI / 3))
#define SHIFT_TO_PHASE_C Complex(cos(2 * M_PI / 3), sin(2 * M_PI / 3))

using namespace CPS;

EMT::Ph3::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	addAttribute<Matrix>("R_series", &mSeriesRes, Flags::read | Flags::write);
	addAttribute<Matrix>("L_series", &mSeriesInd, Flags::read | Flags::write);
	addAttribute<Matrix>("C_parallel", &mParallelCap, Flags::read | Flags::write);
	addAttribute<Matrix>("G_parallel", &mParallelCond, Flags::read | Flags::write);
}

PowerComponent<Real>::Ptr EMT::Ph3::PiLine::clone(String name) {
	auto copy = PiLine::make(name, mLogLevel);
	copy->setParameters(mSeriesRes, mSeriesInd, mParallelCap, mParallelCond);
	return copy;
}

void EMT::Ph3::PiLine::initialize(Matrix frequencies) {
	PowerComponent<Real>::initialize(frequencies);
}

void EMT::Ph3::PiLine::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	// By default there is always a small conductance to ground to
	// avoid problems with floating nodes.
	Matrix defaultParallelCond = Matrix::Zero(3, 3);
	defaultParallelCond <<
		1e-8, 0, 0,
		0, 1e-8, 0,
		0, 0, 1e-8;
	mParallelCond = (mParallelCond(0, 0) > 0) ? mParallelCond : defaultParallelCond;

	// Static calculation
	Real omega = 2. * PI * frequency;
	MatrixComp impedance = MatrixComp::Zero(3, 3);
	impedance <<
		Complex(mSeriesRes(0, 0), omega * mSeriesInd(0, 0)), Complex(mSeriesRes(0, 1), omega * mSeriesInd(0, 1)), Complex(mSeriesRes(0, 2), omega * mSeriesInd(0, 2)),
		Complex(mSeriesRes(1, 0), omega * mSeriesInd(1, 0)), Complex(mSeriesRes(1, 1), omega * mSeriesInd(1, 1)), Complex(mSeriesRes(1, 2), omega * mSeriesInd(1, 2)),
		Complex(mSeriesRes(2, 0), omega * mSeriesInd(2, 0)), Complex(mSeriesRes(2, 1), omega * mSeriesInd(2, 1)), Complex(mSeriesRes(2, 2), omega * mSeriesInd(2, 2));

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	MatrixComp iInit = impedance.inverse() * vInitABC;
	mIntfCurrent = iInit.real();
	mIntfVoltage = vInitABC.real();

	// Initialization of virtual node
	// Initial voltage of phase B,C is set after A
	MatrixComp vInitTerm0 = MatrixComp::Zero(3, 1);
	vInitTerm0(0, 0) = initialSingleVoltage(0);
	vInitTerm0(1, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_B;
	vInitTerm0(2, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_C;

	mVirtualNodes[0]->setInitialVoltage(vInitTerm0 + mSeriesRes * iInit);

	// Create series sub components
	mSubSeriesResistor = std::make_shared<EMT::Ph3::Resistor>(mName + "_res", mLogLevel);
	mSubSeriesResistor->setParameters(mSeriesRes);
	mSubSeriesResistor->connect({ mTerminals[0]->node(), mVirtualNodes[0] });
	mSubSeriesResistor->initialize(mFrequencies);
	mSubSeriesResistor->initializeFromPowerflow(frequency);

	mSubSeriesInductor = std::make_shared<EMT::Ph3::Inductor>(mName + "_ind", mLogLevel);
	mSubSeriesInductor->setParameters(mSeriesInd);
	mSubSeriesInductor->connect({ mVirtualNodes[0], mTerminals[1]->node() });
	mSubSeriesInductor->initialize(mFrequencies);
	mSubSeriesInductor->initializeFromPowerflow(frequency);

	// Create parallel sub components
	if (mParallelCond(0,0) > 0) {
		mSubParallelResistor0 = std::make_shared<EMT::Ph3::Resistor>(mName + "_con0", mLogLevel);
		mSubParallelResistor0->setParameters(2. * mParallelCond.inverse());
		mSubParallelResistor0->connect(Node::List{ Node::GND, mTerminals[0]->node() });
		mSubParallelResistor0->initialize(mFrequencies);
		mSubParallelResistor0->initializeFromPowerflow(frequency);

		mSubParallelResistor1 = std::make_shared<EMT::Ph3::Resistor>(mName + "_con1", mLogLevel);
		mSubParallelResistor1->setParameters(2. * mParallelCond.inverse());
		mSubParallelResistor1->connect(Node::List{ Node::GND, mTerminals[1]->node() });
		mSubParallelResistor1->initialize(mFrequencies);
		mSubParallelResistor1->initializeFromPowerflow(frequency);
	}

	if (mParallelCap(0,0) > 0) {
		mSubParallelCapacitor0 = std::make_shared<EMT::Ph3::Capacitor>(mName + "_cap0", mLogLevel);
		mSubParallelCapacitor0->setParameters(mParallelCap / 2.);
		mSubParallelCapacitor0->connect(Node::List{ Node::GND, mTerminals[0]->node() });
		mSubParallelCapacitor0->initialize(mFrequencies);
		mSubParallelCapacitor0->initializeFromPowerflow(frequency);

		mSubParallelCapacitor1 = std::make_shared<EMT::Ph3::Capacitor>(mName + "_cap1", mLogLevel);
		mSubParallelCapacitor1->setParameters(mParallelCap / 2.);
		mSubParallelCapacitor1->connect(Node::List{ Node::GND, mTerminals[1]->node() });
		mSubParallelCapacitor1->initialize(mFrequencies);
		mSubParallelCapacitor1->initializeFromPowerflow(frequency);
	}

	mSLog->debug(
		"\n--debug--"
		"\n seriesRes: {:s}"
		"\n seriesInd: {:s}"
		"\n Impedance: {:s}"
		"\n vInit: {:s}"
		"\n iInit: {:s}",
		Logger::matrixToString(mSeriesRes),
		Logger::matrixToString(mSeriesInd),
		Logger::matrixCompToString(impedance),
		Logger::matrixCompToString(vInitABC),
		Logger::matrixCompToString(iInit));

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
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

void EMT::Ph3::PiLine::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	MNAInterface::List subComps({ mSubSeriesResistor, mSubSeriesInductor });

	mSubSeriesResistor->mnaInitialize(omega, timeStep, leftVector);
	mSubSeriesInductor->mnaInitialize(omega, timeStep, leftVector);
	mRightVectorStamps.push_back(&mSubSeriesInductor->attribute<Matrix>("right_vector")->get());
	if (mParallelCond(0,0) > 0) {
		mSubParallelResistor0->mnaInitialize(omega, timeStep, leftVector);
		mSubParallelResistor1->mnaInitialize(omega, timeStep, leftVector);
		subComps.push_back(mSubParallelResistor0);
		subComps.push_back(mSubParallelResistor1);
	}
	if (mParallelCap(0,0) >= 0) {
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

void EMT::Ph3::PiLine::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubSeriesResistor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSeriesInductor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mParallelCond(0,0) >= 0) {
		mSubParallelResistor0->mnaApplySystemMatrixStamp(systemMatrix);
		mSubParallelResistor1->mnaApplySystemMatrixStamp(systemMatrix);
	}
	if (mParallelCap(0,0) >= 0) {
		mSubParallelCapacitor0->mnaApplySystemMatrixStamp(systemMatrix);
		mSubParallelCapacitor1->mnaApplySystemMatrixStamp(systemMatrix);
	}
}

void EMT::Ph3::PiLine::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;
}

void EMT::Ph3::PiLine::MnaPreStep::execute(Real time, Int timeStepCount) {
	mLine.mnaApplyRightSideVectorStamp(mLine.mRightVector);
}

void EMT::Ph3::PiLine::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLine.mnaUpdateVoltage(*mLeftVector);
	mLine.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::PiLine::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage = Matrix::Zero(3, 1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, simNode(1, 0));
		mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, simNode(1, 1));
		mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, simNode(1, 2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::realFromVectorElement(leftVector, simNode(0, 0));
		mIntfVoltage(1, 0) = mIntfVoltage(1, 0) - Math::realFromVectorElement(leftVector, simNode(0, 1));
		mIntfVoltage(2, 0) = mIntfVoltage(2, 0) - Math::realFromVectorElement(leftVector, simNode(0, 2));
	}
}

void EMT::Ph3::PiLine::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mSubSeriesInductor->intfCurrent();
}
//
//MNAInterface::List EMT::Ph3::PiLine::mnaTearGroundComponents() {
//	MNAInterface::List gndComponents;
//
//	if (mParallelCond(0,0) > 0) {
//		gndComponents.push_back(mSubParallelResistor0);
//		gndComponents.push_back(mSubParallelResistor1);
//	}
//
//	if (mParallelCap(0,0) > 0) {
//		gndComponents.push_back(mSubParallelCapacitor0);
//		gndComponents.push_back(mSubParallelCapacitor1);
//	}
//
//	return gndComponents;
//}
//
//void EMT::Ph3::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
//	mSubSeriesResistor->mnaTearSetIdx(mTearIdx);
//	mSubSeriesResistor->mnaTearInitialize(omega, timeStep);
//	mSubSeriesInductor->mnaTearSetIdx(mTearIdx);
//	mSubSeriesInductor->mnaTearInitialize(omega, timeStep);
//}
//
//void EMT::Ph3::PiLine::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
//	mSubSeriesResistor->mnaTearApplyMatrixStamp(tearMatrix);
//	mSubSeriesInductor->mnaTearApplyMatrixStamp(tearMatrix);
//}
//
//void EMT::Ph3::PiLine::mnaTearApplyVoltageStamp(Matrix& voltageVector) {
//	mSubSeriesInductor->mnaTearApplyVoltageStamp(voltageVector);
//}
//
//void EMT::Ph3::PiLine::mnaTearPostStep(Complex voltage, Complex current) {
//	mSubSeriesInductor->mnaTearPostStep(voltage - current * mSeriesRes, current);
//}
