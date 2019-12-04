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

#include <cps/DP/DP_Ph1_Transformer.h>

using namespace CPS;

DP::Ph1::Transformer::Transformer(String uid, String name,
	Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setTerminalNumber(2);
	setVirtualNodeNumber(1);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Complex>("ratio", &mRatio, Flags::write | Flags::read);
	addAttribute<Real>("R", &mResistance, Flags::write | Flags::read);
	addAttribute<Real>("L", &mInductance, Flags::write | Flags::read);
}

PowerComponent<Complex>::Ptr DP::Ph1::Transformer::clone(String name) {
	auto copy = Transformer::make(name, mLogLevel);
	copy->setParameters(std::abs(mRatio), std::arg(mRatio), mResistance, mInductance);
	return copy;
}

void DP::Ph1::Transformer::setParameters(Real ratioAbs, Real ratioPhase,
	Real resistance, Real inductance) {
	Base::Ph1::Transformer::setParameters(ratioAbs, ratioPhase, resistance, inductance);

	if (resistance > 0)
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);
}

void DP::Ph1::Transformer::initialize(Matrix frequencies) {
	PowerComponent<Complex>::initialize(frequencies);
}

void DP::Ph1::Transformer::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	// A snubber conductance is added on the low voltage side
	Real snubberResistance = 1e3;

	// Component parameters are referred to high voltage side.
	// Switch terminals if transformer is connected the other way around.
	if (Math::abs(mRatio) < 1.) {
		mRatio = 1. / mRatio;
		std::shared_ptr<Terminal<Complex>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
	}

	// Set initial voltage of virtual node in between
	mVirtualNodes[0]->setInitialVoltage( initialSingleVoltage(1) * mRatio );

	// Static calculations from load flow data
	Real omega = 2.*PI* frequency;
	Complex impedance = { mResistance, omega * mInductance };
	mIntfVoltage(0,0) = mVirtualNodes[0]->initialSingleVoltage() - initialSingleVoltage(0);
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / impedance;

	// Create series sub components
	mSubInductor = std::make_shared<DP::Ph1::Inductor>(mName + "_ind", mLogLevel);
	mSubInductor->setParameters(mInductance);

	if (mNumVirtualNodes == 3) {
		mVirtualNodes[2]->setInitialVoltage(initialSingleVoltage(0));
		mSubResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({node(0), mVirtualNodes[2]});
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromPowerflow(frequency);
		mSubInductor->connect({mVirtualNodes[2], mVirtualNodes[0]});
	} else {
		mSubInductor->connect({node(0), mVirtualNodes[0]});
	}
	mSubInductor->initialize(mFrequencies);
	mSubInductor->initializeFromPowerflow(frequency);

	// Create parallel sub components
	mSubSnubResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_snub_res", mLogLevel);
	mSubSnubResistor->setParameters(snubberResistance);
	mSubSnubResistor->connect({ node(1), DP::Node::GND });
	mSubSnubResistor->initialize(mFrequencies);
	mSubSnubResistor->initializeFromPowerflow(frequency);

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

void DP::Ph1::Transformer::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	auto subComponents = MNAInterface::List({mSubInductor, mSubSnubResistor});
	if (mSubResistor)
		subComponents.push_back(mSubResistor);
	for (auto comp : subComponents) {
		comp->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : comp->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mSLog->info(
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\nTerminal 1 connected to {:s} = sim node {:d}",
		mTerminals[0]->node()->name(), mTerminals[0]->node()->simNode(),
		mTerminals[1]->node()->name(), mTerminals[1]->node()->simNode());
}

void DP::Ph1::Transformer::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Ideal transformer equations
	if (terminalNotGrounded(0)) {
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), mVirtualNodes[1]->simNode(), Complex(-1.0, 0));
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->simNode(), mVirtualNodes[0]->simNode(), Complex(1.0, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setMatrixElement(systemMatrix, simNode(1), mVirtualNodes[1]->simNode(), mRatio);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->simNode(), simNode(1), -mRatio);
	}

	// Add inductive part to system matrix
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSnubResistor->mnaApplySystemMatrixStamp(systemMatrix);

	if (mNumVirtualNodes == 3) {
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	}

	if (terminalNotGrounded(0)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->simNode(),  mVirtualNodes[1]->simNode());
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->simNode(), mVirtualNodes[0]->simNode());
	}
	if (terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(mRatio),
			simNode(1), mVirtualNodes[1]->simNode());
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-mRatio),
			mVirtualNodes[1]->simNode(), simNode(1));
	}
}

void DP::Ph1::Transformer::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::Transformer::MnaPreStep::execute(Real time, Int timeStepCount) {
	mTransformer.mnaApplyRightSideVectorStamp(mTransformer.mRightVector);
}

void DP::Ph1::Transformer::MnaPostStep::execute(Real time, Int timeStepCount) {
	mTransformer.mnaUpdateVoltage(*mLeftVector);
	mTransformer.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph1::Transformer::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0,0) = mSubInductor->intfCurrent()(0, 0);
}

void DP::Ph1::Transformer::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage(0, 0) = 0;
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, simNode(1));
	mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->simNode());
	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString(mIntfVoltage(0, 0)));
}

