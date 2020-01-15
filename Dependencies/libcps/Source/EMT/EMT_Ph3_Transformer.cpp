/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 *		   Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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

#include <cps/EMT/EMT_Ph3_Transformer.h>
#define SHIFT_TO_PHASE_B Complex(cos(-2 * M_PI / 3), sin(-2 * M_PI / 3))
#define SHIFT_TO_PHASE_C Complex(cos(2 * M_PI / 3), sin(2 * M_PI / 3))

using namespace CPS;

EMT::Ph3::Transformer::Transformer(String uid, String name,
	Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(2);
	setVirtualNodeNumber(1);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(1, 1);

	addAttribute<Complex>("ratio", &mRatio, Flags::write | Flags::read);
	addAttribute<Matrix>("R", &mResistance, Flags::write | Flags::read);
	addAttribute<Matrix>("L", &mInductance, Flags::write | Flags::read);
}

PowerComponent<Real>::Ptr EMT::Ph3::Transformer::clone(String name) {
	auto copy = Transformer::make(name, mLogLevel);
	copy->setParameters(std::abs(mRatio), std::arg(mRatio), mResistance, mInductance);
	return copy;
}

void EMT::Ph3::Transformer::setParameters(Real ratioAbs, Real ratioPhase,
	Matrix resistance, Matrix inductance) {
	Base::Ph3::Transformer::setParameters(ratioAbs, ratioPhase, resistance, inductance);

	if (resistance(0, 0) > 0)
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);

	parametersSet = true;
}

void EMT::Ph3::Transformer::initialize(Matrix frequencies) {
	PowerComponent<Real>::initialize(frequencies);
}

void EMT::Ph3::Transformer::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	// A snubber conductance is added on the low voltage side
	Matrix snubberResistance = Matrix::Zero(3, 3);
	snubberResistance <<
		1e3, 0, 0,
		0, 1e3, 0,
		0, 0, 1e3;

	// Component parameters are referred to high voltage side.
	// Switch terminals if transformer is connected the other way around.
	if (Math::abs(mRatio) < 1.) {
		mRatio = 1. / mRatio;
		std::shared_ptr<Terminal<Real>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
	}

	// Set initial voltage of virtual node in between
	mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(1) * mRatio);

	// Static calculations from load flow data
	Real omega = 2. * PI * frequency;
	MatrixComp impedance = MatrixComp::Zero(3, 3);
	impedance <<
		Complex(mResistance(0, 0), omega * mInductance(0, 0)), Complex(mResistance(0, 1), omega * mInductance(0, 1)), Complex(mResistance(0, 2), omega * mInductance(0, 2)),
		Complex(mResistance(1, 0), omega * mInductance(1, 0)), Complex(mResistance(1, 1), omega * mInductance(1, 1)), Complex(mResistance(1, 2), omega * mInductance(1, 2)),
		Complex(mResistance(2, 0), omega * mInductance(2, 0)), Complex(mResistance(2, 1), omega * mInductance(2, 1)), Complex(mResistance(2, 2), omega * mInductance(2, 2));

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = mVirtualNodes[0]->initialSingleVoltage() - initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

	MatrixComp iInit = impedance.inverse() * vInitABC;
	mIntfCurrent = iInit.real();
	mIntfVoltage = vInitABC.real();

	// Create series sub components
	mSubInductor = std::make_shared<EMT::Ph3::Inductor>(mName + "_ind", mLogLevel);
	mSubInductor->setParameters(mInductance);

	if (mNumVirtualNodes == 3) {
		mVirtualNodes[2]->setInitialVoltage(initialSingleVoltage(0));
		mSubResistor = std::make_shared<EMT::Ph3::Resistor>(mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ node(0), mVirtualNodes[2] });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromPowerflow(frequency);
		mSubInductor->connect({ mVirtualNodes[2], mVirtualNodes[0] });
	}
	else {
		mSubInductor->connect({ node(0), mVirtualNodes[0] });
	}
	mSubInductor->initialize(mFrequencies);
	mSubInductor->initializeFromPowerflow(frequency);

	// Create parallel sub components
	mSubSnubResistor = std::make_shared<EMT::Ph3::Resistor>(mName + "_snub_res", mLogLevel);
	mSubSnubResistor->setParameters(snubberResistance);
	mSubSnubResistor->connect({ node(1), EMT::Node::GND });
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
		Logger::matrixToString(mIntfVoltage),
		Logger::matrixToString(mIntfCurrent),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

void EMT::Ph3::Transformer::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	auto subComponents = MNAInterface::List({ mSubInductor, mSubSnubResistor });
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

void EMT::Ph3::Transformer::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Ideal transformer equations
	if (terminalNotGrounded(0)) {
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(PhaseType::A), mVirtualNodes[1]->simNode(PhaseType::A), -1.);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(PhaseType::B), mVirtualNodes[1]->simNode(PhaseType::B), -1.);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(PhaseType::C), mVirtualNodes[1]->simNode(PhaseType::C), -1.);

		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->simNode(PhaseType::A), mVirtualNodes[0]->simNode(PhaseType::A), 1.);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->simNode(PhaseType::B), mVirtualNodes[0]->simNode(PhaseType::B), 1.);
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->simNode(PhaseType::C), mVirtualNodes[0]->simNode(PhaseType::C), 1.);

	}
	if (terminalNotGrounded(1)) {
		Math::setMatrixElement(systemMatrix, simNode(1, 0), mVirtualNodes[1]->simNode(PhaseType::A), mRatio.real());
		Math::setMatrixElement(systemMatrix, simNode(1, 1), mVirtualNodes[1]->simNode(PhaseType::B), mRatio.real());
		Math::setMatrixElement(systemMatrix, simNode(1, 2), mVirtualNodes[1]->simNode(PhaseType::C), mRatio.real());
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->simNode(PhaseType::A), simNode(1, 0), -mRatio.real());
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->simNode(PhaseType::B), simNode(1, 1), -mRatio.real());
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->simNode(PhaseType::C), simNode(1, 2), -mRatio.real());
	}

	// Add inductive part to system matrix
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSnubResistor->mnaApplySystemMatrixStamp(systemMatrix);

	if (mNumVirtualNodes == 3) {
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	}

	if (terminalNotGrounded(0)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->simNode(PhaseType::A), mVirtualNodes[1]->simNode(PhaseType::A));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->simNode(PhaseType::B), mVirtualNodes[1]->simNode(PhaseType::B));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(-1.0, 0)),
			mVirtualNodes[0]->simNode(PhaseType::C), mVirtualNodes[1]->simNode(PhaseType::C));

		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->simNode(PhaseType::A), mVirtualNodes[0]->simNode(PhaseType::A));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->simNode(PhaseType::B), mVirtualNodes[0]->simNode(PhaseType::B));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(Complex(1.0, 0)),
			mVirtualNodes[1]->simNode(PhaseType::C), mVirtualNodes[0]->simNode(PhaseType::C));
	}
	if (terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(mRatio),
			simNode(1, 0), mVirtualNodes[1]->simNode(PhaseType::A));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(mRatio),
			simNode(1, 1), mVirtualNodes[1]->simNode(PhaseType::B));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(mRatio),
			simNode(1, 2), mVirtualNodes[1]->simNode(PhaseType::C));

		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-mRatio),
			mVirtualNodes[1]->simNode(PhaseType::A), simNode(1, 0));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-mRatio),
			mVirtualNodes[1]->simNode(PhaseType::B), simNode(1, 1));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-mRatio),
			mVirtualNodes[1]->simNode(PhaseType::C), simNode(1, 2));
	}
}

void EMT::Ph3::Transformer::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
}

void EMT::Ph3::Transformer::MnaPreStep::execute(Real time, Int timeStepCount) {
	mTransformer.mnaApplyRightSideVectorStamp(mTransformer.mRightVector);
}

void EMT::Ph3::Transformer::MnaPostStep::execute(Real time, Int timeStepCount) {
	mTransformer.mnaUpdateVoltage(*mLeftVector);
	mTransformer.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::Transformer::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mSubInductor->intfCurrent();
}

void EMT::Ph3::Transformer::mnaUpdateVoltage(const Matrix& leftVector) {
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

