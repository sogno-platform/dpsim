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

#include <cps/EMT/EMT_Ph3_RXLoad.h>
#define SHIFT_TO_PHASE_B Complex(cos(-2 * M_PI / 3), sin(-2 * M_PI / 3))
#define SHIFT_TO_PHASE_C Complex(cos(2 * M_PI / 3), sin(2 * M_PI / 3))

using namespace CPS;

EMT::Ph3::RXLoad::RXLoad(String uid, String name,
	Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	addAttribute<Matrix>("P", &mActivePower, Flags::read | Flags::write);
	addAttribute<Matrix>("Q", &mReactivePower, Flags::read | Flags::write);
	addAttribute<Real>("V_nom", &mNomVoltage, Flags::read | Flags::write);
}

EMT::Ph3::RXLoad::RXLoad(String name,
	Logger::Level logLevel)
	: RXLoad(name, name, logLevel) {
}

EMT::Ph3::RXLoad::RXLoad(String name,
	Matrix activePower, Matrix reactivePower, Real volt,
	Logger::Level logLevel)
	: RXLoad(name, logLevel) {
	mActivePower = activePower;
	mReactivePower = reactivePower;
	mPower = MatrixComp::Zero(3,3);
	mPower <<
		Complex(mActivePower(0, 0), mReactivePower(0, 0)), Complex(mActivePower(0, 1), mReactivePower(0, 1)), Complex(mActivePower(0, 2), mReactivePower(0, 2)),
		Complex(mActivePower(1, 0), mReactivePower(1, 0)), Complex(mActivePower(1, 1), mReactivePower(1, 1)), Complex(mActivePower(1, 2), mReactivePower(1, 2)),
		Complex(mActivePower(2, 0), mReactivePower(2, 0)), Complex(mActivePower(2, 1), mReactivePower(2, 1)), Complex(mActivePower(2, 2), mReactivePower(2, 2));

	mNomVoltage = volt;
	initPowerFromTerminal = false;
}

PowerComponent<Real>::Ptr EMT::Ph3::RXLoad::clone(String name) {
	// everything set by initializeFromPowerflow
	return RXLoad::make(name, mLogLevel);
}

void EMT::Ph3::RXLoad::initialize(Matrix frequencies) {
	PowerComponent<Real>::initialize(frequencies);
}

void EMT::Ph3::RXLoad::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	if (initPowerFromTerminal) {
	mActivePower = Matrix::Zero(3, 3);
	mActivePower(0, 0) = mTerminals[0]->singleActivePower();
	mActivePower(1, 1) = mTerminals[0]->singleActivePower();
	mActivePower(2, 2) = mTerminals[0]->singleActivePower();

	mReactivePower = Matrix::Zero(3, 3);
	mReactivePower(0, 0) = mTerminals[0]->singleReactivePower();
	mReactivePower(1, 1) = mTerminals[0]->singleReactivePower();
	mReactivePower(2, 2) = mTerminals[0]->singleReactivePower();

	// complex power
	mPower = MatrixComp::Zero(3, 3);
	mPower(0, 0) = { mActivePower(0, 0), mReactivePower(0, 0) };
	mPower(1, 1) = { mActivePower(1, 1), mReactivePower(1, 1) };
	mPower(2, 2) = { mActivePower(2, 2), mReactivePower(2, 2) };

	mNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());
	}

	if (mActivePower(0,0) != 0) {
		mResistance = std::pow(mNomVoltage, 2) * mActivePower.inverse();
		mConductance = mResistance.inverse();
		mSubResistor = std::make_shared<EMT::Ph3::Resistor>(mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ Node::GND, mTerminals[0]->node() });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromPowerflow(frequency);
	}

	if (mReactivePower(0, 0) != 0)
		mReactance = std::pow(mNomVoltage, 2) * mReactivePower.inverse();
	else
		mReactance = Matrix::Zero(0, 0);

	if (mReactance(0,0) > 0) {
		mInductance = mReactance / (2 * PI * frequency);

		mSubInductor = std::make_shared<EMT::Ph3::Inductor>(mName + "_ind", mLogLevel);
		mSubInductor->setParameters(mInductance);
		mSubInductor->connect({ Node::GND, mTerminals[0]->node() });
		mSubInductor->initialize(mFrequencies);
		mSubInductor->initializeFromPowerflow(frequency);
	}
	else if (mReactance(0,0) < 0) {
		mCapacitance = -1 / (2 * PI * frequency) * mReactance.inverse();

		mSubCapacitor = std::make_shared<EMT::Ph3::Capacitor>(mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(mCapacitance);
		mSubCapacitor->connect({ Node::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromPowerflow(frequency);
	}

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = mTerminals[0]->initialSingleVoltage();
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	mIntfVoltage = vInitABC.real();

	MatrixComp iInitABC = MatrixComp::Zero(3, 1);
	// v i^T* = S
	// v^T v i^T* = v^T S
	// i^T*= (|v|^2)^(-1) v^T S
	Complex v_ = vInitABC(0, 0)*vInitABC(0, 0) + vInitABC(1, 0)*vInitABC(1, 0) + vInitABC(2, 0)*vInitABC(2, 0);
	MatrixComp rhs_ = Complex(1, 0) / v_ * vInitABC.transpose() * mPower;
	iInitABC = rhs_.conjugate().transpose();
	mIntfCurrent = iInitABC.real();

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(mIntfVoltage),
		Logger::matrixToString(mIntfCurrent),
		Logger::phasorToString(initialSingleVoltage(0)));
}

void EMT::Ph3::RXLoad::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	if (mSubResistor) {
		mSubResistor->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubResistor->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	if (mSubInductor) {
		mSubInductor->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubInductor->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	if (mSubCapacitor) {
		mSubCapacitor->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubCapacitor->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph3::RXLoad::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mSubResistor)
		mSubResistor->mnaApplyRightSideVectorStamp(rightVector);
	if (mSubInductor)
		mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
	if (mSubCapacitor)
		mSubCapacitor->mnaApplyRightSideVectorStamp(rightVector);
}

void EMT::Ph3::RXLoad::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (mSubResistor)
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubInductor)
		mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubCapacitor)
		mSubCapacitor->mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::RXLoad::MnaPreStep::execute(Real time, Int timeStepCount) {
	mLoad.mnaApplyRightSideVectorStamp(mLoad.mRightVector);
}

void EMT::Ph3::RXLoad::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLoad.mnaUpdateVoltage(*mLeftVector);
	mLoad.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::RXLoad::mnaUpdateVoltage(const Matrix& leftVector) {
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

void EMT::Ph3::RXLoad::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = Matrix::Zero(3, 1);
	if (mSubResistor)
		mIntfCurrent += mSubResistor->intfCurrent();
	if (mSubInductor)
		mIntfCurrent += mSubInductor->intfCurrent();
	if (mSubCapacitor)
		mIntfCurrent += mSubCapacitor->intfCurrent();
}
