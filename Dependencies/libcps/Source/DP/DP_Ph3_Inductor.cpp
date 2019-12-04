/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 *         Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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

#include <cps/DP/DP_Ph3_Inductor.h>

using namespace CPS;


DP::Ph3::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	mEquivCurrent = MatrixComp::Zero(3,1);
	mIntfVoltage = MatrixComp::Zero(3,1);
	mIntfCurrent = MatrixComp::Zero(3,1);
	setTerminalNumber(2);

	addAttribute<Real>("L", &mInductance, Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr DP::Ph3::Inductor::clone(String name) {
	auto copy = Inductor::make(name, mLogLevel);
	copy->setParameters(mInductance);
	return copy;
}

void DP::Ph3::Inductor::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Real omega = 2 * PI * frequency;
	MatrixComp impedance = Matrix::Zero(3,1);
     impedance<<
     Complex( 0, omega * mInductance ), Complex(0, omega * mInductance), Complex(0, omega * mInductance);

	 // IntfVoltage initialization for each phase
	 mIntfVoltage(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	 Real voltMag = Math::abs(mIntfVoltage(0, 0));
	 Real voltPhase = Math::phase(mIntfVoltage(0, 0));
	 mIntfVoltage(1, 0) = Complex(
		 voltMag*cos(voltPhase - 2. / 3.*M_PI),
		 voltMag*sin(voltPhase - 2. / 3.*M_PI));
	 mIntfVoltage(2, 0) = Complex(
		 voltMag*cos(voltPhase + 2. / 3.*M_PI),
		 voltMag*sin(voltPhase + 2. / 3.*M_PI));

	 mIntfCurrent = impedance.cwiseInverse().cwiseProduct(mIntfVoltage);

	//TODO
	 mSLog->info( "--- Initialize according to power flow ---" );
				// << "in phase A: " << std::endl
				// << "Voltage across: " << std::abs(mIntfVoltage(0,0))
				// << "<" << Math::phaseDeg(mIntfVoltage(0,0)) << std::endl
				// << "Current: " << std::abs(mIntfCurrent(0,0))
				// << "<" << Math::phaseDeg(mIntfCurrent(0,0)) << std::endl
				// << "Terminal 0 voltage: " << std::abs(initialSingleVoltage(0))
				// << "<" << Math::phaseDeg(initialSingleVoltage(0)) << std::endl
				// << "Terminal 1 voltage: " << std::abs(initialSingleVoltage(1))
				// << "<" << Math::phaseDeg(initialSingleVoltage(1)) << std::endl
				// << "--- Power flow initialization finished ---" << std::endl;
}

void DP::Ph3::Inductor::initVars(Real omega, Real timeStep) {
	Real a = timeStep / (2. * mInductance);
	Real b = timeStep * omega / 2.;

	Real equivCondReal = a / (1. + b * b);
	Real equivCondImag =  -a * b / (Real(1.) + b * b);
	mEquivCond = Matrix::Zero(3, 1);
	mEquivCond << Complex(equivCondReal, equivCondImag),
		Complex(equivCondReal, equivCondImag),
		Complex(equivCondReal, equivCondImag);


	Real preCurrFracReal = (1. - b * b) / (1. + b * b);
	Real preCurrFracImag =  (-2. * b) / (1. + b * b);
	mPrevCurrFac = Matrix::Zero(3, 1);
	mPrevCurrFac << Complex(preCurrFracReal, preCurrFracImag),
		Complex(preCurrFracReal, preCurrFracImag),
		Complex(preCurrFracReal, preCurrFracImag);


	// TODO: check if this is correct or if it should be only computed before the step
	mEquivCurrent = mEquivCond .cwiseProduct( mIntfVoltage) + mPrevCurrFac.cwiseProduct( mIntfCurrent);
	// no need to update this now
	//mIntfCurrent = mEquivCond.cwiseProduct(mIntfVoltage) + mEquivCurrent;
}

void DP::Ph3::Inductor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateSimNodes();
	initVars(omega, timeStep);

	mSLog->info(  "Initial voltage {}",  Math::abs(mIntfVoltage(0,0)));
				// << "<" << Math::phaseDeg(mIntfVoltage(0,0)) << std::endl
				// << "Initial current " << Math::abs(mIntfCurrent(0,0))
				// << "<" << Math::phaseDeg(mIntfCurrent(0,0)) << std::endl;

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph3::Inductor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 0), mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 1), mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 2), mEquivCond(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 0), mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 1), mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 2), mEquivCond(2, 0));
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 0), -mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 0), -mEquivCond(0, 0));

		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 1), -mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 1), -mEquivCond(1, 0));

		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 2), -mEquivCond(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 2), -mEquivCond(2, 0));
	}


	// if (terminalNotGrounded(0))
	// 	mLog.debug() << "Add " << mEquivCond << " to system at " << simNode(0) << "," << simNode(0) << std::endl;
	// if (terminalNotGrounded(1))
	// 	mLog.debug() << "Add " << mEquivCond << " to system at " << simNode(1) << "," << simNode(1) << std::endl;
	// if (terminalNotGrounded(0)  &&  terminalNotGrounded(1))
	// 	mLog.debug() << "Add " << -mEquivCond << " to system at " << simNode(0) << "," << simNode(1) << std::endl
	// 				 << "Add " << -mEquivCond << " to system at " << simNode(1) << "," << simNode(0) << std::endl;
}

void DP::Ph3::Inductor::mnaApplyRightSideVectorStamp(Matrix& rightVector) {

	// Calculate equivalent current source for next time step
	mEquivCurrent = mEquivCond.cwiseProduct(mIntfVoltage) + mPrevCurrFac.cwiseProduct(mIntfCurrent);

	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, simNode(0, 0), mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, simNode(0, 1), mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, simNode(0, 2), mEquivCurrent(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, simNode(1, 0), -mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, simNode(1, 1), -mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, simNode(1, 2), -mEquivCurrent(2, 0));
	}
}

void DP::Ph3::Inductor::MnaPreStep::execute(Real time, Int timeStepCount) {
	mInductor.mnaApplyRightSideVectorStamp(mInductor.mRightVector);
}

void DP::Ph3::Inductor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mInductor.mnaUpdateVoltage(*mLeftVector);
	mInductor.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph3::Inductor::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage = Matrix::Zero(3, 1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, simNode(1, 0));
		mIntfVoltage(1, 0) = Math::complexFromVectorElement(leftVector, simNode(1, 1));
		mIntfVoltage(2, 0) = Math::complexFromVectorElement(leftVector, simNode(1, 2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, simNode(0, 0));
		mIntfVoltage(1, 0) = mIntfVoltage(1, 0) - Math::complexFromVectorElement(leftVector, simNode(0, 1));
		mIntfVoltage(2, 0) = mIntfVoltage(2, 0) - Math::complexFromVectorElement(leftVector, simNode(0, 2));
	}
}

void DP::Ph3::Inductor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mEquivCond .cwiseProduct(mIntfVoltage) + mEquivCurrent;
}

void DP::Ph3::Inductor::mnaTearInitialize(Real omega, Real timeStep) {
	initVars(omega, timeStep);
}

void DP::Ph3::Inductor::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
	/*
	Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx, mEquivCond.cwiseInverse()(0,0));
	*/
}

void DP::Ph3::Inductor::mnaTearApplyVoltageStamp(Matrix& voltageVector) {
	/*
	mEquivCurrent = mEquivCond * mIntfVoltage(0,0) + mPrevCurrFac * mIntfCurrent(0,0);
	Math::addToVectorElement(voltageVector, mTearIdx, mEquivCurrent .cwiseProduct( mEquivCond.cwiseInverse()));
	*/
}

void DP::Ph3::Inductor::mnaTearPostStep(Complex voltage, Complex current) {
	/*
	mIntfVoltage = voltage;
	mIntfCurrent = mEquivCond * voltage + mEquivCurrent;
	*/
}

