/**
 * @file
 * @author Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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

#include "cps/SP/SP_Ph1_RXLine.h"

using namespace CPS;

SP::Ph1::RXLine::RXLine(String uid, String name, Real baseVoltage,
	Real resistance, Real inductance,
	Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {

	setTerminalNumber(2);

	mBaseVoltage = baseVoltage;
	mSeriesRes = resistance;
	mInductance = inductance;

	addAttribute<Real>("base_Voltage", &mBaseVoltage, Flags::read | Flags::write);
	addAttribute<Real>("R_series", &mSeriesRes, Flags::read | Flags::write);
	addAttribute<Real>("L_series", &mInductance, Flags::read | Flags::write);
	addAttribute<Complex>("current", &mCurrent(0), Flags::read | Flags::write);
	addAttribute<Complex>("current_1", &mCurrent(1), Flags::read | Flags::write);
	addAttribute<Real>("p_branch", &mActivePowerBranch(0), Flags::read | Flags::write);
	addAttribute<Real>("q_branch", &mReactivePowerBranch(0), Flags::read | Flags::write);
	addAttribute<Real>("p_branch_1", &mActivePowerBranch(1), Flags::read | Flags::write);
	addAttribute<Real>("q_branch_1", &mReactivePowerBranch(1), Flags::read | Flags::write);

	addAttribute<Bool>("nodal_injection_stored", &mStoreNodalPowerInjection, Flags::read | Flags::write);
	addAttribute<Real>("p_inj", &mActivePowerInjection, Flags::read | Flags::write);
	addAttribute<Real>("q_inj", &mReactivePowerInjection, Flags::read | Flags::write);

	// mLog.Log(Logger::Level::DEBUG) << "Create " << this->type() << " " << name
	// 	<< " R=" << resistance << " L=" << inductance
	// 	 << std::endl;
}

SP::Ph1::RXLine::RXLine(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Real>("R", &mSeriesRes, Flags::read | Flags::write);
	addAttribute<Real>("L", &mSeriesInd, Flags::read | Flags::write);
}


void SP::Ph1::RXLine::setPerUnitSystem(Real baseApparentPower, Real baseOmega) {
	mBaseApparentPower = baseApparentPower;
	mBaseOmega = baseOmega;
	mBaseImpedance = (mBaseVoltage * mBaseVoltage) / mBaseApparentPower;
	mBaseAdmittance = 1.0 / mBaseImpedance;
	mBaseInductance = mBaseImpedance / mBaseOmega;
	/// I_base = S_base / V_line
	mBaseCurrent = baseApparentPower / (mBaseVoltage * sqrt(3));
/*
	mLog.Log(Logger::Level::INFO) << "#### Set Per Unit System for " << mName << std::endl;
	mLog.Log(Logger::Level::INFO) << " Base Voltage= " << mBaseVoltage << " [V] " << " Base Impedance= " << mBaseImpedance << " [Ohm] " << std::endl;
	*/

    mSeriesResPerUnit = mSeriesRes / mBaseImpedance;
	mSeriesIndPerUnit = mInductance / mBaseInductance;
/*
	mLog.Log(Logger::Level::INFO) << "Series Resistance Per Unit= " << " " << mSeriesResPerUnit << " [Ohm] "
		<< " Series Inductance Per Unit= " << " " << mSeriesIndPerUnit << " [H] "
		<< std::endl;
    mLog.Log(Logger::Level::INFO)  << "r " << mSeriesResPerUnit << std::endl << "	x: " << mBaseOmega * mInductance / mBaseImpedance<<std::endl;*/
}

void SP::Ph1::RXLine::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) {
	updateSimNodes();
	int bus1 = this->simNode(0);
	int bus2 = this->simNode(1);

	//dimension check
	/* TODO: FIX
	if (bus1 > (n - 1) || bus2 > (n - 1)) {
		std::stringstream ss;
		ss << "Line>>" << this->getName() << ": Wrong Y dimension: " << n;
		throw std::invalid_argument(ss.str());
		//std::cout << "Line>>" << Name << ": Wrong Y dimension: " << n << endl;
		return;
	}
	*/

	//create the element admittance matrix
	Complex y = Complex(1, 0) / Complex(mSeriesResPerUnit, 1. * mSeriesIndPerUnit);

	//Fill the internal matrix
	mY_element = MatrixComp(2, 2);
	mY_element(0, 0) = y;
	mY_element(0, 1) = -y;
	mY_element(1, 0) = -y;
	mY_element(1, 1) = y;

	//check for inf or nan
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			if (std::isinf(mY_element.coeff(i, j).real()) || std::isinf(mY_element.coeff(i, j).imag())) {
				std::cout << mY_element << std::endl;
				std::stringstream ss;
				ss << "Line>>" << this->name() << ": infinite or nan values in the element Y at: " << i << "," << j;
				throw std::invalid_argument(ss.str());
				std::cout << "Line>>" << this->name() << ": infinite or nan values in the element Y at: " << i << "," << j << std::endl;
			}

	//set the circuit matrix values
	Y.coeffRef(bus1, bus1) += mY_element.coeff(0, 0);
	Y.coeffRef(bus1, bus2) += mY_element.coeff(0, 1);
	Y.coeffRef(bus2, bus1) += mY_element.coeff(1, 0);
	Y.coeffRef(bus2, bus2) += mY_element.coeff(1, 1);

	//mLog.Log(Logger::Level::INFO) << "#### Y matrix stamping: " << std::endl;
	//mLog.Log(Logger::Level::INFO) << mY_element << std::endl;
}


void SP::Ph1::RXLine::updateBranchFlow(VectorComp& current, VectorComp& powerflow) {
	mCurrent = current * mBaseCurrent;
	mActivePowerBranch = powerflow.real() * mBaseApparentPower;
	mReactivePowerBranch = powerflow.imag() * mBaseApparentPower;
}

void SP::Ph1::RXLine::storeNodalInjection(Complex powerInjection) {
	mActivePowerInjection = std::real(powerInjection) * mBaseApparentPower;
	mReactivePowerInjection = std::imag(powerInjection) * mBaseApparentPower;
	mStoreNodalPowerInjection = true;
}


MatrixComp SP::Ph1::RXLine::Y_element() {
	return mY_element;
}


PowerComponent<Complex>::Ptr SP::Ph1::RXLine::clone(String name) {
	auto copy = RXLine::make(name, mLogLevel);
	copy->setParameters(mSeriesRes, mSeriesInd);
	return copy;
}

void SP::Ph1::RXLine::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	mIntfVoltage(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	Complex impedance = { mSeriesRes, mSeriesInd * 2. * PI * frequency };
	mIntfCurrent(0, 0) = mIntfVoltage(0, 0) / impedance;
	mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(0) + mIntfCurrent(0, 0) * mSeriesRes);

	// Default model with virtual node in between
	mSubResistor = std::make_shared<SP::Ph1::Resistor>(mName + "_res", mLogLevel);
	mSubResistor->setParameters(mSeriesRes);
	mSubResistor->connect({ mTerminals[0]->node(), mVirtualNodes[0] });
	mSubResistor->initializeFromPowerflow(frequency);

	mSubInductor = std::make_shared<SP::Ph1::Inductor>(mName + "_ind", mLogLevel);
	mSubInductor->setParameters(mSeriesInd);
	mSubInductor->connect({ mVirtualNodes[0], mTerminals[1]->node() });
	mSubInductor->initializeFromPowerflow(frequency);

	mInitialResistor = std::make_shared<SP::Ph1::Resistor>(mName + "_snubber_res", mLogLevel);
	mInitialResistor->setParameters(1e6);
	mInitialResistor->connect({ Node::GND, mTerminals[1]->node() });
	mInitialResistor->initializeFromPowerflow(frequency);

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

void SP::Ph1::RXLine::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	mSubInductor->mnaInitialize(omega, timeStep, leftVector);
	mSubResistor->mnaInitialize(omega, timeStep, leftVector);
	mInitialResistor->mnaInitialize(omega, timeStep, leftVector);
	for (auto task : mSubInductor->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
	for (auto task : mSubResistor->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void SP::Ph1::RXLine::mnaApplyInitialSystemMatrixStamp(Matrix& systemMatrix) {
	mInitialResistor->mnaApplySystemMatrixStamp(systemMatrix);
}

void SP::Ph1::RXLine::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mInitialResistor->mnaApplySystemMatrixStamp(systemMatrix);
}


void SP::Ph1::RXLine::MnaPreStep::execute(Real time, Int timeStepCount) {
	mLine.mnaApplyRightSideVectorStamp(mLine.mRightVector);
}

void SP::Ph1::RXLine::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLine.mnaUpdateVoltage(*mLeftVector);
	mLine.mnaUpdateCurrent(*mLeftVector);
}

void SP::Ph1::RXLine::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0, 0) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, simNode(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, simNode(0));
}

void SP::Ph1::RXLine::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = mSubInductor->intfCurrent()(0, 0);
}
