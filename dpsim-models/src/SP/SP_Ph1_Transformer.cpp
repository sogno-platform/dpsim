/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_Transformer.h>

using namespace CPS;

// #### General ####
SP::Ph1::Transformer::Transformer(String uid, String name, Logger::Level logLevel)
	: Base::Ph1::Transformer(mAttributes), MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
	mBaseVoltage(mAttributes->create<Real>("base_Voltage")),
	mCurrent(mAttributes->create<MatrixComp>("current_vector")),
	mActivePowerBranch(mAttributes->create<Matrix>("p_branch_vector")),
	mReactivePowerBranch(mAttributes->create<Matrix>("q_branch_vector")),
	mActivePowerInjection(mAttributes->create<Real>("p_inj")),
	mReactivePowerInjection(mAttributes->create<Real>("q_inj")) {

	setTerminalNumber(2);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);

	**mCurrent = MatrixComp::Zero(2,1);
	**mActivePowerBranch = Matrix::Zero(2,1);
	**mReactivePowerBranch = Matrix::Zero(2,1);
}


void SP::Ph1::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	// Note: to be consistent impedance values must be referred to high voltage side (and base voltage set to higher voltage)
	Base::Ph1::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);

	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]", **mNominalVoltageEnd1, **mNominalVoltageEnd2);
	SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [Ohm] Inductance={} [H] (referred to primary side)", **mResistance, **mInductance);
    SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [/] Phase Shift={} [deg]", std::abs(**mRatio), std::arg(**mRatio));
	SPDLOG_LOGGER_INFO(mSLog, "Rated Power={} [W]", **mRatedPower);

	mRatioAbs = std::abs(**mRatio);
	mRatioPhase = std::arg(**mRatio);

	mParametersSet = true;
}

void SP::Ph1::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	**mRatedPower = ratedPower;
	SPDLOG_LOGGER_INFO(mSLog, "Rated Power={} [W]", **mRatedPower);

	SP::Ph1::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);
	mSLog->flush();
}

void SP::Ph1::Transformer::initializeFromNodesAndTerminals(Real frequency) {
	mNominalOmega = 2. * PI * frequency;
	mReactance = mNominalOmega * **mInductance;
	mImpedance = { **mResistance, mReactance };
	SPDLOG_LOGGER_INFO(mSLog, "Impedance={} [Ohm] (referred to primary side)", mImpedance);
	SPDLOG_LOGGER_INFO(mSLog, "Reactance={} [Ohm] (referred to primary side)", mReactance);
	mSLog->flush();

	// Component parameters are referred to higher voltage side.
	// Switch terminals to have terminal 0 at higher voltage side
	// if transformer is connected the other way around.
	if (Math::abs(**mRatio) < 1.) {
		**mRatio = 1. / **mRatio;
		mRatioAbs = std::abs(**mRatio);
		mRatioPhase = std::arg(**mRatio);
		std::shared_ptr<SimTerminal<Complex>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
		Real tmpVolt = **mNominalVoltageEnd1;
		**mNominalVoltageEnd1 = **mNominalVoltageEnd2;
		**mNominalVoltageEnd2 = tmpVolt;
		SPDLOG_LOGGER_INFO(mSLog, "Switching terminals to have first terminal at higher voltage side. Updated parameters: ");
		SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage End 1 = {} [V] Nominal Voltage End 2 = {} [V]", **mNominalVoltageEnd1, **mNominalVoltageEnd2);
		SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio = {} [ ] Phase Shift = {} [deg]", mRatioAbs, mRatioPhase);
	}

	// Static calculations from load flow data
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(0) - initialSingleVoltage(1) * **mRatio;
	(**mIntfCurrent)(0, 0) = -(initialSingleVoltage(0) - initialSingleVoltage(1) * **mRatio) / mImpedance;

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across primary side: {:s}"
		"\nPrimary side current flowing into node 0: {:s}"
		"\nTerminal 0 voltage (HV side voltage): {:s}"
		"\nTerminal 1 voltage (LV side voltage): {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		Logger::phasorToString((**mIntfCurrent)(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
	mSLog->flush();
}


// #### Powerflow section ####

void SP::Ph1::Transformer::setBaseVoltage(Real baseVoltage) {
	// Note: to be consistent set base voltage to higher voltage (and impedance values must be referred to high voltage side)
	// TODO: use attribute setter for setting base voltage
    **mBaseVoltage = baseVoltage;
}

void SP::Ph1::Transformer::calculatePerUnitParameters(Real baseApparentPower, Real baseOmega) {
	SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}", **mName);
    mBaseApparentPower = baseApparentPower;
	mBaseOmega = baseOmega;
    SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]  Base Omega={} [1/s]", baseApparentPower, baseOmega);

	mBaseImpedance = **mBaseVoltage * **mBaseVoltage / mBaseApparentPower;
	mBaseAdmittance = 1.0 / mBaseImpedance;
	mBaseCurrent = baseApparentPower / (**mBaseVoltage * sqrt(3)); // I_base=(S_threephase/3)/(V_line_to_line/sqrt(3))
	SPDLOG_LOGGER_INFO(mSLog, "Base Voltage={} [V]  Base Impedance={} [Ohm]", **mBaseVoltage, mBaseImpedance);

	mResistancePerUnit = **mResistance / mBaseImpedance;
	mReactancePerUnit = mReactance / mBaseImpedance;
    SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [pu]  Reactance={} [pu]", mResistancePerUnit, mReactancePerUnit);

	mBaseInductance = mBaseImpedance / mBaseOmega;
	mInductancePerUnit = **mInductance / mBaseInductance;
	// omega per unit=1, hence 1.0*mInductancePerUnit.
	mLeakagePerUnit = Complex(mResistancePerUnit,1.*mInductancePerUnit);
	SPDLOG_LOGGER_INFO(mSLog, "Leakage Impedance={} [pu] ", mLeakagePerUnit);

    mRatioAbsPerUnit = mRatioAbs / **mNominalVoltageEnd1 * **mNominalVoltageEnd2;
    SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [pu]", mRatioAbsPerUnit);
}

void SP::Ph1::Transformer::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) {
	// calculate matrix stamp
	mY_element = MatrixComp(2, 2);
	Complex y = Complex(1, 0) / mLeakagePerUnit;

	mY_element(0, 0) = y;
	mY_element(0, 1) = -y*mRatioAbsPerUnit;
	mY_element(1, 0) = -y*mRatioAbsPerUnit;
	mY_element(1, 1) = y*std::pow(mRatioAbsPerUnit, 2);

	//check for inf or nan
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			if (std::isinf(mY_element.coeff(i, j).real()) || std::isinf(mY_element.coeff(i, j).imag())) {
				std::cout << mY_element << std::endl;
				std::cout << "Zl:" << y << std::endl;
				std::cout << "tap:" << mRatioAbsPerUnit << std::endl;
				std::stringstream ss;
				ss << "Transformer>>" << this->name() << ": infinite or nan values in the element Y at: " << i << "," << j;
				throw std::invalid_argument(ss.str());
			}

	//set the circuit matrix values
	Y.coeffRef(this->matrixNodeIndex(0), this->matrixNodeIndex(0)) += mY_element.coeff(0, 0);
	Y.coeffRef(this->matrixNodeIndex(0), this->matrixNodeIndex(1)) += mY_element.coeff(0, 1);
	Y.coeffRef(this->matrixNodeIndex(1), this->matrixNodeIndex(1)) += mY_element.coeff(1, 1);
	Y.coeffRef(this->matrixNodeIndex(1), this->matrixNodeIndex(0)) += mY_element.coeff(1, 0);

	SPDLOG_LOGGER_INFO(mSLog, "#### Y matrix stamping: {}", mY_element);
}


void SP::Ph1::Transformer::updateBranchFlow(VectorComp& current, VectorComp& powerflow) {
	**mCurrent = current * mBaseCurrent;
	**mActivePowerBranch = powerflow.real()*mBaseApparentPower;
	**mReactivePowerBranch = powerflow.imag()*mBaseApparentPower;
}


void SP::Ph1::Transformer::storeNodalInjection(Complex powerInjection) {
	**mActivePowerInjection = std::real(powerInjection)*mBaseApparentPower;
	**mReactivePowerInjection = std::imag(powerInjection)*mBaseApparentPower;
}

MatrixComp SP::Ph1::Transformer::Y_element() {
	return mY_element;
}

// #### MNA Section ####
void SP::Ph1::Transformer::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();

}

void SP::Ph1::Transformer::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	SPDLOG_LOGGER_INFO(mSLog, "-- Matrix Stamp ---");
	if (terminalNotGrounded(0)) {
<<<<<<< HEAD
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), 1. / mImpedance);
=======
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), 1. / mImpedance);
>>>>>>> b615f3a4 (delete virtual nodes of SP Transformer)
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			(Complex(1, 0) / mImpedance).real(), (Complex(1, 0) / mImpedance).imag(), matrixNodeIndex(0), matrixNodeIndex(0));
	}
	if (terminalNotGrounded(1)) {
<<<<<<< HEAD
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), std::pow(**mRatio,2) / mImpedance );
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			(std::pow(**mRatio,2) / mImpedance).real(), (std::pow(**mRatio,2) / mImpedance).imag(), matrixNodeIndex(1), matrixNodeIndex(1));
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -**mRatio / mImpedance );
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			(-**mRatio / mImpedance).real(), (-**mRatio / mImpedance).imag(), matrixNodeIndex(0), matrixNodeIndex(1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -**mRatio / mImpedance );
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			(-**mRatio / mImpedance).real(), (-**mRatio / mImpedance).imag(), matrixNodeIndex(1), matrixNodeIndex(0));
	}

	mSLog->flush();
=======
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), std::pow(**mRatio,2) / mImpedance );
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			(std::pow(**mRatio,2) / mImpedance).real(), (std::pow(**mRatio,2) / mImpedance).imag(), matrixNodeIndex(1), matrixNodeIndex(1));
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -**mRatio / mImpedance );
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			(-**mRatio / mImpedance).real(), (-**mRatio / mImpedance).imag(), matrixNodeIndex(0), matrixNodeIndex(1));
		Math::setMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -**mRatio / mImpedance );
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			(-**mRatio / mImpedance).real(), (-**mRatio / mImpedance).imag(), matrixNodeIndex(1), matrixNodeIndex(0));
	}
>>>>>>> b615f3a4 (delete virtual nodes of SP Transformer)
}

void SP::Ph1::Transformer::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, 
	AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes,
	Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::Transformer::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void SP::Ph1::Transformer::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v0 - v1
	(**mIntfVoltage)(0, 0) = 0.0;
<<<<<<< HEAD
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0, 0) += Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
	if (terminalNotGrounded(1)) 
		(**mIntfVoltage)(0, 0) -= Math::complexFromVectorElement(leftVector, matrixNodeIndex(1)) * **mRatio;
}

void SP::Ph1::Transformer::mnaCompUpdateCurrent(const Matrix& leftVector) {
	// primary side current flowing into node 0
	(**mIntfCurrent)(0, 0) = -(**mIntfVoltage)(0, 0) / mImpedance;
=======
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
	}
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0, 0) -= Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	}
	
}

void SP::Ph1::Transformer::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = 0.0;
	if (terminalNotGrounded(0)) {
		(**mIntfCurrent)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0)) / mImpedance;
	}
	if (terminalNotGrounded(1)) {
		(**mIntfCurrent)(0, 0) -= Math::complexFromVectorElement(leftVector, matrixNodeIndex(1)) * (**mRatio / mImpedance);
	}
>>>>>>> b615f3a4 (delete virtual nodes of SP Transformer)
}