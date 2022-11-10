/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_RXLoad.h>

using namespace CPS;

EMT::Ph3::RXLoad::RXLoad(String uid, String name, Logger::Level logLevel)
	: CompositePowerComp<Real>(uid, name, true, true, logLevel),
	mActivePower(mAttributes->create<Matrix>("P")),
	mReactivePower(mAttributes->create<Matrix>("Q")),
	mNomVoltage(mAttributes->create<Real>("V_nom")) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
	mSLog->flush();

	//DAE Solver
	mIntfDerCurrent = Matrix::Zero(3, 1);
}

EMT::Ph3::RXLoad::RXLoad(String name,
	Logger::Level logLevel)
	: RXLoad(name, name, logLevel) {
}

EMT::Ph3::RXLoad::RXLoad(String name,
	Matrix activePower, Matrix reactivePower, Real volt,
	Logger::Level logLevel)
	: RXLoad(name, logLevel) {
	**mActivePower = activePower;
	**mReactivePower = reactivePower;
	mPower = MatrixComp::Zero(3,3);
	mPower <<
		Complex((**mActivePower)(0, 0), (**mReactivePower)(0, 0)), Complex((**mActivePower)(0, 1), (**mReactivePower)(0, 1)), Complex((**mActivePower)(0, 2), (**mReactivePower)(0, 2)),
		Complex((**mActivePower)(1, 0), (**mReactivePower)(1, 0)), Complex((**mActivePower)(1, 1), (**mReactivePower)(1, 1)), Complex((**mActivePower)(1, 2), (**mReactivePower)(1, 2)),
		Complex((**mActivePower)(2, 0), (**mReactivePower)(2, 0)), Complex((**mActivePower)(2, 1), (**mReactivePower)(2, 1)), Complex((**mActivePower)(2, 2), (**mReactivePower)(2, 2));

	**mNomVoltage = volt;
	initPowerFromTerminal = false;
}

void EMT::Ph3::RXLoad::setParameters(Matrix activePower, Matrix reactivePower, Real volt) {
	**mActivePower = activePower;
	**mReactivePower = reactivePower;

	// complex power
	mPower = MatrixComp::Zero(3, 3);
	mPower(0, 0) = { (**mActivePower)(0, 0), (**mReactivePower)(0, 0) };
	mPower(1, 1) = { (**mActivePower)(1, 1), (**mReactivePower)(1, 1) };
	mPower(2, 2) = { (**mActivePower)(2, 2), (**mReactivePower)(2, 2) };

	**mNomVoltage = volt;

	SPDLOG_LOGGER_INFO(mSLog, "\nActive Power [W]: {}"
			"\nReactive Power [VAr]: {}",
			Logger::matrixToString(**mActivePower),
			Logger::matrixToString(**mReactivePower));
	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);

	initPowerFromTerminal = false;
}

SimPowerComp<Real>::Ptr EMT::Ph3::RXLoad::clone(String name) {
	// everything set by initializeFromNodesAndTerminals
	return RXLoad::make(name, mLogLevel);
}

void EMT::Ph3::RXLoad::initializeFromNodesAndTerminals(Real frequency) {

		if (initPowerFromTerminal) {
		**mActivePower = Matrix::Zero(3, 3);
		(**mActivePower)(0, 0) = mTerminals[0]->singleActivePower() / 3.;
		(**mActivePower)(1, 1) = mTerminals[0]->singleActivePower() / 3.;
		(**mActivePower)(2, 2) = mTerminals[0]->singleActivePower() / 3.;

		**mReactivePower = Matrix::Zero(3, 3);
		(**mReactivePower)(0, 0) = mTerminals[0]->singleReactivePower() / 3.;
		(**mReactivePower)(1, 1) = mTerminals[0]->singleReactivePower() / 3.;
		(**mReactivePower)(2, 2) = mTerminals[0]->singleReactivePower() / 3.;

		// complex power
		mPower = MatrixComp::Zero(3, 3);
		mPower(0, 0) = { (**mActivePower)(0, 0), (**mReactivePower)(0, 0) };
		mPower(1, 1) = { (**mActivePower)(1, 1), (**mReactivePower)(1, 1) };
		mPower(2, 2) = { (**mActivePower)(2, 2), (**mReactivePower)(2, 2) };

		**mNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());

		SPDLOG_LOGGER_INFO(mSLog, "\nActive Power [W]: {}"
					"\nReactive Power [VAr]: {}",
					Logger::matrixToString(**mActivePower),
					Logger::matrixToString(**mReactivePower));
		SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
	}

	if ((**mActivePower)(0,0) != 0) {
		mResistance = std::pow(**mNomVoltage / sqrt(3), 2) * (**mActivePower).inverse();
		mSubResistor = std::make_shared<EMT::Ph3::Resistor>(**mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	if ((**mReactivePower)(0, 0) != 0)
		mReactance = std::pow(**mNomVoltage / sqrt(3), 2) * (**mReactivePower).inverse();
	else
		mReactance = Matrix::Zero(1, 1);

	if (mReactance(0,0) > 0) {
		mInductance = mReactance / (2 * PI * frequency);

		mSubInductor = std::make_shared<EMT::Ph3::Inductor>(**mName + "_ind", mLogLevel);
		mSubInductor->setParameters(mInductance);
		mSubInductor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubInductor->initialize(mFrequencies);
		mSubInductor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}
	else if (mReactance(0,0) < 0) {
		mCapacitance = -1 / (2 * PI * frequency) * mReactance.inverse();

		mSubCapacitor = std::make_shared<EMT::Ph3::Capacitor>(**mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(mCapacitance);
		mSubCapacitor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubCapacitor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * mTerminals[0]->initialSingleVoltage();
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	**mIntfVoltage = vInitABC.real();

	MatrixComp iInitABC = MatrixComp::Zero(3, 1);
	// v i^T* = S
	// v^T v i^T* = v^T S
	// i^T*= (|v|^2)^(-1) v^T S

	Complex v_ = vInitABC(0, 0)*vInitABC(0, 0) + vInitABC(1, 0)*vInitABC(1, 0) + vInitABC(2, 0)*vInitABC(2, 0);
	MatrixComp rhs_ = Complex(1, 0) / v_ * vInitABC.transpose() * mPower;
	iInitABC = rhs_.conjugate().transpose();
	**mIntfCurrent = iInitABC.real();

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nActive Power: {:s}"
		"\nReactive Power: {:s}"
		"\nResistance: {:s}"
		"\nReactance: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(**mIntfVoltage),
		Logger::matrixToString(**mIntfCurrent),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
		Logger::matrixToString(**mActivePower),
		Logger::matrixToString(**mReactivePower),
		Logger::matrixToString(mResistance),
		Logger::matrixToString(mReactance));
	mSLog->flush();

}

void EMT::Ph3::RXLoad::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	modifiedAttributes.push_back(mRightVector);
};

void EMT::Ph3::RXLoad::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfCurrent);
	modifiedAttributes.push_back(mIntfVoltage);
};

void EMT::Ph3::RXLoad::mnaParentPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::RXLoad::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::RXLoad::mnaCompUpdateVoltage(const Matrix& leftVector) {
	**mIntfVoltage = Matrix::Zero(3, 1);
	(**mIntfVoltage)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	(**mIntfVoltage)(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
	(**mIntfVoltage)(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}

void EMT::Ph3::RXLoad::mnaCompUpdateCurrent(const Matrix& leftVector) {
	**mIntfCurrent = Matrix::Zero(3, 1);
	for (auto& subc : mSubComponents) {
		**mIntfCurrent += subc->intfCurrent();
	}
}

// #### DAE functions ####

void EMT::Ph3::RXLoad::daeInitialize(double time, double state[], double dstate_dt[], 
	double absoluteTolerances[], double stateVarTypes[], int& offset) {

	updateMatrixNodeIndices();

	(**mIntfCurrent)(0,0) = 0.0;
	(**mIntfCurrent)(1,0) = 0.0;
	(**mIntfCurrent)(2,0) = 0.0;

	if (mReactance(0,0) > 0) {
		**mIntfCurrent += mSubInductor->intfCurrent();
		**mIntfCurrent += mSubResistor->intfCurrent();
	}
	if (mReactance(0,0) < 0) {
		**mIntfCurrent += mSubCapacitor->intfCurrent();
		**mIntfCurrent += mSubResistor->intfCurrent();
	}

	// state variable is the inductor current
	// init current throw inductor: i_L = i-i_r
	state[offset] = (**mIntfCurrent)(0,0) - (**mIntfVoltage)(0,0) * mConductance(0,0);
	dstate_dt[offset]   = (**mIntfVoltage)(0,0) / mInductance(0,0);
	state[offset+1] = (**mIntfCurrent)(1,0) - (**mIntfVoltage)(1,0) * mConductance(1,1);
	dstate_dt[offset+1] = (**mIntfVoltage)(1,0) / mInductance(1,1);
	state[offset+2] = (**mIntfCurrent)(2,0) - (**mIntfVoltage)(2,0) * mConductance(2,2);
	dstate_dt[offset+2] = (**mIntfVoltage)(2,0) / mInductance(2, 2);

	mIntfDerCurrent(0, 0) = dstate_dt[offset];
	mIntfDerCurrent(1, 0) = dstate_dt[offset+1];
	mIntfDerCurrent(2, 0) = dstate_dt[offset+2];
	//set state variable as differential variable
	stateVarTypes[offset]   = 0.0;
	stateVarTypes[offset+1] = 0.0;
	stateVarTypes[offset+2] = 0.0;

	//set absolute tolerance
	absoluteTolerances[offset] = mAbsTolerance;
	absoluteTolerances[offset+1] = mAbsTolerance;
	absoluteTolerances[offset+2] = mAbsTolerance;
	
	mSLog->info(
		"\n--- daeInitialize ---"
		"\nmReactance(0,0) > 0  --> state variable are inductor currents"
		"\nAdded current-phase1 through the inductor of RXLoad '{:s}' to state vector, initial value={:f}A"
		"\nAdded current-phase2 through the inductor of RXLoad '{:s}' to state vector, initial value={:f}A"
		"\nAdded current-phase3 through the inductor of RXLoad '{:s}' to state vector, initial value={:f}A"
		"\nAdded derivative of current-phase1 through the inductor of RXLoad '{:s}' to derivative state vector, initial value={:f}"
		"\nAdded derivative of current-phase2 through the inductor of RXLoad '{:s}' to derivative state vector, initial value={:f}"
		"\nAdded derivative of current-phase3 through the inductor of RXLoad '{:s}' to derivative state vector, initial value={:f}"
		"\nInitial current through the resistor-ph1 of RXLoad '{:s}'={:f}A"
		"\nInitial current through the resistor-ph2 of RXLoad '{:s}'={:f}A"
		"\nInitial current through the resistor-ph3 of RXLoadd '{:s}'={:f}A"
		"\nState variables set as differential"
		"\nAbsolute tolerances={:f}"
		"\n--- daeInitialize finished ---",
		this->name(), state[offset],
		this->name(), state[offset+1],
		this->name(), state[offset+2],
		this->name(), dstate_dt[offset],
		this->name(), dstate_dt[offset+1],
		this->name(), dstate_dt[offset+2],
		this->name(), (**mIntfVoltage)(0,0) * mConductance(0,0),
		this->name(), (**mIntfVoltage)(1,0) * mConductance(1,1),
		this->name(), (**mIntfVoltage)(2,0) * mConductance(2,2),
		absoluteTolerances[offset]
	);
	mSLog->flush();
	offset+=3;
}

void EMT::Ph3::RXLoad::daeResidual(double sim_time, 
	const double state[], const double dstate_dt[], 
	double resid[], std::vector<int>& off) {

	int c_offset = off[0]+off[1]; //current offset for component
	int pos_node1 = matrixNodeIndex(0, 0);
	int pos_node2 = matrixNodeIndex(0, 1);
	int pos_node3 = matrixNodeIndex(0, 2);

	if (mReactance(0,0) > 0) {
		resid[c_offset]   = state[pos_node1]  - mInductance(0,0) * dstate_dt[c_offset];
		resid[c_offset+1] = state[pos_node2]  - mInductance(1,1) * dstate_dt[c_offset+1];
		resid[c_offset+2] = state[pos_node3]  - mInductance(2,2) * dstate_dt[c_offset+2];
		resid[pos_node1] += state[c_offset]   + state[pos_node1] * mConductance(0,0);
		resid[pos_node2] += state[c_offset+1] + state[pos_node2] * mConductance(1,1);
		resid[pos_node3] += state[c_offset+2] + state[pos_node3] * mConductance(2,2);
	}
	else if (mReactance(0,0) < 0) {
		resid[c_offset]   = state[pos_node1] - state[c_offset];
		resid[c_offset+1] = state[pos_node2] - state[c_offset+1];
		resid[c_offset+2] = state[pos_node3] - state[c_offset+2];
		resid[pos_node1] += mCapacitance(0,0)*dstate_dt[c_offset]   + state[pos_node1] * mConductance(0,0);
		resid[pos_node2] += mCapacitance(1,1)*dstate_dt[c_offset+1] + state[pos_node2] * mConductance(1,1);
		resid[pos_node3] += mCapacitance(2,2)*dstate_dt[c_offset+2] + state[pos_node3] * mConductance(2,2);
	}	
	// else if (mReactance(0,0) == 0) {}
	
	mSLog->debug(
		"\n\n--- daeResidual RXLoad - name: {:s} SimStep = {:f} ---"
		"\nresid[c_offset]   = state[pos_node1] - mInductance(0,0)*dstate_dt[c_offset]   = {:f} - {:f}*{:f} = {:f}"
		"\nresid[c_offset+1] = state[pos_node2] - mInductance(1,1)*dstate_dt[c_offset+1] = {:f} - {:f}*{:f} = {:f}"
		"\nresid[c_offset+2] = state[pos_node3] - mInductance(2,2)*dstate_dt[c_offset+2] = {:f} - {:f}*{:f} = {:f}"

		"\nupdate nodal equations:"
		"\nresid[pos_node1] += state[c_offset]   + state[pos_node1]*mConductance(0,0) = {:f} + {:f}*{:f} = {:f}"
		"\nresid[pos_node2] += state[c_offset+1] + state[pos_node2]*mConductance(1,1) = {:f} + {:f}*{:f} = {:f}"
		"\nresid[pos_node3] += state[c_offset+2] + state[pos_node3]*mConductance(2,2) = {:f} + {:f}*{:f} = {:f}",

		this->name(), sim_time,
		state[pos_node1], mInductance(0,0), dstate_dt[c_offset], resid[c_offset],
		state[pos_node2], mInductance(1,1), dstate_dt[c_offset+1], resid[c_offset+1],
		state[pos_node3], mInductance(2,2), dstate_dt[c_offset+2], resid[c_offset+2],
		state[c_offset], state[pos_node1], mConductance(0,0), resid[pos_node1],
		state[c_offset+1], state[pos_node2], mConductance(1,1), resid[pos_node2],
		state[c_offset+2], state[pos_node3], mConductance(2,2), resid[pos_node3]
	);
	mSLog->flush();

	off[1] += 3;
}

void EMT::Ph3::RXLoad::daePostStep(double Nexttime, const double state[], 
	const double dstate_dt[], int& offset) {
	
	(**mIntfVoltage)(0, 0) = state[matrixNodeIndex(0, 0)];
	(**mIntfVoltage)(1, 0) = state[matrixNodeIndex(0, 1)];
	(**mIntfVoltage)(2, 0) = state[matrixNodeIndex(0, 2)];

	if (mReactance(0,0) > 0) {
		(**mIntfCurrent)(0, 0) = state[offset]   + state[matrixNodeIndex(0, 0)] * mConductance(0,0);
		(**mIntfCurrent)(1, 0) = state[offset+1] + state[matrixNodeIndex(0, 1)] * mConductance(1,1);
		(**mIntfCurrent)(2, 0) = state[offset+2] + state[matrixNodeIndex(0, 2)] * mConductance(2,2);
	}
	else if (mReactance(0,0) < 0)
	{
		(**mIntfCurrent)(0, 0) = mCapacitance(0,0) * dstate_dt[offset]   + state[matrixNodeIndex(0, 0)] * mConductance(0,0);
		(**mIntfCurrent)(1, 0) = mCapacitance(1,1) * dstate_dt[offset+1] + state[matrixNodeIndex(0, 1)] * mConductance(1,1);
		(**mIntfCurrent)(2, 0) = mCapacitance(2,2) * dstate_dt[offset+2] + state[matrixNodeIndex(0, 2)] * mConductance(2,2);
	}
	mIntfDerCurrent(0, 0) = dstate_dt[offset];
	mIntfDerCurrent(1, 0) = dstate_dt[offset+1];
	mIntfDerCurrent(2, 0) = dstate_dt[offset+2];
	//else (mReactance==0.0) {}
	offset+=3;
}


