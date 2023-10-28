/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_Load.h>

using namespace CPS;

// #### General ####
// please note that P,Q values can not be passed inside constructor since P,Q are currently read from the terminal,
// and these values are not yet assigned to the terminals when this constructor was called in reader.
SP::Ph1::Load::Load(String uid, String name, Logger::Level logLevel)
	: CompositePowerComp<Complex>(uid, name, false, true, logLevel),
	mActivePowerPerUnit(mAttributes->create<Real>("P_pu")),
	mReactivePowerPerUnit(mAttributes->create<Real>("Q_pu")),
	mActivePower(mAttributes->createDynamic<Real>("P")), //Made dynamic so it can be imported through InterfaceVillas
	mReactivePower(mAttributes->createDynamic<Real>("Q")), //Made dynamic so it can be imported through InterfaceVillas
	mNomVoltage(mAttributes->create<Real>("V_nom")) {

	SPDLOG_LOGGER_INFO(mSLog, "Create {} of type {}", **mName, this->type());
	mSLog->flush();
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
    setTerminalNumber(1);
}

void SP::Ph1::Load::setParameters(Real activePower, Real reactivePower) {
	**mActivePower = activePower;
	**mReactivePower = reactivePower;
	initPowerFromTerminal = false;

	SPDLOG_LOGGER_INFO(mSLog, 
		"Active Power={}[W]"
		"\nReactive Power={} [VAr]", 
		**mActivePower, **mReactivePower);
	mSLog->flush();
}

void SP::Ph1::Load::setParameters(Real activePower, Real reactivePower, Real nominalVoltage) {
	setParameters(activePower, reactivePower);
	**mNomVoltage = nominalVoltage;
	initVoltageFromNode = false;

	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
	mSLog->flush();
}

 // #### Powerflow section ####
void SP::Ph1::Load::calculatePerUnitParameters(Real baseApparentPower, Real baseOmega) {
	SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}", **mName);
	mBaseApparentPower = baseApparentPower;
	mBaseOmega = baseOmega;
    SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]  Base Omega={} [1/s]", mBaseApparentPower, mBaseOmega);

	**mActivePowerPerUnit = **mActivePower / mBaseApparentPower;
	**mReactivePowerPerUnit = **mReactivePower /mBaseApparentPower;
	SPDLOG_LOGGER_INFO(mSLog, "Active Power={} [pu] Reactive Power={} [pu]", **mActivePowerPerUnit, **mReactivePowerPerUnit);
	mSLog->flush();
}


void SP::Ph1::Load::modifyPowerFlowBusType(PowerflowBusType powerflowBusType) {
	switch (powerflowBusType)
	{
	case CPS::PowerflowBusType::PV:
		throw std::invalid_argument(" Power flow bus type error, load currently cannot be set as PVNode ");
		break;
	case CPS::PowerflowBusType::PQ:
		mPowerflowBusType = powerflowBusType;
		break;
	case CPS::PowerflowBusType::VD:
		throw std::invalid_argument(" Power flow bus type error, load cannot be set as VDNode ");
		break;
	case CPS::PowerflowBusType::None:
		break;
	default:
		throw std::invalid_argument(" Invalid power flow bus type ");
		break;
	}
};


void SP::Ph1::Load::updatePQ(Real time) {
	if (mLoadProfile.weightingFactors.empty()) {
		**mActivePower = mLoadProfile.pqData.find(time)->second.p;
		**mReactivePower = mLoadProfile.pqData.find(time)->second.q;
	} else {
		Real wf = mLoadProfile.weightingFactors.find(time)->second;
		///THISISBAD: P_nom and Q_nom do not exist as attributes
		Real P_new = this->attributeTyped<Real>("P_nom")->get()*wf;
		Real Q_new = this->attributeTyped<Real>("Q_nom")->get()*wf;
		**mActivePower = P_new;
		**mReactivePower = Q_new;
	}
};


void SP::Ph1::Load::initializeFromNodesAndTerminals(Real frequency) {

	if(initPowerFromTerminal){
		setParameters(
			mTerminals[0]->singleActivePower(),
			mTerminals[0]->singleReactivePower(),
			std::abs(mTerminals[0]->initialSingleVoltage()));
	}
	if (initVoltageFromNode) {
		**mNomVoltage = std::abs(initialSingleVoltage(0));
		SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
	}

	// instantiate subResistor for active power consumption
	if (**mActivePower != 0) {
		mResistance = std::pow(**mNomVoltage, 2) / **mActivePower;
		mConductance = 1.0 / mResistance;
		mSubResistor = std::make_shared<SP::Ph1::Resistor>(**mUID + "_res", **mName + "_res", Logger::Level::off);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	}

	if (**mReactivePower != 0)
		mReactance = std::pow(**mNomVoltage, 2) / **mReactivePower;
	else
		mReactance = 0;

	// instantiate subInductor or subCapacitor for reactive power consumption
	if (mReactance > 0) {
		mInductance = mReactance / (2 * PI * frequency);
		mSubInductor = std::make_shared<SP::Ph1::Inductor>(**mUID + "_res", **mName + "_ind", Logger::Level::off);
		mSubInductor->setParameters(mInductance);
		mSubInductor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubInductor->initialize(mFrequencies);
		mSubInductor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	} else if (mReactance < 0) {
		mCapacitance = -1 / (2 * PI * frequency) / mReactance;
		mSubCapacitor = std::make_shared<SP::Ph1::Capacitor>(**mUID + "_res", **mName + "_cap", Logger::Level::off);
		mSubCapacitor->setParameters(mCapacitance);
		mSubCapacitor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubCapacitor, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	}

	(**mIntfVoltage)(0, 0) = mTerminals[0]->initialSingleVoltage();
	(**mIntfCurrent)(0, 0) = std::conj(Complex(attributeTyped<Real>("P")->get(), attributeTyped<Real>("Q")->get()) / (**mIntfVoltage)(0, 0));

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		Logger::phasorToString((**mIntfCurrent)(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)));
	SPDLOG_LOGGER_INFO(mSLog, 
		"Updated parameters according to powerflow:\n"
		"Active Power={} [W] Reactive Power={} [VAr]", attributeTyped<Real>("P")->get(), attributeTyped<Real>("Q")->get());
	mSLog->flush();
}


// #### MNA section ####

void SP::Ph1::Load::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfCurrent);
	modifiedAttributes.push_back(mIntfVoltage);
};

void SP::Ph1::Load::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void SP::Ph1::Load::mnaCompUpdateVoltage(const Matrix& leftVector) {
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void SP::Ph1::Load::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = 0;

	for (auto& subc : mSubComponents) {
		(**mIntfCurrent)(0, 0) += subc->intfCurrent()(0, 0);
	}
}
