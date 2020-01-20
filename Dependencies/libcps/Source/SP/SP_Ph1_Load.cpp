/**
 * @file
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
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

#include <cps/SP/SP_Ph1_Load.h>

using namespace CPS;

// #### General ####
// please note that P,Q values can not be passed inside constructor since P,Q are currently read from the terminal,
// and these values are not yet assigned to the terminals when this constructor was called in reader.
SP::Ph1::Load::Load(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {

	mSLog->info("Create {} of type {}", mName, this->type());
	mSLog->flush();
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);
    setTerminalNumber(1);

	addAttribute<Real>("P_pu", &mActivePowerPerUnit, Flags::read | Flags::write);
	addAttribute<Real>("Q_pu", &mReactivePowerPerUnit, Flags::read | Flags::write);
	addAttribute<Real>("P", &mActivePower, Flags::read | Flags::write);
	addAttribute<Real>("Q", &mReactivePower, Flags::read | Flags::write);
	addAttribute<Real>("V_nom", &mNomVoltage, Flags::read | Flags::write);
};


void SP::Ph1::Load::setParameters(Real activePower, Real reactivePower, Real nominalVoltage) {
	mActivePower = activePower;
	mReactivePower = reactivePower;
	mPower = { mActivePower, mReactivePower };
	mNomVoltage = nominalVoltage;

	mSLog->info("Active Power={} [W] Reactive Power={} [VAr]", mActivePower, mReactivePower);
	mSLog->flush();

	parametersSet = true;
}


PowerComponent<Complex>::Ptr SP::Ph1::Load::clone(String name) {
	// everything set by initializeFromPowerflow
	return Load::make(name, mLogLevel);
}


 // #### Powerflow section ####
void SP::Ph1::Load::calculatePerUnitParameters(Real baseApparentPower, Real baseOmega) {
	mSLog->info("#### Calculate Per Unit Parameters for {}", mName); 
	mBaseApparentPower = baseApparentPower;
	mBaseOmega = baseOmega;
    mSLog->info("Base Power={} [VA]  Base Omega={} [1/s]", mBaseApparentPower, mBaseOmega);

	mActivePowerPerUnit = mActivePower/mBaseApparentPower;
	mReactivePowerPerUnit = mReactivePower/mBaseApparentPower;
	mSLog->info("Active Power={} [pu] Reactive Power={} [pu]", mActivePowerPerUnit, mReactivePowerPerUnit);
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
		this->attribute<Real>("P")->set(mLoadProfile.pqData.find(time)->second.p);
		this->attribute<Real>("Q")->set(mLoadProfile.pqData.find(time)->second.q);
	} else {
		Real wf = mLoadProfile.weightingFactors.find(time)->second;
		Real P_new = this->attribute<Real>("P_nom")->get()*wf;
		Real Q_new = this->attribute<Real>("Q_nom")->get()*wf;
		this->attribute<Real>("P")->set(P_new);
		this->attribute<Real>("Q")->set(Q_new);
	}
};


void SP::Ph1::Load::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	mActivePower = mTerminals[0]->singleActivePower();
	mReactivePower = mTerminals[0]->singleReactivePower();
	mPower = { mActivePower, mReactivePower };
	mNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());

	// instantiate subResistor for active power consumption
	if (mActivePower != 0) {
		mResistance = std::pow(mNomVoltage, 2) / mActivePower;
		mConductance = 1.0 / mResistance;
		mSubResistor = std::make_shared<SP::Ph1::Resistor>(mUID + "_res", mName + "_res", Logger::Level::off);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ Node::GND, mTerminals[0]->node() });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromPowerflow(frequency);
	}

	if (mReactivePower != 0)
		mReactance = std::pow(mNomVoltage, 2) / mReactivePower;
	else
		mReactance = 0;

	// instantiate subInductor or subCapacitor for reactive power consumption
	if (mReactance > 0) {
		mInductance = mReactance / (2 * PI * frequency);
		mSubInductor = std::make_shared<SP::Ph1::Inductor>(mUID + "_res", mName + "_ind", Logger::Level::off);
		mSubInductor->setParameters(mInductance);
		mSubInductor->connect({ Node::GND, mTerminals[0]->node() });
		mSubInductor->initialize(mFrequencies);
		mSubInductor->initializeFromPowerflow(frequency);
	} else if (mReactance < 0) {
		mCapacitance = -1 / (2 * PI * frequency) / mReactance;
		mSubCapacitor = std::make_shared<SP::Ph1::Capacitor>(mUID + "_res", mName + "_cap", Logger::Level::off);
		mSubCapacitor->setParameters(mCapacitance);
		mSubCapacitor->connect({ Node::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromPowerflow(frequency);
	}

	mIntfVoltage(0, 0) = mTerminals[0]->initialSingleVoltage();
	mIntfCurrent(0, 0) = std::conj(Complex(mActivePower, mReactivePower) / mIntfVoltage(0, 0));

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)));
	mSLog->info(
		"Updated parameters according to powerflow:\n"
		"Active Power={} [W] Reactive Power={} [VAr]", mActivePower, mReactivePower);
	mSLog->flush();
}


// #### MNA section ####
void SP::Ph1::Load::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
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
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}


void SP::Ph1::Load::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (mSubResistor)
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubInductor)
		mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubCapacitor)
		mSubCapacitor->mnaApplySystemMatrixStamp(systemMatrix);
}


void SP::Ph1::Load::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLoad.mnaUpdateVoltage(*mLeftVector);
	mLoad.mnaUpdateCurrent(*mLeftVector);
}


void SP::Ph1::Load::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, simNode(0));
}


void SP::Ph1::Load::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = 0;
	if (mSubResistor)
		mIntfCurrent(0, 0) += mSubResistor->intfCurrent()(0, 0);
	if (mSubInductor)
		mIntfCurrent(0, 0) += mSubInductor->intfCurrent()(0, 0);
	if (mSubCapacitor)
		mIntfCurrent(0, 0) += mSubCapacitor->intfCurrent()(0, 0);
}
