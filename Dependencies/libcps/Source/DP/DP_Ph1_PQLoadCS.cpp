/** PQ Load
 *
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

#include <cps/DP/DP_Ph1_PQLoadCS.h>

using namespace CPS;

DP::Ph1::PQLoadCS::PQLoadCS(String uid, String name,
	Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setTerminalNumber(1);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Real>("P", Flags::read | Flags::write);
	addAttribute<Real>("Q", Flags::read | Flags::write);
	addAttribute<Real>("V_nom", Flags::read | Flags::write);
	attribute<Real>("P")->set(0.);
	attribute<Real>("Q")->set(0.);
}

DP::Ph1::PQLoadCS::PQLoadCS(String uid, String name,
	Real activePower, Real reactivePower, Real nomVolt,
	Logger::Level logLevel)
	: PQLoadCS(uid, name, logLevel) {

	setParameters(activePower, reactivePower, nomVolt);
}

DP::Ph1::PQLoadCS::PQLoadCS(String name, Logger::Level logLevel)
	: PQLoadCS(name, name, logLevel) {
}

DP::Ph1::PQLoadCS::PQLoadCS(String name,
	Real activePower, Real reactivePower, Real nomVolt,
	Logger::Level logLevel)
	: PQLoadCS(name, name, activePower, reactivePower, nomVolt, logLevel) {
}

void DP::Ph1::PQLoadCS::setParameters(Real activePower, Real reactivePower, Real nomVolt) {
	attribute<Real>("P")->set(activePower);
	attribute<Real>("Q")->set(reactivePower);
	attribute<Real>("V_nom")->set(nomVolt);

	parametersSet = true;
}

PowerComponent<Complex>::Ptr DP::Ph1::PQLoadCS::clone(String name) {
	auto copy = PQLoadCS::make(name, mLogLevel);
	copy->setParameters(attribute<Real>("P")->get(), attribute<Real>("Q")->get(), attribute<Real>("V_nom")->get());
	return copy;
}

void DP::Ph1::PQLoadCS::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	mActivePower = attribute<Real>("P");
	mReactivePower = attribute<Real>("Q");
	mNomVoltage = attribute<Real>("V_nom");
	// Get power from Terminals if it was not set previously.
	if (mActivePower->get() == 0 && mReactivePower->get() == 0 && !parametersSet) {
		mActivePower->set(mTerminals[0]->singleActivePower());
		mReactivePower->set(mTerminals[0]->singleReactivePower());
		mNomVoltage->set(std::abs(mTerminals[0]->initialSingleVoltage()));
	}
	Complex power = Complex(mActivePower->get(), mReactivePower->get());

	Complex current;
	if (mNomVoltage != 0)
		current = std::conj(power / mNomVoltage->get());
	else
		current = 0;

	mSubCurrentSource = std::make_shared<DP::Ph1::CurrentSource>(mName + "_cs", mLogLevel);
	mSubCurrentSource->setParameters(current);
	mCurrentSourceRef = mSubCurrentSource->attribute<Complex>("I_ref");
	// A positive power should result in a positive current to ground.
	mSubCurrentSource->connect({ mTerminals[0]->node(), Node::GND });
	mSubCurrentSource->initializeFromPowerflow(frequency);
	updateIntfValues();

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nCurrent set point: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(current));
}

void DP::Ph1::PQLoadCS::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	mSubCurrentSource->mnaInitialize(omega, timeStep, leftVector);
	setAttributeRef("right_vector", mSubCurrentSource->attribute("right_vector"));
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this));
	for (auto task : mSubCurrentSource->mnaTasks())
		mMnaTasks.push_back(task);
}

void DP::Ph1::PQLoadCS::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubCurrentSource->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::PQLoadCS::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubCurrentSource->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::PQLoadCS::updateSetPoint() {
	// Calculate new current set point.
	Complex power = { mActivePower->get(), mReactivePower->get()};
	Complex current = power / mNomVoltage->get();
	//Complex current = power / mIntfVoltage(0,0);

	mCurrentSourceRef->set(std::conj(current));
	mSLog->debug(
		"\n--- update set points ---"
		"\npower: {:s}"
		"\nCurrent: {:s}",
		Logger::phasorToString(power),
		Logger::phasorToString(std::conj(current)));
}

void DP::Ph1::PQLoadCS::MnaPreStep::execute(Real time, Int timeStepCount) {
	mLoad.updateSetPoint();
}

void DP::Ph1::PQLoadCS::updateIntfValues() {
	mIntfCurrent = mSubCurrentSource->intfCurrent();
	mIntfVoltage = mSubCurrentSource->intfVoltage();
}

void DP::Ph1::PQLoadCS::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLoad.updateIntfValues();
}

