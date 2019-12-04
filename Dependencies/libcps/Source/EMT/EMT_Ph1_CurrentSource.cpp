/** Current source
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

#include <cps/EMT/EMT_Ph1_CurrentSource.h>

using namespace CPS;

EMT::Ph1::CurrentSource::CurrentSource(String uid, String name,	Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(1,1);
	mIntfCurrent = Matrix::Zero(1,1);

	addAttribute<Complex>("I_ref", Flags::read | Flags::write);
	addAttribute<Real>("f_src", Flags::read | Flags::write);
}

PowerComponent<Real>::Ptr EMT::Ph1::CurrentSource::clone(String name) {
	auto copy = CurrentSource::make(name, mLogLevel);
	copy->setParameters(attribute<Complex>("I_ref")->get(), attribute<Real>("f_src")->get());
	return copy;
}

void EMT::Ph1::CurrentSource::setParameters(Complex currentRef, Real srcFreq) {
	attribute<Complex>("I_ref")->set(currentRef);
	attribute<Real>("f_src")->set(srcFreq);
}

void EMT::Ph1::CurrentSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mCurrentRef = attribute<Complex>("I_ref");
	mSrcFreq = attribute<Real>("f_src");
	mIntfCurrent(0,0) = Math::abs(mCurrentRef->get()) * cos(Math::phase(mCurrentRef->get()));
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph1::CurrentSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, simNode(0), -mIntfCurrent(0,0));

	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, simNode(1), mIntfCurrent(0,0));
}

void EMT::Ph1::CurrentSource::updateState(Real time) {
	Complex currentRef = mCurrentRef->get();
	Real srcFreq = mSrcFreq->get();
	if (srcFreq > 0)
		mIntfCurrent(0,0) = Math::abs(currentRef) * cos(time * 2.*PI*srcFreq + Math::phase(currentRef));
	else
		mIntfCurrent(0,0) = currentRef.real();
}

void EMT::Ph1::CurrentSource::MnaPreStep::execute(Real time, Int timeStepCount) {
	mCurrentSource.updateState(time);
	mCurrentSource.mnaApplyRightSideVectorStamp(mCurrentSource.mRightVector);
}

void EMT::Ph1::CurrentSource::MnaPostStep::execute(Real time, Int timeStepCount) {
	mCurrentSource.mnaUpdateVoltage(*mLeftVector);
}

void EMT::Ph1::CurrentSource::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0,0) = 0;
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = Math::realFromVectorElement(leftVector, simNode(0));
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::realFromVectorElement(leftVector, simNode(1));
}
