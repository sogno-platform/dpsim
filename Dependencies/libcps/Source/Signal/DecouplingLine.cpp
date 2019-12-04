/** Signalling part of a decoupling transmission line
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
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

#include <cps/Signal/DecouplingLine.h>

using namespace CPS;
using namespace CPS::DP::Ph1;
using namespace CPS::Signal;

DecouplingLine::DecouplingLine(String name, Node<Complex>::Ptr node1, Node<Complex>::Ptr node2,
	Real resistance, Real inductance, Real capacitance, Logger::Level logLevel) :
	SignalComponent(name, name, logLevel),
	mResistance(resistance), mInductance(inductance), mCapacitance(capacitance) {
	addAttribute<Matrix>("states", &mStates);
	addAttribute<Complex>("i_src1", &mSrcCur1Ref, Flags::read);
	addAttribute<Complex>("i_src2", &mSrcCur2Ref, Flags::read);

	mSurgeImpedance = sqrt(inductance / capacitance);
	mDelay = sqrt(inductance * capacitance);
	mSLog->info("surge impedance: {}", mSurgeImpedance);
	mSLog->info("delay: {}", mDelay);

	mRes1 = Resistor::make(name + "_r1", logLevel);
	mRes1->setParameters(mSurgeImpedance + resistance / 4);
	mRes1->connect({node1, Node<Complex>::GND});
	mRes2 = Resistor::make(name + "_r2", logLevel);
	mRes2->setParameters(mSurgeImpedance + resistance / 4);
	mRes2->connect({node2, Node<Complex>::GND});

	mSrc1 = CurrentSource::make(name + "_i1", logLevel);
	mSrc1->setParameters(0);
	mSrc1->connect({node1, Node<Complex>::GND});
	mSrcCur1 = mSrc1->attributeComplex("I_ref");
	mSrc2 = CurrentSource::make(name + "_i2", logLevel);
	mSrc2->setParameters(0);
	mSrc2->connect({node2, Node<Complex>::GND});
	mSrcCur2 = mSrc2->attributeComplex("I_ref");
}

void DecouplingLine::initialize(Real omega, Real timeStep) {
	if (mDelay < timeStep)
		throw SystemError("Timestep too large for decoupling");

	mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
	mAlpha = 1 - (mBufSize - mDelay / timeStep);
	mSLog->info("bufsize {} alpha {}", mBufSize, mAlpha);

	Complex volt1 = mRes1->initialSingleVoltage(0);
	Complex volt2 = mRes2->initialSingleVoltage(0);
	// TODO different initialization for lumped resistance?
	Complex initAdmittance = 1. / Complex(mResistance, omega * mInductance) + Complex(0, omega * mCapacitance / 2);
	Complex cur1 = volt1 * initAdmittance - volt2 / Complex(mResistance, omega * mInductance);
	Complex cur2 = volt2 * initAdmittance - volt1 / Complex(mResistance, omega * mInductance);
	mSLog->info("initial voltages: v_k {} v_m {}", volt1, volt2);
	mSLog->info("initial currents: i_km {} i_mk {}", cur1, cur2);
	mVolt1.resize(mBufSize, volt1);
	mVolt2.resize(mBufSize, volt2);
	mCur1.resize(mBufSize, cur1);
	mCur2.resize(mBufSize, cur2);
}

Complex DecouplingLine::interpolate(std::vector<Complex>& data) {
	// linear interpolation of the nearest values
	Complex c1 = data[mBufIdx];
	Complex c2 = mBufIdx == mBufSize-1 ? data[0] : data[mBufIdx+1];
	return mAlpha * c1 + (1-mAlpha) * c2;
}

void DecouplingLine::step(Real time, Int timeStepCount) {
	Complex volt1 = interpolate(mVolt1);
	Complex volt2 = interpolate(mVolt2);
	Complex cur1 = interpolate(mCur1);
	Complex cur2 = interpolate(mCur2);

	if (timeStepCount == 0) {
		// bit of a hack for proper initialization
		mSrcCur1Ref = cur1 - volt1 / (mSurgeImpedance + mResistance / 4);
		mSrcCur1Ref = cur2 - volt2 / (mSurgeImpedance + mResistance / 4);
	} else {
		// Update currents
		Real denom = (mSurgeImpedance + mResistance/4) * (mSurgeImpedance + mResistance/4);
		mSrcCur1Ref = -mSurgeImpedance / denom * (volt2 + (mSurgeImpedance - mResistance/4) * cur2)
			- mResistance/4 / denom * (volt1 + (mSurgeImpedance - mResistance/4) * cur1);
		mSrcCur1Ref = -mSurgeImpedance / denom * (volt1 + (mSurgeImpedance - mResistance/4) * cur1)
			- mResistance/4 / denom * (volt2 + (mSurgeImpedance - mResistance/4) * cur2);
	}
	mSrcCur1->set(mSrcCur1Ref);
	mSrcCur2->set(mSrcCur1Ref);
}

void DecouplingLine::PreStep::execute(Real time, Int timeStepCount) {
	mLine.step(time, timeStepCount);
}

void DecouplingLine::postStep() {
	// Update ringbuffers with new values
	mVolt1[mBufIdx] = -mRes1->intfVoltage()(0, 0);
	mVolt2[mBufIdx] = -mRes2->intfVoltage()(0, 0);
	mCur1[mBufIdx] = -mRes1->intfCurrent()(0, 0) + mSrcCur1->get();
	mCur2[mBufIdx] = -mRes2->intfCurrent()(0, 0) + mSrcCur2->get();

	mBufIdx++;
	if (mBufIdx == mBufSize)
		mBufIdx = 0;
}

void DecouplingLine::PostStep::execute(Real time, Int timeStepCount) {
	mLine.postStep();
}

Task::List DecouplingLine::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<PostStep>(*this)});
}

Component::List DecouplingLine::getLineComponents() {
	return Component::List({mRes1, mRes2, mSrc1, mSrc2});
}
