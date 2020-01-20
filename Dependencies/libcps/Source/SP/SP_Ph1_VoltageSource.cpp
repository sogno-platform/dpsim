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

#include <cps/SP/SP_Ph1_VoltageSource.h>

using namespace CPS;

SP::Ph1::VoltageSource::VoltageSource(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Complex>("V_ref", Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr SP::Ph1::VoltageSource::clone(String name) {
	auto copy = VoltageSource::make(name, mLogLevel);
	copy->setParameters(attribute<Complex>("V_ref")->get());
	return copy;
}

void SP::Ph1::VoltageSource::setParameters(Complex voltageRef) {
	attribute<Complex>("V_ref")->set(voltageRef);
	parametersSet = true;
}

void SP::Ph1::VoltageSource::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	mVoltageRef = attribute<Complex>("V_ref");

	if (mVoltageRef->get() == Complex(0, 0))
		mVoltageRef->set(initialSingleVoltage(1) - initialSingleVoltage(0));

}

void SP::Ph1::VoltageSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateSimNodes();
	mIntfVoltage(0, 0) = mVoltageRef->get();

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void SP::Ph1::VoltageSource::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), mVirtualNodes[0]->simNode(), Complex(-1, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(0), Complex(-1, 0));

	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(1), mVirtualNodes[0]->simNode(), Complex(1, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(1), Complex(1, 0));
	}

}

void SP::Ph1::VoltageSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0, 0));

}

void SP::Ph1::VoltageSource::updateVoltage(Real time) {
	// can't we just do nothing??
	// TODO: remove updateVoltage
	mIntfVoltage(0, 0) = mVoltageRef->get();
}

void SP::Ph1::VoltageSource::MnaPreStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.updateVoltage(time);
	mVoltageSource.mnaApplyRightSideVectorStamp(mVoltageSource.mRightVector);
}

void SP::Ph1::VoltageSource::MnaPostStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.mnaUpdateCurrent(*mLeftVector);
}

void SP::Ph1::VoltageSource::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->simNode());
}

void SP::Ph1::VoltageSource::daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off) {
	/* new state vector definintion:
		state[0]=node0_voltage
		state[1]=node1_voltage
		....
		state[n]=noden_voltage
		state[n+1]=component0_voltage
		state[n+2]=component0_inductance (not yet implemented)
		...
		state[m-1]=componentm_voltage
		state[m]=componentm_inductance
	*/

	//int Pos1 = simNode(0);
	//int Pos2 = simNode(1);
	//int c_offset = off[0] + off[1]; //current offset for component
	//int n_offset_1 = c_offset + Pos1 + 1;// current offset for first nodal equation
	//int n_offset_2 = c_offset + Pos2 + 1;// current offset for second nodal equation
	//resid[c_offset] = (state[Pos2] - state[Pos1]) - state[c_offset]; // Voltage equation for Resistor
	////resid[++c_offset] = ; //TODO : add inductance equation
	//resid[n_offset_1] += std::real(current());
	//resid[n_offset_2] += std::real(current());
	//off[1] += 1;
}

Complex SP::Ph1::VoltageSource::daeInitialize() {
	mIntfVoltage(0, 0) = mVoltageRef->get();
	return mVoltageRef->get();
}
