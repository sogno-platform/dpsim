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

#include <cps/SP/SP_Ph1_NetworkInjection.h>

using namespace CPS;


SP::Ph1::externalGridInjection::externalGridInjection(String uid, String name,
    Logger::Level logLevel) : PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(1);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);
    
    addAttribute<Real>("V_set", &mVoltageSetPoint, Flags::read | Flags::write);
    addAttribute<Real>("V_set_pu", &mVoltageSetPointPerUnit, Flags::read | Flags::write);
	addAttribute<Real>("p_inj", &mActivePowerInjection, Flags::read | Flags::write);
	addAttribute<Real>("q_inj", &mReactivePowerInjection, Flags::read | Flags::write);
	
	// MNA attributes
	addAttribute<Complex>("V_ref", Flags::read | Flags::write);
	addAttribute<Real>("f_src", Flags::read | Flags::write);
}

// #### Powerflow section ####

void SP::Ph1::externalGridInjection::setParameters(Real vSetPointPerUnit) {
	attribute<Real>("V_set_pu")->set(vSetPointPerUnit);
}

void SP::Ph1::externalGridInjection::modifyPowerFlowBusType(PowerflowBusType powerflowBusType) {
	mPowerflowBusType = powerflowBusType;
}

void SP::Ph1::externalGridInjection::updatePowerInjection(Complex powerInj) {
	mActivePowerInjection = powerInj.real();
	mReactivePowerInjection = powerInj.imag();
}

// #### MNA Section ####

void SP::Ph1::externalGridInjection::setParameters(Complex voltageRef, Real srcFreq) {
	attribute<Complex>("V_ref")->set(voltageRef);
	attribute<Real>("f_src")->set(srcFreq);
}

PowerComponent<Complex>::Ptr SP::Ph1::externalGridInjection::clone(String name) {
	auto copy = externalGridInjection::make(name, mLogLevel);
	copy->setParameters(attribute<Complex>("V_ref")->get());
	return copy;
}

void SP::Ph1::externalGridInjection::initialize(Matrix frequencies) {
	checkForUnconnectedTerminals();

	mFrequencies = frequencies;
	mNumFreqs = mFrequencies.size();

	mIntfVoltage = MatrixComp::Zero(1, mNumFreqs);
	mIntfCurrent = MatrixComp::Zero(1, mNumFreqs);
}

void SP::Ph1::externalGridInjection::initializeFromPowerflow(Real frequency) {
	mVoltageRef = attribute<Complex>("V_ref");
	mSrcFreq = attribute<Real>("f_src");
	if (mVoltageRef->get() == Complex(0, 0))
		//mVoltageRef->set(Complex(std::abs(initialSingleVoltage(0).real()), std::abs(initialSingleVoltage(0).imag())));
		mVoltageRef->set(initialSingleVoltage(0));
	mSLog->info(
		"\n--- Initialization from node voltages ---"
		"\nVoltage across: {:e}<{:e}"
		"\nTerminal 0 voltage: {:e}<{:e}"
		"\n--- Initialization from node voltages ---",
		std::abs(mVoltageRef->get()), std::arg(mVoltageRef->get()),
		std::abs(initialSingleVoltage(0)), std::arg(initialSingleVoltage(0)));
}

// #### MNA functions ####

void SP::Ph1::externalGridInjection::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mIntfVoltage(0, 0) = mVoltageRef->get();
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void SP::Ph1::externalGridInjection::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(0), Complex(1, 0));
	Math::setMatrixElement(systemMatrix, simNode(0), mVirtualNodes[0]->simNode(), Complex(1, 0));
	mSLog->info("-- Matrix Stamp ---");
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., simNode(0), mVirtualNodes[0]->simNode());
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->simNode(), simNode(0));
}


void SP::Ph1::externalGridInjection::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// TODO: Is this correct with two nodes not gnd?
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0, 0));
	SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
		Logger::complexToString(mIntfVoltage(0, 0)), mVirtualNodes[0]->simNode());
}

void SP::Ph1::externalGridInjection::updateVoltage(Real time) {
	if (mSrcFreq->get() < 0) {
		mIntfVoltage(0, 0) = mVoltageRef->get();
	}
	else {
		mIntfVoltage(0, 0) = Complex(
			Math::abs(mVoltageRef->get()) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())),
			Math::abs(mVoltageRef->get()) * sin(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())));
	}
}

void SP::Ph1::externalGridInjection::MnaPreStep::execute(Real time, Int timeStepCount) {
	mExternalGridInjection.updateVoltage(time);
	mExternalGridInjection.mnaApplyRightSideVectorStamp(mExternalGridInjection.mRightVector);
}


void SP::Ph1::externalGridInjection::MnaPostStep::execute(Real time, Int timeStepCount) {
	mExternalGridInjection.mnaUpdateCurrent(*mLeftVector);
}

void SP::Ph1::externalGridInjection::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->simNode());
}

void SP::Ph1::externalGridInjection::daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off) {
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

	int Pos1 = simNode(0);
	int Pos2 = simNode(1);
	int c_offset = off[0] + off[1]; //current offset for component
	int n_offset_1 = c_offset + Pos1 + 1;// current offset for first nodal equation
	int n_offset_2 = c_offset + Pos2 + 1;// current offset for second nodal equation
	resid[c_offset] = (state[Pos2] - state[Pos1]) - state[c_offset]; // Voltage equation for Resistor
	//resid[++c_offset] = ; //TODO : add inductance equation
	resid[n_offset_1] += mIntfCurrent(0, 0).real();
	resid[n_offset_2] += mIntfCurrent(0, 0).real();
	off[1] += 1;
}

Complex SP::Ph1::externalGridInjection::daeInitialize() {
	mIntfVoltage(0, 0) = mVoltageRef->get();
	return mVoltageRef->get();
}
