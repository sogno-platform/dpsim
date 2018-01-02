/** PQ Load
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
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

#include "PQLoadDP.h"

using namespace DPsim;

PQLoadDP::PQLoadDP(String name, Int node, Real activePower, Real reactivePower, Real volt, Real angle) : BaseComponent(name, node, 0) {
	// we need the system frequency to calculate the impedance, so we initialize
	// it with the dummy value of 1+j1 here for now
	mActivePower = activePower;
	mReactivePower = reactivePower;
	mSvVoltage = volt;
	attrMap["activePower"] = {AttrReal, &mActivePower};
	attrMap["reactivePower"] = {AttrReal, &mReactivePower};
	attrMap["svVoltage"] = {AttrReal, &mSvVoltage};
}

void PQLoadDP::init(Real om, Real dt) {
	Real abs = mActivePower*mActivePower + mReactivePower*mReactivePower;
	mResistance = mSvVoltage*mSvVoltage*mActivePower/abs;
	mConductance = 1.0 / mResistance;
	mReactance = mSvVoltage*mSvVoltage*mReactivePower/abs;
	mInductance = mReactance / om;

	inductor = std::make_shared<InductorDP>(mName + "_ind", mNode1, mNode2, mInductance);
	resistor = std::make_shared<ResistorDP>(mName + "_res", mNode1, mNode2, mResistance);
	inductor->init(om, dt);
	resistor->init(om, dt);
}

void PQLoadDP::applySystemMatrixStamp(SystemModel& system) {		
	// Add resistive part to system matrix
	resistor->applySystemMatrixStamp(system);

	// Add inductive part to system matrix
	inductor->applySystemMatrixStamp(system);	
}

void PQLoadDP::step(SystemModel& system, Real time) {
	inductor->step(system, time);
}


void PQLoadDP::postStep(SystemModel& system) {
	inductor->postStep(system);
}

Complex PQLoadDP::getCurrent(SystemModel& system) {
	return inductor->getCurrent(system);
}