/** PQ Load
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
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

PQLoad::PQLoad(std::string name, int src, int dest, Real p, Real q, Real volt, Real angle) : RxLine(name, src, dest, 1, 1) {
	// we need the system frequency to calculate the impedance, so we initialize
	// it with the dummy value of 1+j1 here for now
	mActivePower = p;
	mReactivePower = q;
	mSvVoltage = volt;
	// the parameters of the RxLine shouldn't be modified directly; the face that
	// this component inherits from RxLine is just an implementation details that
	// may change
	attrMap.erase(attrMap.find("resistance"));
	attrMap.erase(attrMap.find("inductance"));
	attrMap["activePower"] = {AttrReal, &this->mActivePower};
	attrMap["reactivePower"] = {AttrReal, &this->mReactivePower};
	attrMap["svVoltage"] = {AttrReal, &this->mSvVoltage};
}

void PQLoad::init(Real om, Real dt) {
	Real abs = mActivePower*mActivePower+mReactivePower*mReactivePower;
	mResistance = mSvVoltage*mSvVoltage*mActivePower/abs;
	mConductance = 1.0 / mResistance;
	Real reactance = mSvVoltage*mSvVoltage*mReactivePower/abs;
	mInductance = reactance/om;
	RxLine::init(om, dt);
}

void PQLoad::applySystemMatrixStamp(SystemModel& system) {
	// powers / svvoltage might have changed, so update them
	Real abs = mActivePower*mActivePower+mReactivePower*mReactivePower;
	mResistance = mSvVoltage*mSvVoltage*mActivePower/abs;
	mConductance = 1.0 / mResistance;
	Real reactance = mSvVoltage*mSvVoltage*mReactivePower/abs;
	mInductance = reactance/system.getOmega();
	RxLine::applySystemMatrixStamp(system);
}
