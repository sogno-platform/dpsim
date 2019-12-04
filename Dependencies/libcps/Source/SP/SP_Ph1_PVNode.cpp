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

#include <cps/SP/SP_Ph1_PVNode.h>

using namespace CPS;


SP::Ph1::PVNode::PVNode(String uid, String name,
	Logger::Level logLevel) : PowerComponent<Complex>(uid, name, logLevel) {

	addAttribute<Real>("P_set", &mPowerSetPoint, Flags::read | Flags::write);
	addAttribute<Real>("V_set", &mVoltageSetPoint, Flags::read | Flags::write);
	addAttribute<Real>("V_set_pu", &mVoltagePerUnit, Flags::read | Flags::write);

}


SP::Ph1::PVNode::PVNode(String uid, String name, Real power, Real vSetPoint, Real maxQ, Real ratedU, Real ratedS,
	Logger::Level logLevel)
	:PVNode(uid, name, logLevel) {

	mPowerSetPoint =  power;
	mVoltageSetPoint = vSetPoint;
	mRatedU = ratedU;

	mVoltagePerUnit = vSetPoint / ratedU;

	// maxQ=0 means no limit.
	mQLimit = maxQ;

	mSLog->info("Create PV node for {} P={}, V={}", name, mPowerSetPoint, mVoltageSetPoint);
}



