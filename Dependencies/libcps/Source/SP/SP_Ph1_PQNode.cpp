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

#include <cps/SP/SP_Ph1_PQNode.h>

using namespace CPS;

 SP::Ph1::PQNode::PQNode(String uid, String name,
	 Logger::Level logLevel)
	 : PowerComponent<Complex>(uid, name, logLevel) {

	addAttribute<Real>("P", &mPower, Flags::read | Flags::write);
	addAttribute<Real>("Q", &mReactivePower, Flags::read | Flags::write);
	addAttribute<Real>("P_nom", &mPowerNom, Flags::read | Flags::write);
	addAttribute<Real>("Q_nom", &mReactivePowerNom, Flags::read | Flags::write);

}
 SP::Ph1::PQNode::PQNode(String uid, String name, Real power, Real reactive_power,
	 Logger::Level logLevel):PQNode(uid,name,logLevel) {

	 mPower = power;
	 mPowerNom = power;
	 mReactivePower = reactive_power;
	 mReactivePowerNom = reactive_power;

     mSLog->info("Create PQ node for {} P={}, Q={}", name, mPowerNom, mReactivePowerNom);

 }

