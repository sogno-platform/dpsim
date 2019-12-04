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

#pragma once

#include <cps/PowerComponent.h>


namespace CPS {
namespace SP {
namespace Ph1 {

	class PQNode: public PowerComponent<Complex>, public SharedFactory<PQNode>{
	private:
		Real mPowerNom;
		Real mReactivePowerNom;
		Real mPower;
		Real mReactivePower;


		Real mVoltageAbsPerUnit;
		Complex mVoltagePerUnit;

	public:
		PQNode(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);

        PQNode(String uid, String name, Real power, Real reactive_power,
            Logger::Level logLevel = Logger::Level::off);


		void initializeFromTerminal();
		void setPerUnitSystem();

	};




}
}
}
