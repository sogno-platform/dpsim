/** Voltage source base class
*
* @file
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

#pragma once

#include "EMT_SynchronGenerator_Simplified.h"

#include "Base.h"

namespace DPsim {
namespace Component {

	template <class T>
	class VoltageSourceBase : public Component::Base {

	protected:
		T mVoltage;

	public:
		VoltageSourceBase() { };

		/// Define paramenters of the voltage source
		VoltageSourceBase(String name, Int src, Int dest, T voltage)
			: Base(name, src, dest)
		{
			mVoltage = voltage;
		}

		/// Modified voltage
		void setVoltage(T voltage)
		{
			mVoltage = voltage;
		}
	};
}
}
