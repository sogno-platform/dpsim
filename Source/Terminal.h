/** Base component
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

#include "Definitions.h"

namespace DPsim {
	class Node;
	class Component;

	class Terminal {
	public:
		String mRID;
		Real mActivePower;
		Real mReactivePower;
		std::shared_ptr<Node> mNode;
		std::shared_ptr<Component> mComponent;
		Terminal(String rid) : mRID(rid) {}
	};
}
