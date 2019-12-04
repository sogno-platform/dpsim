/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

#include <cps/Component.h>
#include <cps/TopologicalTerminal.h>
#include <cps/TopologicalNode.h>

namespace CPS {

	class TopologicalComponent : public Component {
	protected:
		/// Determines the number of Terminals which can be connected to network Nodes
		UInt mNumTerminals = 0;
		/// Determines the number of virtual or internal Nodes
		UInt mNumVirtualNodes = 0;

	public:
		typedef std::shared_ptr<TopologicalComponent> Ptr;
		typedef std::vector<Ptr> List;

		using Component::Component;

		///
		virtual TopologicalNode::List topologicalNodes() = 0;
		///
		virtual TopologicalTerminal::List topologicalTerminals() = 0;
	};
}
