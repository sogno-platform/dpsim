/**
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include <typeinfo>
#include <iostream>
#include <vector>
#include <memory>

#include <cps/Logger.h>
#include <cps/IdentifiedObject.h>
#include <cps/TopologicalTerminal.h>
#include <cps/TopologicalNode.h>

namespace CPS {
	/// Base class for all electrical components that are
	/// connected to nodes via terminals
	class TopologicalComponent : public IdentifiedObject {
	public:
		enum Behaviour { Initialization, Simulation };
	protected:
		/// Determines the number of Terminals which can be connected to network Nodes
		UInt mNumTerminals = 0;
		/// Determines the number of virtual or internal Nodes
		UInt mNumVirtualNodes = 0;
		/// Component logger
		Logger::Log mSLog;
		/// Component logger control for internal variables
		Logger::Level mLogLevel;
		/// Determine state of the simulation, e.g. to implement
		/// special behavior for components during initialization
		Bool mBehaviour = Behaviour::Simulation;

	public:
		typedef std::shared_ptr<TopologicalComponent> Ptr;
		typedef std::vector<Ptr> List;

		/// Basic constructor that takes UID, name and log level
		TopologicalComponent(String uid, String name, Logger::Level logLevel = Logger::Level::off)
			: IdentifiedObject(uid, name), mLogLevel(logLevel) {
			mSLog = Logger::get(name, logLevel);
		}
		/// Basic constructor that takes name and log level and sets the UID to name as well
		TopologicalComponent(String name, Logger::Level logLevel = Logger::Level::off)
			: TopologicalComponent(name, name, logLevel) { }
		/// Destructor - does not do anything
		virtual ~TopologicalComponent() { }

		/// Returns nodes connected to this component
		virtual TopologicalNode::List topologicalNodes() = 0;
		/// Returns terminal that are part of the component
		virtual TopologicalTerminal::List topologicalTerminals() = 0;
		/// Set behavior of component, e.g. initialization
		void setBehaviour(Behaviour behaviour) { mBehaviour = behaviour; }
	};
}
