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

#include <iostream>
#include <vector>
#include <memory>

#include "PtrFactory.h"
#include "SystemModel.h"
#include "Definitions.h"
#include "MathLibrary.h"
#include "Attribute.h"

namespace DPsim {

	/// Base class for all components that might be added to the matrix.
	class Component {

	protected:
		/// Component logger
		Logger mLog;
		/// Component logger control for internal variables
		Logger::Level mLogLevel;
		/// Component name
		String mName;
		/// Component node 1
		Int mNode1;
		/// Component node 2
		Int mNode2;
		/// Component node 3
		Int mNode3;

		/// Determines if the component has a virtual node
		Int mNumVirtualNodes = 0;
		/// Index of virtual node
		std::vector<Int> mVirtualNodes;
		/// Map of all attributes that should be exported to the Python interface
		AttributeBase::Map mAttributes;

	public:
		class InvalidAttributeException { };

		typedef std::shared_ptr<Component> Ptr;
		typedef std::vector<Ptr> List;

		/// Creates a new component with basic features: name and nodes
		/// Decrementing the node number is default so that the user can use zero for the ground node. It needs to be
		/// deactivated for subcomponents that are created inside other components since otherwise the node number
		/// would be decremented twice.
		Component(String name, Int node1, Int node2, Logger::Level logLevel = Logger::Level::NONE)
			: mLog("Logs/" + name + ".log", logLevel) {
			mName = name;
			mNode1 = node1;
			mNode2 = node2;
			mLogLevel = logLevel;

			mAttributes["name"]  = Attribute<String>::make(&mName, Flags::read);
			mAttributes["node1"] = Attribute<Int>::make(&mNode1, Flags::read);
			mAttributes["node2"] = Attribute<Int>::make(&mNode2, Flags::read);
		}

		Component(String name, Int node1, Int node2, Int node3, Logger::Level loglevel = Logger::Level::NONE)
			: Component(name, node1, node2, loglevel) {
			mNode3 = node3;
			mAttributes["node3"] = Attribute<Int>::make(&mNode3, Flags::read);
		}

		virtual ~Component() { }

		/// get value of node1
		Int getNode1() { return mNode1; }
		/// get value of node2
		Int getNode2() { return mNode2; }
		/// get value of node3
		Int getNode3() { return mNode3; }
		/// Returns true if virtual node number is greater than zero.
		Bool hasVirtualNodes() { return mNumVirtualNodes > 0; }
		/// Returns true if virtual node number is greater than zero.
		Int getVirtualNodesNum() { return mNumVirtualNodes; }
		/// get virtual node
		Int getVirtualNode(Int nodeNum) { return mVirtualNodes[nodeNum]; }
		/// set virtual node
		void setVirtualNode(Int nodeNum, Int virtualNode) { mVirtualNodes[nodeNum] = virtualNode; }

		// Return pointer to an attribute
		AttributeBase::Ptr findAttribute(const String &name) {
			auto it = mAttributes.find(name);
			if (it == mAttributes.end())
				throw InvalidAttributeException();

			return it->second;
		}

		String getName() { return mName; }
		String getType();

		/// Initializes variables of components
		virtual void initialize(SystemModel& system) { }

		/// Stamps conductance matrix
		virtual void applySystemMatrixStamp(SystemModel& system) = 0;

		/// Stamps current source vector
		virtual void applyRightSideVectorStamp(SystemModel& system) { }

		/// Upgrade values on the current source vector
		virtual void step(SystemModel& system, Real time) { }

		/// Upgrade variable values based on the solution of the step
		virtual void postStep(SystemModel& system) { }
	};
}
