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

#include <typeinfo>
#include <iostream>
#include <vector>
#include <memory>

#include <cps/Logger.h>
#include <cps/IdentifiedObject.h>

namespace CPS {
	/// Base class of all objects running internal calculations and
	/// having states or measurements.
	class Component : public IdentifiedObject {
	public:
		enum Behaviour { Initialization, Simulation };

	protected:
		/// Component logger
		Logger::Log mSLog;
		/// Component logger control for internal variables
		Logger::Level mLogLevel;
		///
		Bool mBehaviour = Behaviour::Simulation;
	public:
		typedef std::shared_ptr<Component> Ptr;
		typedef std::vector<Ptr> List;

		/// Basic constructor that takes UID, name and log level
		Component(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Basic constructor that takes name and log level and sets the UID to name as well
		Component(String name, Logger::Level logLevel = Logger::Level::off);
		/// Destructor - does not do anything
		virtual ~Component() { }
		///
		void setBehaviour(Behaviour behaviour) { mBehaviour = behaviour; }
	};
}
