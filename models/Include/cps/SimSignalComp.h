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

#include <cps/Task.h>
#include <cps/TopologicalSignalComp.h>

namespace CPS {
	/// Base class for all signal type components
	/// that have only unidirectional connections
	class SimSignalComp : public TopologicalSignalComp {
	public:
		enum Behaviour { Initialization, Simulation };
	protected:
		/// Determine state of the simulation, e.g. to implement
		/// special behavior for components during initialization
		Bool mBehaviour = Behaviour::Simulation;
	public:
		typedef std::shared_ptr<SimSignalComp> Ptr;
		typedef std::vector<Ptr> List;

		///
		SimSignalComp(String uid, String name, Logger::Level logLevel = Logger::Level::off)
			: TopologicalSignalComp(uid, name, logLevel) { }
		///
		SimSignalComp(String name, Logger::Level logLevel = Logger::Level::off)
			: SimSignalComp(name, name, logLevel) { }
		///
		virtual ~SimSignalComp() { }

		///
		virtual void initialize(Real timeStep) { }
		///
		virtual void initialize(Real omega, Real timeStep) { initialize(timeStep); }
		///
		virtual Task::List getTasks() {
			return Task::List();
		}
		/// Set behavior of component, e.g. initialization
		void setBehaviour(Behaviour behaviour) { mBehaviour = behaviour; }
	};
}
