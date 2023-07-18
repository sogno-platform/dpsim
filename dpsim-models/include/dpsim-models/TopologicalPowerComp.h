/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <typeinfo>
#include <iostream>
#include <vector>
#include <memory>

#include <dpsim-models/Logger.h>
#include <dpsim-models/IdentifiedObject.h>
#include <dpsim-models/TopologicalTerminal.h>
#include <dpsim-models/TopologicalNode.h>

namespace CPS {
	/// Base class for all electrical components that are
	/// connected to nodes via terminals
	class TopologicalPowerComp : public IdentifiedObject {
	public:
		enum Behaviour { Initialization, MNASimulation, PFSimulation };
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
		Behaviour mBehaviour = Behaviour::MNASimulation;
		/// Flag indicating that parameters are set via setParameters() function
		bool mParametersSet = false;

	public:
		typedef std::shared_ptr<TopologicalPowerComp> Ptr;
		typedef std::vector<Ptr> List;

		/// Basic constructor that takes UID, name and log level
		TopologicalPowerComp(String uid, String name, Logger::Level logLevel = Logger::Level::off)
			: IdentifiedObject(uid, name),
			/* We also want to set the CLI loglevel according to the logLevel
			 * std::max(Logger::Level::info, logLevel). But because of excessive
			 * logging to Level::info that is currently infeasible. */
			mSLog(Logger::get(Logger::LoggerType::COMPONENT, name, logLevel, Logger::Level::warn)),
			mLogLevel(logLevel) { }

		/// Basic constructor that takes name and log level and sets the UID to name as well
		TopologicalPowerComp(String name, Logger::Level logLevel = Logger::Level::off)
			: TopologicalPowerComp(name, name, logLevel) { }
		/// Destructor - does not do anything
		virtual ~TopologicalPowerComp() { }

		/// Returns nodes connected to this component
		virtual TopologicalNode::List topologicalNodes() = 0;
		/// Returns terminal that are part of the component
		virtual TopologicalTerminal::List topologicalTerminals() = 0;
		/// Set behavior of component, e.g. initialization
		void setBehaviour(Behaviour behaviour) {
			mBehaviour = behaviour;
			if (mBehaviour == Behaviour::Initialization)
				SPDLOG_LOGGER_DEBUG(mSLog, "Set component behaviour to Initialization");
			else if (mBehaviour == Behaviour::PFSimulation)
				SPDLOG_LOGGER_DEBUG(mSLog, "Set component behaviour to PFSimulation");
			else if (mBehaviour == Behaviour::MNASimulation)
				SPDLOG_LOGGER_DEBUG(mSLog, "Set component behaviour to MNASimulation");
			else
				SPDLOG_LOGGER_WARN(mSLog, "Set component behaviour not fully supported yet");
		}
	};
}
