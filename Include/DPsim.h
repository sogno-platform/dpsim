/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <dpsim/Config.h>
#include <dpsim/Utils.h>
#include <dpsim/Simulation.h>

#ifndef _MSC_VER
  #include <dpsim/RealTimeSimulation.h>
#endif

#include <cps/Components.h>
#include <cps/Logger.h>

#ifdef WITH_SHMEM
  #include <cps/Interface.h>
#endif

#ifdef WITH_CIM
  #include <cps/CIM/Reader.h>
#endif

namespace DPsim {
	// #### CPS for users ####
	using SystemTopology = CPS::SystemTopology;
	using SystemNodeList = CPS::TopologicalNode::List;
	using SystemComponentList = CPS::Component::List;
	using Logger = CPS::Logger;
	using Domain = CPS::Domain;
	using PhaseType = CPS::PhaseType;
#ifdef WITH_CIM
	using CIMReader = CPS::CIM::Reader;
#endif
}
