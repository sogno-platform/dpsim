/**
 *
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

#include <cps/Config.h>
#include <cps/Terminal.h>
#include <cps/Node.h>
#include <cps/Logger.h>
#include <cps/Definitions.h>
#include <cps/MathUtils.h>
#include <cps/PtrFactory.h>
#include <cps/AttributeList.h>
#include <cps/Task.h>
#include <cps/SP/SP_Ph1_PVNode.h>
#include <cps/SP/SP_Ph1_PQNode.h>
#include <cps/SP/SP_Ph1_VDNode.h>

namespace CPS {
	/// Common base class of all Component templates.
	class PFSolverInterfaceBus {
	protected:
	Task::List mPFTasks;

	public:
		typedef std::shared_ptr<PFSolverInterfaceBus> Ptr;
		typedef std::vector<Ptr> List;

		std::shared_ptr<CPS::SP::Ph1::PQNode>mPQ;
		std::shared_ptr<CPS::SP::Ph1::PVNode>mPV;
		std::shared_ptr<CPS::SP::Ph1::VDNode>mVD;

        /// Define the type of bus the component is modelled by
		PowerflowBusType mPowerflowBusType;
		PFSolverInterfaceBus() {};

		virtual void modifyPowerFlowBusType(PowerflowBusType powerflowBusType)=0;
		virtual void pfBusInitialize(){
			mPFTasks.clear();
		}
		const Task::List& pfTasks() {
			return mPFTasks;
		}

		};

	}



