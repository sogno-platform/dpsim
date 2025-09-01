/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/Config.h>
#include <dpsim/Simulation.h>
#include <dpsim/Utils.h>

#ifndef _MSC_VER
#include <dpsim/RealTimeDataLogger.h>
#include <dpsim/RealTimeSimulation.h>
#endif

#include <dpsim-models/Components.h>
#include <dpsim-models/Logger.h>

#ifdef WITH_CIM
#include <dpsim-models/CIM/Reader.h>
#endif

#ifdef WITH_OPENMP
#include <dpsim/OpenMPLevelScheduler.h>
#endif

namespace DPsim {
// #### CPS for users ####
using SystemTopology = CPS::SystemTopology;
using SystemNodeList = CPS::TopologicalNode::List;
using SystemComponentList = CPS::IdentifiedObject::List;
using Logger = CPS::Logger;
using Domain = CPS::Domain;
using PhaseType = CPS::PhaseType;
#ifdef WITH_CIM
using CIMReader = CPS::CIM::Reader;
#endif
} // namespace DPsim
