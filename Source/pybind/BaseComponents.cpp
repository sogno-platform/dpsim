/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <BaseComponents.h>
#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <cps/IdentifiedObject.h>
#include <cps/CIM/Reader.h>
#include <DPsim.h>
#include <cps/CSVReader.h>
#include <Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addBaseComponents(py::module_ mBase) {
    py::class_<CPS::Base::Ph1::Switch, std::shared_ptr<CPS::Base::Ph1::Switch>>(mBase, "Switch");
    py::class_<CPS::Base::Ph3::Switch, std::shared_ptr<CPS::Base::Ph3::Switch>>(mBase, "SwitchPh3");
}
