/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <SignalComponents.h>
#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <cps/IdentifiedObject.h>
#include <cps/CIM/Reader.h>
#include <DPsim.h>
#include <cps/CSVReader.h>
#include <Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addSignalComponents(py::module_ mSignal) {

    py::class_<CPS::TopologicalSignalComp, std::shared_ptr<CPS::TopologicalSignalComp>, CPS::IdentifiedObject>(mSignal, "TopologicalSignalComp");
	py::class_<CPS::SimSignalComp, std::shared_ptr<CPS::SimSignalComp>, CPS::TopologicalSignalComp>(mSignal, "SimSignalComp");

    py::class_<CPS::Signal::DecouplingLine, std::shared_ptr<CPS::Signal::DecouplingLine>, CPS::SimSignalComp>(mSignal, "DecouplingLine", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::Signal::DecouplingLine::setParameters, "node_1"_a, "node_2"_a, "resistance"_a, "inductance"_a, "capacitance"_a)
		.def("get_line_components", &CPS::Signal::DecouplingLine::getLineComponents);

    py::class_<CPS::Signal::DecouplingLineEMT, std::shared_ptr<CPS::Signal::DecouplingLineEMT>, CPS::SimSignalComp>(mSignal, "DecouplingLineEMT", py::multiple_inheritance())
        .def(py::init<std::string>())
        .def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::Signal::DecouplingLineEMT::setParameters, "node_1"_a, "node_2"_a, "resistance"_a, "inductance"_a, "capacitance"_a)
        .def("get_line_components", &CPS::Signal::DecouplingLineEMT::getLineComponents);

}
