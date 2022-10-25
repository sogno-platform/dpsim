/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/pybind/SignalComponents.h>
#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <dpsim-models/IdentifiedObject.h>
#include <DPsim.h>
#include <dpsim-models/CSVReader.h>
#include <dpsim/pybind/Utils.h>

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

    py::class_<CPS::Base::ExciterParameters>(mSignal, "ExciterParameters")
        .def_readwrite("Tr", &CPS::Base::ExciterParameters::Tr)
        .def_readwrite("Tb", &CPS::Base::ExciterParameters::Tb)
        .def_readwrite("Tc", &CPS::Base::ExciterParameters::Tc)
        .def_readwrite("Ta", &CPS::Base::ExciterParameters::Ta)
        .def_readwrite("Ka", &CPS::Base::ExciterParameters::Ka)
        .def_readwrite("Tef", &CPS::Base::ExciterParameters::Tef)
        .def_readwrite("Kef", &CPS::Base::ExciterParameters::Kef)
        .def_readwrite("Tf", &CPS::Base::ExciterParameters::Tf)
        .def_readwrite("Kf", &CPS::Base::ExciterParameters::Kf)
        .def_readwrite("Aef", &CPS::Base::ExciterParameters::Aef)
        .def_readwrite("Bef", &CPS::Base::ExciterParameters::Bef)
        .def_readwrite("MaxVr", &CPS::Base::ExciterParameters::MaxVr)
        .def_readwrite("MinVr", &CPS::Base::ExciterParameters::MinVr);
        
    py::class_<CPS::Signal::ExciterDC1Simp, std::shared_ptr<CPS::Signal::ExciterDC1Simp>, CPS::SimSignalComp>(mSignal, "Exciter", py::multiple_inheritance())
        .def(py::init<std::string>())
        .def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::Signal::ExciterDC1Simp::setParameters, "exciter_parameters"_a);

    py::class_<CPS::Signal::TurbineGovernorType1, std::shared_ptr<CPS::Signal::TurbineGovernorType1>, CPS::SimSignalComp>(mSignal, "TurbineGovernorType1", py::multiple_inheritance())
        .def(py::init<std::string>())
        .def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::Signal::TurbineGovernorType1::setParameters, "T3"_a, "T4"_a, "T5"_a, "Tc"_a, "Ts"_a, "R"_a, "Tmin"_a, "Tmax"_a, "OmRef"_a);
}
