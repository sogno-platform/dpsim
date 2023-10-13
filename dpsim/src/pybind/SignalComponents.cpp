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

    py::class_<CPS::Signal::ExciterDC1Parameters, std::shared_ptr<CPS::Signal::ExciterDC1Parameters>, CPS::Base::ExciterParameters>(mSignal, "ExciterDC1Parameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("Tr", &CPS::Signal::ExciterDC1Parameters::Tr)
        .def_readwrite("Tb", &CPS::Signal::ExciterDC1Parameters::Tb)
        .def_readwrite("Tc", &CPS::Signal::ExciterDC1Parameters::Tc)
        .def_readwrite("Ta", &CPS::Signal::ExciterDC1Parameters::Ta)
        .def_readwrite("Ka", &CPS::Signal::ExciterDC1Parameters::Ka)
        .def_readwrite("Tef", &CPS::Signal::ExciterDC1Parameters::Tef)
        .def_readwrite("Kef", &CPS::Signal::ExciterDC1Parameters::Kef)
        .def_readwrite("Tf", &CPS::Signal::ExciterDC1Parameters::Tf)
        .def_readwrite("Kf", &CPS::Signal::ExciterDC1Parameters::Kf)
        .def_readwrite("Aef", &CPS::Signal::ExciterDC1Parameters::Aef)
        .def_readwrite("Bef", &CPS::Signal::ExciterDC1Parameters::Bef)
        .def_readwrite("MaxVa", &CPS::Signal::ExciterDC1Parameters::MaxVa)
        .def_readwrite("MinVa", &CPS::Signal::ExciterDC1Parameters::MinVa);

    py::class_<CPS::Signal::ExciterDC1SimpParameters, std::shared_ptr<CPS::Signal::ExciterDC1SimpParameters>, CPS::Base::ExciterParameters>(mSignal, "ExciterDC1SimpParameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("Tr", &CPS::Signal::ExciterDC1SimpParameters::Tr)
        .def_readwrite("Ta", &CPS::Signal::ExciterDC1SimpParameters::Ta)
        .def_readwrite("Ka", &CPS::Signal::ExciterDC1SimpParameters::Ka)
        .def_readwrite("Tef", &CPS::Signal::ExciterDC1SimpParameters::Tef)
        .def_readwrite("Kef", &CPS::Signal::ExciterDC1SimpParameters::Kef)
        .def_readwrite("Tf", &CPS::Signal::ExciterDC1SimpParameters::Tf)
        .def_readwrite("Kf", &CPS::Signal::ExciterDC1SimpParameters::Kf)
        .def_readwrite("Aef", &CPS::Signal::ExciterDC1SimpParameters::Aef)
        .def_readwrite("Bef", &CPS::Signal::ExciterDC1SimpParameters::Bef)
        .def_readwrite("MaxVa", &CPS::Signal::ExciterDC1SimpParameters::MaxVa)
        .def_readwrite("MinVa", &CPS::Signal::ExciterDC1SimpParameters::MinVa);

    // Exciters
    py::class_<CPS::Signal::ExciterST1Parameters, std::shared_ptr<CPS::Signal::ExciterST1Parameters>, CPS::Base::ExciterParameters>(mSignal, "ExciterST1Parameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("Tr", &CPS::Signal::ExciterST1Parameters::Tr)
        .def_readwrite("Ka", &CPS::Signal::ExciterST1Parameters::Ka)
        .def_readwrite("MaxVa", &CPS::Signal::ExciterST1Parameters::MaxVa)
        .def_readwrite("MinVa", &CPS::Signal::ExciterST1Parameters::MinVa);

    // Governos
    py::class_<CPS::Signal::TurbineGovernorType1, std::shared_ptr<CPS::Signal::TurbineGovernorType1>, CPS::Base::Governor>(mSignal, "TurbineGovernorType1", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off);

    py::class_<CPS::Signal::TurbineGovernorType1Parameters, std::shared_ptr<CPS::Signal::TurbineGovernorType1Parameters>, CPS::Base::GovernorParameters>(mSignal, "TurbineGovernorType1Parameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("Pmax", &CPS::Signal::TurbineGovernorType1Parameters::Pmax)
        .def_readwrite("Pmin", &CPS::Signal::TurbineGovernorType1Parameters::Pmin)
        .def_readwrite("R", &CPS::Signal::TurbineGovernorType1Parameters::R)
        .def_readwrite("T3", &CPS::Signal::TurbineGovernorType1Parameters::T3)
        .def_readwrite("T4", &CPS::Signal::TurbineGovernorType1Parameters::T4)
        .def_readwrite("T5", &CPS::Signal::TurbineGovernorType1Parameters::T5)
        .def_readwrite("Tc", &CPS::Signal::TurbineGovernorType1Parameters::Tc)
        .def_readwrite("Ts", &CPS::Signal::TurbineGovernorType1Parameters::Ts)
        .def_readwrite("OmRef", &CPS::Signal::TurbineGovernorType1Parameters::OmRef);
}
