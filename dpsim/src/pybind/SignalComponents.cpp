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

    // Exciters
    py::class_<CPS::Signal::ExciterDC1, std::shared_ptr<CPS::Signal::ExciterDC1>, CPS::Base::Exciter>(mSignal, "ExciterDC1", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off);
    py::class_<CPS::Signal::ExciterDC1Simp, std::shared_ptr<CPS::Signal::ExciterDC1Simp>, CPS::Base::Exciter>(mSignal, "ExciterDC1Simp", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off);
    py::class_<CPS::Signal::ExciterST1Simp, std::shared_ptr<CPS::Signal::ExciterST1Simp>, CPS::Base::Exciter>(mSignal, "ExciterST1Simp", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off);
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
    py::class_<CPS::Signal::ExciterST1Parameters, std::shared_ptr<CPS::Signal::ExciterST1Parameters>, CPS::Base::ExciterParameters>(mSignal, "ExciterST1Parameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("Tr", &CPS::Signal::ExciterST1Parameters::Tr)
        .def_readwrite("Ka", &CPS::Signal::ExciterST1Parameters::Ka)
        .def_readwrite("MaxVa", &CPS::Signal::ExciterST1Parameters::MaxVa)
        .def_readwrite("MinVa", &CPS::Signal::ExciterST1Parameters::MinVa);

    // PSS
    py::class_<CPS::Signal::PSS1A, std::shared_ptr<CPS::Signal::PSS1A>, CPS::Base::PSS>(mSignal, "PSS1A", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off);

    py::class_<CPS::Signal::PSS1AParameters, std::shared_ptr<CPS::Signal::PSS1AParameters>, CPS::Base::PSSParameters>(mSignal, "PSS1AParameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("Kp", &CPS::Signal::PSS1AParameters::Kp)
        .def_readwrite("Kv", &CPS::Signal::PSS1AParameters::Kv)
        .def_readwrite("Kw", &CPS::Signal::PSS1AParameters::Kw)
        .def_readwrite("T1", &CPS::Signal::PSS1AParameters::T1)
        .def_readwrite("T2", &CPS::Signal::PSS1AParameters::T2)
        .def_readwrite("T3", &CPS::Signal::PSS1AParameters::T3)
        .def_readwrite("T4", &CPS::Signal::PSS1AParameters::T4)
        .def_readwrite("Tw", &CPS::Signal::PSS1AParameters::Tw)
        .def_readwrite("Vs_max", &CPS::Signal::PSS1AParameters::Vs_max)
        .def_readwrite("Vs_min", &CPS::Signal::PSS1AParameters::Vs_min);
    
    // Governos
    py::class_<CPS::Signal::TurbineGovernorType1, std::shared_ptr<CPS::Signal::TurbineGovernorType1>, CPS::Base::Governor>(mSignal, "TurbineGovernorType1", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off);
    py::class_<CPS::Signal::HydroTurbineGovernor, std::shared_ptr<CPS::Signal::HydroTurbineGovernor>, CPS::Base::Governor>(mSignal, "HydroTurbineGovernor", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off);
    py::class_<CPS::Signal::SteamTurbineGovernor, std::shared_ptr<CPS::Signal::SteamTurbineGovernor>, CPS::Base::Governor>(mSignal, "SteamTurbineGovernor", py::multiple_inheritance())
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
    py::class_<CPS::Signal::HydroGorvernorParameters, std::shared_ptr<CPS::Signal::HydroGorvernorParameters>, CPS::Base::GovernorParameters>(mSignal, "HydroGorvernorParameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("R", &CPS::Signal::HydroGorvernorParameters::R)
        .def_readwrite("T1", &CPS::Signal::HydroGorvernorParameters::T1)
        .def_readwrite("T2", &CPS::Signal::HydroGorvernorParameters::T2)
        .def_readwrite("T3", &CPS::Signal::HydroGorvernorParameters::T3)
        .def_readwrite("Pmax", &CPS::Signal::HydroGorvernorParameters::Pmax)
        .def_readwrite("Pmin", &CPS::Signal::HydroGorvernorParameters::Pmin)
        .def_readwrite("OmRef", &CPS::Signal::HydroGorvernorParameters::OmRef);
    py::class_<CPS::Signal::SteamGorvernorParameters, std::shared_ptr<CPS::Signal::SteamGorvernorParameters>, CPS::Base::GovernorParameters>(mSignal, "SteamGorvernorParameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("R", &CPS::Signal::SteamGorvernorParameters::R)
        .def_readwrite("T2", &CPS::Signal::SteamGorvernorParameters::T2)
        .def_readwrite("T3", &CPS::Signal::SteamGorvernorParameters::T3)
        .def_readwrite("dPmax", &CPS::Signal::SteamGorvernorParameters::dPmax)
        .def_readwrite("dPmin", &CPS::Signal::SteamGorvernorParameters::dPmin)
        .def_readwrite("Pmax", &CPS::Signal::SteamGorvernorParameters::Pmax)
        .def_readwrite("Pmin", &CPS::Signal::SteamGorvernorParameters::Pmin)
        .def_readwrite("OmRef", &CPS::Signal::SteamGorvernorParameters::OmRef);

    // Turbines
    py::class_<CPS::Signal::HydroTurbine, std::shared_ptr<CPS::Signal::HydroTurbine>, CPS::Base::Turbine>(mSignal, "HydroTurbine", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off);
    py::class_<CPS::Signal::SteamTurbine, std::shared_ptr<CPS::Signal::SteamTurbine>, CPS::Base::Turbine>(mSignal, "SteamTurbine", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off);

    py::class_<CPS::Signal::HydroTurbineParameters, std::shared_ptr<CPS::Signal::HydroTurbineParameters>, CPS::Base::TurbineParameters>(mSignal, "HydroTurbineParameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("Tw", &CPS::Signal::HydroTurbineParameters::Tw);
    py::class_<CPS::Signal::SteamTurbineParameters, std::shared_ptr<CPS::Signal::SteamTurbineParameters>, CPS::Base::TurbineParameters>(mSignal, "SteamTurbineParameters", py::multiple_inheritance())
        .def(py::init())
        .def_readwrite("Fhp", &CPS::Signal::SteamTurbineParameters::Fhp)
        .def_readwrite("Fip", &CPS::Signal::SteamTurbineParameters::Fip)
        .def_readwrite("Flp", &CPS::Signal::SteamTurbineParameters::Flp)
        .def_readwrite("Tch", &CPS::Signal::SteamTurbineParameters::Tch)
        .def_readwrite("Trh", &CPS::Signal::SteamTurbineParameters::Trh)
        .def_readwrite("Tco", &CPS::Signal::SteamTurbineParameters::Tco);
}
