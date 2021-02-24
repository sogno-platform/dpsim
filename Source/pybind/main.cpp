/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>

#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <cps/IdentifiedObject.h>
#include <cps/CIM/Reader.h>
#include <DPsim.h>

#include <cps/CSVReader.h>

namespace py = pybind11;

PYBIND11_MODULE(dpsimpy, m) {
    m.doc() = R"pbdoc(
        Pybind11 DPsim plugin
        -----------------------
        .. currentmodule:: dpsimpy
        .. autosummary::
           :toctree: _generate
    )pbdoc";

    py::class_<DPsim::Simulation>(m, "Simulation")
	    .def(py::init<std::string>())
		.def("name", &DPsim::Simulation::name)
		.def("set_time_step", &DPsim::Simulation::setTimeStep)
		.def("set_final_time", &DPsim::Simulation::setFinalTime)
		.def("add_logger", &DPsim::Simulation::addLogger)
		.def("set_system", &DPsim::Simulation::setSystem)
		.def("run", &DPsim::Simulation::run)
		.def("set_solver", &DPsim::Simulation::setSolverType)
		.def("set_domain", &DPsim::Simulation::setDomain)
		.def("start", &DPsim::Simulation::start)
		.def("next", &DPsim::Simulation::next)
		.def("set_attribute", static_cast<void (DPsim::Simulation::*)(const std::string&, const std::string&, CPS::Real)>(&DPsim::Simulation::setAttribute))
		.def("set_attribute", static_cast<void (DPsim::Simulation::*)(const std::string&, const std::string&, CPS::Complex)>(&DPsim::Simulation::setAttribute))
		.def("get_real_attribute", &DPsim::Simulation::getRealAttribute)
		.def("get_complex_attribute", &DPsim::Simulation::getComplexAttribute);

	py::class_<DPsim::RealTimeSimulation>(m, "RealTimeSimulation")
	    .def(py::init<std::string>())
		.def("name", &DPsim::Simulation::name)
		.def("set_time_step", &DPsim::Simulation::setTimeStep)
		.def("set_final_time", &DPsim::Simulation::setFinalTime)
		.def("add_logger", &DPsim::Simulation::addLogger)
		.def("set_system", &DPsim::Simulation::setSystem)
		.def("run", &DPsim::Simulation::run)
		.def("set_solver", &DPsim::Simulation::setSolverType)
		.def("set_domain", &DPsim::Simulation::setDomain);

	py::class_<CPS::SystemTopology, std::shared_ptr<CPS::SystemTopology>>(m, "SystemTopology")
        .def(py::init<CPS::Real, CPS::TopologicalNode::List, CPS::IdentifiedObject::List>())
		.def(py::init<CPS::Real>())
		.def("add", &DPsim::SystemTopology::addComponent)
		.def("_repr_svg_", &DPsim::SystemTopology::render)
		.def("render_to_file", &DPsim::SystemTopology::renderToFile)
		.def_readwrite("nodes", &DPsim::SystemTopology::mNodes);

	py::class_<DPsim::DataLogger, std::shared_ptr<DPsim::DataLogger>>(m, "Logger")
        .def(py::init<std::string>())
		.def("log_attribute", (void (DPsim::DataLogger::*)(const CPS::String &, const CPS::String &, CPS::IdentifiedObject::Ptr)) &DPsim::DataLogger::addAttribute);

	py::class_<CPS::IdentifiedObject, std::shared_ptr<CPS::IdentifiedObject>>(m, "IdentifiedObject")
		.def("name", &CPS::IdentifiedObject::name);

	py::enum_<CPS::Domain>(m, "Domain")
		.value("SP", CPS::Domain::SP)
		.value("DP", CPS::Domain::DP)
		.value("EMT", CPS::Domain::EMT);

	py::enum_<CPS::PhaseType>(m, "PhaseType")
		.value("A", CPS::PhaseType::A)
		.value("B", CPS::PhaseType::B)
		.value("C", CPS::PhaseType::C)
		.value("ABC", CPS::PhaseType::ABC)
		.value("Single", CPS::PhaseType::Single);

	py::enum_<DPsim::Solver::Type>(m, "Solver")
		.value("MNA", DPsim::Solver::Type::MNA)
		.value("DAE", DPsim::Solver::Type::DAE)
		.value("NRP", DPsim::Solver::Type::NRP);

	py::enum_<CPS::Logger::Level>(m, "LogLevel")
		.value("trace", CPS::Logger::Level::trace)
		.value("debug", CPS::Logger::Level::debug)
		.value("info", CPS::Logger::Level::info)
		.value("warn", CPS::Logger::Level::warn)
		.value("err", CPS::Logger::Level::err)
		.value("critical", CPS::Logger::Level::critical)
		.value("off", CPS::Logger::Level::off);		

	py::enum_<CPS::CSVReader::Mode>(m, "CSVReaderMode")
		.value("AUTO", CPS::CSVReader::Mode::AUTO)
		.value("MANUAL", CPS::CSVReader::Mode::MANUAL);		

	py::enum_<CPS::CSVReader::DataFormat>(m, "CSVReaderFormat")
		.value("HHMMSS", CPS::CSVReader::DataFormat::HHMMSS)
		.value("SECONDS", CPS::CSVReader::DataFormat::SECONDS)
		.value("HOURS", CPS::CSVReader::DataFormat::HOURS)
		.value("MINUTES", CPS::CSVReader::DataFormat::MINUTES);
		
	py::class_<CPS::CIM::Reader>(m, "CIMReader")
		.def(py::init<std::string>())
		.def("loadCIM", (CPS::SystemTopology (CPS::CIM::Reader::*)(CPS::Real, const std::list<CPS::String> &, CPS::Domain, CPS::PhaseType)) &CPS::CIM::Reader::loadCIM);

	py::class_<CPS::CSVReader>(m, "CSVReader")
		.def(py::init<std::string, const std::string &, std::map<std::string, std::string> &, CPS::Logger::Level>())
		.def("assignLoadProfile", &CPS::CSVReader::assignLoadProfile);

	py::class_<CPS::TopologicalPowerComp, std::shared_ptr<CPS::TopologicalPowerComp>, CPS::IdentifiedObject>(m, "TopologicalPowerComp");
	py::class_<CPS::SimPowerComp<CPS::Complex>, std::shared_ptr<CPS::SimPowerComp<CPS::Complex>>, CPS::TopologicalPowerComp>(m, "SimPowerCompComplex");

	py::class_<CPS::TopologicalNode, std::shared_ptr<CPS::TopologicalNode>, CPS::IdentifiedObject>(m, "TopologicalNode");

	py::module mDP = m.def_submodule("dp", "dynamic phasor models");
	py::module mDPPh1 = mDP.def_submodule("ph1", "single phase dynamic phasor models");
	py::module mEMT = m.def_submodule("emt", "electromagnetic-transient models");
	py::module mEMTPh1 = mEMT.def_submodule("ph1", "single phase electromagnetic-transient models");

    py::class_<CPS::DP::SimNode, std::shared_ptr<CPS::DP::SimNode>, CPS::TopologicalNode>(mDP, "SimNode")
        .def(py::init<std::string>())
		.def_readonly_static("gnd", &CPS::DP::SimNode::GND);

	py::class_<CPS::DP::Ph1::CurrentSource, std::shared_ptr<CPS::DP::Ph1::CurrentSource>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "CurrentSource", py::multiple_inheritance())
        .def(py::init<std::string>())
        .def("set_parameters", &CPS::DP::Ph1::CurrentSource::setParameters)
		.def("connect", &CPS::DP::Ph1::CurrentSource::connect);

	py::class_<CPS::DP::Ph1::Resistor, std::shared_ptr<CPS::DP::Ph1::Resistor>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "Resistor", py::multiple_inheritance())
        .def(py::init<std::string>())
        .def("set_parameters", &CPS::DP::Ph1::Resistor::setParameters)
		.def("connect", &CPS::DP::Ph1::Resistor::connect);

	py::class_<CPS::DP::Ph1::Capacitor, std::shared_ptr<CPS::DP::Ph1::Capacitor>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "Capacitor", py::multiple_inheritance())
        .def(py::init<std::string>())
        .def("set_parameters", &CPS::DP::Ph1::Capacitor::setParameters)
		.def("connect", &CPS::DP::Ph1::Capacitor::connect);

	py::class_<CPS::DP::Ph1::Inductor, std::shared_ptr<CPS::DP::Ph1::Inductor>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "Inductor", py::multiple_inheritance())
        .def(py::init<std::string>())
        .def("set_parameters", &CPS::DP::Ph1::Inductor::setParameters)
		.def("connect", &CPS::DP::Ph1::Inductor::connect);



#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
